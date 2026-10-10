// main.cpp
// encoder-relay: Pico encoder records from the Pi UART to the V5 Brain USB
// user port, one "ENC1:" line per record (encoder-record/encoder_record.h).
// No localization, no field knowledge, nothing sent to the Pico.
// Linux only. Log lines go to stderr (journald under systemd).

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <poll.h>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "brain_port.h"
#include "forwarder.h"
#include "serial_port.h"

namespace
{

using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::seconds;

constexpr milliseconds kRetry{1000};       // reopen attempts
constexpr milliseconds kPoll{20};          // longest wait in one pass
constexpr milliseconds kBrainStall{500};   // no write progress: drop queued output
constexpr std::size_t  kReadChunk = 512;   // bytes per read call

struct Options {
    std::string uart       = "/dev/ttyAMA0";
    int         baud       = 115200;
    std::string brain      = "auto";
    bool        echo_brain = false;
    int         status_s   = 10;
};

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) { g_stop = 1; }

void logLine(const char* format, ...) {
    std::fputs("encoder-relay: ", stderr);
    va_list args;
    va_start(args, format);
    std::vfprintf(stderr, format, args);
    va_end(args);
    std::fputc('\n', stderr);
}

void usage() {
    std::fputs(
        "usage: encoder-relay [options]\n"
        "  --uart PATH      Pico UART (default /dev/ttyAMA0)\n"
        "  --baud N         Pico UART baud (default 115200)\n"
        "  --brain auto|PATH  Brain USB user port; auto finds USB 2888:0501\n"
        "                   interface 02 (default auto)\n"
        "  --echo-brain     copy the Brain's console output to stdout (raw COBS\n"
        "                   bytes, for spectagatr --source file --replay -)\n"
        "  --status-s N     status line every N seconds, 0 off (default 10)\n",
        stderr);
}

bool parse(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        const bool        has  = i + 1 < argc;
        if (flag == "--uart" && has) {
            o.uart = argv[++i];
        } else if (flag == "--baud" && has) {
            o.baud = std::atoi(argv[++i]);
            if (!relay::supportedBaud(o.baud)) {
                logLine("unsupported baud %s", argv[i]);
                return false;
            }
        } else if (flag == "--brain" && has) {
            o.brain = argv[++i];
        } else if (flag == "--echo-brain") {
            o.echo_brain = true;
        } else if (flag == "--status-s" && has) {
            o.status_s = std::atoi(argv[++i]);
        } else {
            return false;
        }
    }
    return true;
}

// One reopening serial device.
struct Port {
    explicit Port(const char* name) : label(name) {}

    const char*       label;
    int               fd = -1;
    std::string       path;
    Clock::time_point retry_at{};
    std::string       last_problem; // logged once until it changes

    void problem(const std::string& text) {
        if (text != last_problem) {
            logLine("%s: %s", label, text.c_str());
            last_problem = text;
        }
    }

    void close(const char* reason) {
        if (fd >= 0) {
            ::close(fd);
            logLine("%s %s closed: %s", label, path.c_str(), reason);
        }
        fd           = -1;
        last_problem = "";
        retry_at     = Clock::now() + kRetry;
    }
};

bool openPort(Port& port, const std::string& path, int baud) {
    port.retry_at = Clock::now() + kRetry;
    const int fd  = relay::openSerial(path, baud);
    if (fd < 0) {
        port.problem("cannot open " + path + ": " + std::strerror(errno));
        return false;
    }
    port.fd           = fd;
    port.path         = path;
    port.last_problem = "";
    logLine("%s %s open", port.label, path.c_str());
    return true;
}

std::string brainPath(const Options& o, Port& brain) {
    if (o.brain != "auto") {
        return o.brain;
    }
    const relay::BrainPortSearch found = relay::findBrainUserPort();
    if (found.brains == 0) {
        brain.problem("no V5 Brain user port (USB 2888:0501 interface 02), retrying");
    } else if (found.brains > 1) {
        brain.problem(std::to_string(found.brains) + " Brains found, pass --brain PATH");
    }
    brain.retry_at = Clock::now() + kRetry;
    return found.path;
}

// Reads what is there. False when the device went away.
bool readSome(Port& port, short revents, uint8_t* buffer, ssize_t& count) {
    count = 0;
    if (revents & POLLNVAL) {
        port.close("invalid");
        return false;
    }
    if (!(revents & (POLLIN | POLLERR | POLLHUP))) {
        return true;
    }
    count = ::read(port.fd, buffer, kReadChunk);
    if (count > 0) {
        return true;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        count = 0;
        return true;
    }
    // Readable but no bytes: the USB device is gone.
    port.close(count == 0 ? "hang up" : std::strerror(errno));
    count = 0;
    return false;
}

// Copies Brain output to stdout without blocking: a chunk is dropped when the
// reader is behind. False once the reader is gone.
bool echo(const uint8_t* data, std::size_t size, uint64_t& dropped) {
    pollfd out{STDOUT_FILENO, POLLOUT, 0};
    if (::poll(&out, 1, 0) < 0) {
        return true;
    }
    if (out.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        return false;
    }
    if (!(out.revents & POLLOUT)) {
        dropped += size;
        return true;
    }
    // At most kReadChunk bytes, under PIPE_BUF: a pipe with room takes it whole.
    const ssize_t written = ::write(STDOUT_FILENO, data, size);
    if (written < 0 && errno == EPIPE) {
        return false;
    }
    if (written != static_cast<ssize_t>(size)) {
        dropped += size;
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parse(argc, argv, options)) {
        usage();
        return 2;
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGPIPE, SIG_IGN);

    logLine("uart %s at %d baud, brain %s", options.uart.c_str(), options.baud,
            options.brain.c_str());

    relay::Forwarder  forwarder;
    Port              uart("uart");
    Port              brain("brain");
    uint16_t          last_run      = 0;
    bool              seen_run      = false;
    uint64_t          brain_bytes   = 0;
    uint64_t          echo_dropped  = 0;
    uint64_t          stalls        = 0;
    Clock::time_point last_progress = Clock::now();
    Clock::time_point next_status   = Clock::now() + seconds(options.status_s);
    uint8_t           buffer[kReadChunk];

    while (!g_stop) {
        const Clock::time_point now = Clock::now();

        if (uart.fd < 0 && now >= uart.retry_at && openPort(uart, options.uart, options.baud)) {
            forwarder.resetInput();
        }
        if (brain.fd < 0 && now >= brain.retry_at) {
            const std::string path = brainPath(options, brain);
            if (!path.empty() && openPort(brain, path, 115200)) {
                forwarder.dropPending(); // nothing older than this open goes out
                last_progress = now;
            }
        }

        pollfd fds[2]  = {{uart.fd, POLLIN, 0}, {brain.fd, POLLIN, 0}};
        const bool out = brain.fd >= 0 && forwarder.pending().size > 0;
        if (out) {
            fds[1].events |= POLLOUT;
        }
        if (::poll(fds, 2, static_cast<int>(kPoll.count())) < 0 && errno != EINTR) {
            logLine("poll: %s", std::strerror(errno));
            return 1;
        }

        ssize_t count = 0;
        if (uart.fd >= 0 && readSome(uart, fds[0].revents, buffer, count) && count > 0) {
            if (forwarder.receive(buffer, static_cast<std::size_t>(count)) > 0) {
                const uint16_t run = forwarder.lastRecord().run_id;
                if (!seen_run || run != last_run) {
                    logLine("pico run %04X%s", run, seen_run ? " (Pico restarted)" : "");
                    last_run = run;
                    seen_run = true;
                }
            }
        }

        // Brain console output is read so it never backs up; kept only with --echo-brain.
        if (brain.fd >= 0 && readSome(brain, fds[1].revents, buffer, count) && count > 0) {
            brain_bytes += static_cast<uint64_t>(count);
            if (options.echo_brain &&
                !echo(buffer, static_cast<std::size_t>(count), echo_dropped)) {
                logLine("stdout closed, stopping");
                break;
            }
        }

        if (brain.fd >= 0) {
            const relay::Pending p = forwarder.pending();
            if (p.size == 0) {
                last_progress = Clock::now();
            } else {
                const ssize_t written = ::write(brain.fd, p.data, p.size);
                if (written > 0) {
                    forwarder.wrote(static_cast<std::size_t>(written));
                    last_progress = Clock::now();
                } else if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
                           errno != EINTR) {
                    brain.close(std::strerror(errno));
                } else if (Clock::now() - last_progress > kBrainStall) {
                    // Brain not taking input: drop what the kernel queued so it
                    // never receives a stale backlog later.
                    ::tcflush(brain.fd, TCOFLUSH);
                    forwarder.dropPending();
                    last_progress = Clock::now();
                    if (stalls++ == 0) {
                        logLine("brain not taking input, dropping queued lines");
                    }
                }
            }
        }

        if (options.status_s > 0 && Clock::now() >= next_status) {
            next_status                      = Clock::now() + seconds(options.status_s);
            const relay::ForwarderStats& s  = forwarder.stats();
            logLine("status uart %s brain %s | records %llu lines %llu superseded %llu "
                    "discarded %llu uart_skipped_bytes %lu stalls %llu brain_bytes %llu",
                    uart.fd >= 0 ? "open" : "closed", brain.fd >= 0 ? "open" : "closed",
                    static_cast<unsigned long long>(s.records),
                    static_cast<unsigned long long>(s.lines),
                    static_cast<unsigned long long>(s.superseded),
                    static_cast<unsigned long long>(s.discarded),
                    static_cast<unsigned long>(forwarder.scanStats().skipped),
                    static_cast<unsigned long long>(stalls),
                    static_cast<unsigned long long>(brain_bytes));
            if (echo_dropped > 0) {
                logLine("echo dropped %llu bytes", static_cast<unsigned long long>(echo_dropped));
            }
        }
    }

    uart.close("stopping");
    brain.close("stopping");
    return 0;
}
