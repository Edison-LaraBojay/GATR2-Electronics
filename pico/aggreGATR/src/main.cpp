#include <Arduino.h>
#include <hardware/gpio.h>
#include <hardware/uart.h>
#include <stdio.h>

#include "board.h"
#include "config.h"
#include "diag_report.h"
#include "frame_codec.h"
#include "frames.h"
#include "imu.h"
#include "pi_link.h"
#include "pico_commands.h"
#include "quadrature.h"

using namespace translagatr;

namespace
{

// RP2040 UART TX FIFO depth.
constexpr uint16_t kTxFifoLen = 32;

// Largest sensor frame sent: v2 header, three encoders and the gyro.
constexpr uint16_t kMaxSentLen =
    kSensorV2HeaderLen + kSensorWidth[0] + kSensorWidth[1] + kSensorWidth[2] + kSensorWidth[3] + 1;
static_assert(kMaxSentLen <= kTxFifoLen, "a sensor frame must fit the empty TX FIFO");

constexpr uint16_t kStatusFrameLen = kPicoStatusLen + kLinkEnvelopeLen;
static_assert(kStatusFrameLen <= kTxFifoLen, "a status frame must fit the empty TX FIFO");

constexpr uint16_t kDiagFrameLen = kPicoDiagLen + kLinkEnvelopeLen;
static_assert(kDiagFrameLen <= kTxFifoLen, "a diagnostic frame must fit the empty TX FIFO");

// Pins read back for the diagnostic frame. gpio_get reads the pad input
// only; no pin changes function, direction or pull.
constexpr pilink::DiagPin kDiagPins[] = {
    {kPicoPinImuInt, board::kImuIntPin},
#if defined(GATR2_IMU_BNO08X)
    {kPicoPinImuRst, board::kImuRstPin},
    {kPicoPinImuWake, board::kImuWakePin},
#endif
    {kPicoPinImuCs, board::kImuCsPin},
    {kPicoPinEnc0A, board::kEncPinA[0]},
    {kPicoPinEnc0B, board::kEncPinB[0]},
    {kPicoPinEnc1A, board::kEncPinA[1]},
    {kPicoPinEnc1B, board::kEncPinB[1]},
    {kPicoPinEnc2A, board::kEncPinA[2]},
    {kPicoPinEnc2B, board::kEncPinB[2]},
    {kPicoPinPiRx, board::kPiRxPin},
};

constexpr bool assignedPin(uint8_t gpio) {
    for (uint8_t p : cfg::kAllPins) {
        if (p == gpio) {
            return true;
        }
    }
    return false;
}

constexpr bool diagPinsAssigned() {
    for (const pilink::DiagPin& d : kDiagPins) {
        if (!assignedPin(d.gpio)) {
            return false;
        }
    }
    return true;
}
static_assert(diagPinsAssigned(), "a diagnostic pin is not in the pin budget");

uint8_t  g_seq       = 0;
uint32_t g_next_tick = 0;

// Acquisition identity for sensor v2 and status frames.
uint16_t g_boot_id   = 0;
uint8_t  g_acq_epoch = 0;

FrameReader            g_rx;
pilink::Commands       g_commands;
pilink::StatusSchedule g_status(cfg::kStatusPeriodUs, cfg::kStatusMinGapUs,
                                pilink::airtimeUs(kStatusFrameLen, cfg::kPiBaud) +
                                    cfg::kStatusMarginUs);
pilink::DiagSchedule   g_diag(pilink::airtimeUs(kDiagFrameLen, cfg::kPiBaud) + cfg::kStatusMarginUs);
imu::Status            g_imu_seen;
uint32_t               g_status_sent   = 0;
uint32_t               g_diag_sent     = 0;
uint8_t                g_diag_seq      = 0;
uint32_t               g_ticks_skipped = 0;

// Serial1.availableForWrite() only reports 0 or 1 on this core.
bool txFifoEmpty() { return (uart_get_hw(uart0)->fr & UART_UARTFR_TXFE_BITS) != 0; }

void apply(const pilink::Outcome& out) {
    switch (out.effect) {
    case pilink::Effect::None:
        break;
    case pilink::Effect::EnableImu:
        imu::setEnabled(true);
        break;
    case pilink::Effect::DisableImu:
        imu::setEnabled(false);
        break;
    case pilink::Effect::ReinitImu:
        imu::reinit();
        break;
    case pilink::Effect::RestartAcquisition:
        // Before the next sample, so every frame under the new epoch counts from zero.
        encoder::zeroAll();
        ++g_acq_epoch;
        break;
    case pilink::Effect::SetDiagnostics:
        g_diag.setRate(out.diag_hz);
        break;
    }
}

// Parses the Pi command bytes that have arrived, without waiting for more.
void receiveCommands() {
    for (int i = 0; i < cfg::kRxBytesPerPass && Serial1.available() > 0; ++i) {
        const int b = Serial1.read();
        if (b < 0) {
            break;
        }
        if (!g_rx.push(static_cast<uint8_t>(b))) {
            continue;
        }
        do {
            if (g_rx.frameType() != kFramePicoCommand) {
                continue;
            }
            const pilink::Outcome out =
                g_commands.receive(g_rx.frame(), g_rx.frameLen(), g_boot_id, imu::status().enabled);
            apply(out);
            if (out.answered) {
                g_status.request();
            }
        } while (g_rx.next());
    }
}

// Settles running IMU commands and asks for a status frame on any change.
void watchImu() {
    const imu::Status s = imu::status();
    if (g_commands.imuProgress(s.state)) {
        g_status.request();
    }
    if (s.enabled != g_imu_seen.enabled || s.state != g_imu_seen.state ||
        s.reason != g_imu_seen.reason || s.attempts != g_imu_seen.attempts ||
        s.epoch != g_imu_seen.epoch) {
        g_imu_seen = s;
        g_status.request();
    }
}

// Reads every sensor that reported and sets only those bits. One time
// snapshot stamps the encoder latch and cuts the gyro interval.
SensorSample sample() {
    const uint64_t now_us = time_us_64();

    SensorSample s{};
    s.seq       = g_seq;
    s.stamp_ms  = static_cast<uint32_t>(now_us / 1000);
    s.identity  = true;
    s.boot_id   = g_boot_id;
    s.acq_epoch = g_acq_epoch;
    s.imu_epoch = imu::status().epoch;

    for (uint8_t ch = 0; ch < encoder::kChannels; ++ch) {
        if (encoder::started(ch)) {
            s.enc[ch] = encoder::count(ch);
            s.mask |= static_cast<uint16_t>(kSensorEnc0 << ch);
        }
    }

    int32_t gyro = 0;
    if (imu::readGyroZ(static_cast<uint32_t>(now_us), gyro)) {
        s.gyro_z = gyro;
        s.mask |= kSensorGyroZ;
    }

    return s;
}

PicoStatus picoStatus() {
    const imu::Status i = imu::status();
    PicoStatus        st;
    st.boot_id      = g_boot_id;
    st.acq_epoch    = g_acq_epoch;
    st.imu_epoch    = i.epoch;
    st.uptime_ms    = static_cast<uint32_t>(time_us_64() / 1000);
    st.imu_state    = i.state;
    st.imu_reason   = i.reason;
    st.imu_attempts = i.attempts;
    st.flags        = i.enabled ? kPicoImuEnabled : 0;
    st.firmware     = imu::firmware();
    g_commands.fill(st);
    return st;
}

void sendStatus(uint32_t now) {
    uint8_t        buf[kStatusFrameLen];
    const uint16_t n = encodePicoStatus(picoStatus(), buf, sizeof(buf));
    if (n == 0) {
        return;
    }
    Serial1.write(buf, n);
    g_status.sent(now);
    ++g_status_sent;
}

// Only from an idle window (DiagSchedule): the frame fits the empty FIFO and
// its airtime ends before the next sensor tick.
void sendDiag(uint32_t now) {
    pilink::DiagInputs in;
    in.boot_id  = g_boot_id;
    in.seq      = g_diag_seq;
    in.firmware = imu::firmware();
    pilink::readPins(kDiagPins, sizeof(kDiagPins) / sizeof(kDiagPins[0]),
                     [](uint8_t gpio) { return gpio_get(gpio); }, in.pins, in.pins_known);
    in.imu_present         = true;
    in.imu                 = imu::counters();
    const FrameReaderStats& rx = g_rx.stats();
    in.link_rx_bad         = rx.length_errors + rx.check_errors;
    in.ticks_skipped       = g_ticks_skipped;

    uint8_t        buf[kDiagFrameLen];
    const uint16_t n = encodePicoDiag(pilink::makeDiag(in), buf, sizeof(buf));
    if (n == 0) {
        return;
    }
    Serial1.write(buf, n);
    g_diag.sent(now);
    ++g_diag_seq;
    ++g_diag_sent;
}

// Print the same sample sent to the Pi, without reading/consuming the IMU twice.
// USB is optional: never wait for a monitor or for space in its transmit FIFO.
void logUsb(const SensorSample& s) {
    static uint32_t last_ms = 0;
    if (!cfg::kUsbDebug || !Serial || s.stamp_ms - last_ms < cfg::kUsbDebugPeriodMs) {
        return;
    }
    last_ms = s.stamp_ms;

    static uint32_t last_status_ms = 0;
    if (s.stamp_ms - last_status_ms >= 1000) {
        char   status[320];
        size_t length = imu::formatDiagnostics(status, sizeof(status));
        if (length > 0 && Serial.availableForWrite() >= static_cast<int>(length)) {
            Serial.write(reinterpret_cast<const uint8_t*>(status), length);
            last_status_ms = s.stamp_ms;
        }
        const PicoStatus st = picoStatus();
        const int        n =
            snprintf(status, sizeof(status),
                     "link boot=%04X acq=%u imu_epoch=%u imu_state=%u reason=%u attempts=%u "
                     "status_sent=%lu cmd=%lu dup=%lu ignored=%lu last=%u/%u/%u/%u "
                     "diag_hz=%u diag_sent=%lu rx_bad=%lu ticks_skipped=%lu\r\n",
                     static_cast<unsigned>(st.boot_id), static_cast<unsigned>(st.acq_epoch),
                     static_cast<unsigned>(st.imu_epoch), static_cast<unsigned>(st.imu_state),
                     static_cast<unsigned>(st.imu_reason), static_cast<unsigned>(st.imu_attempts),
                     static_cast<unsigned long>(g_status_sent),
                     static_cast<unsigned long>(g_commands.received()),
                     static_cast<unsigned long>(g_commands.duplicates()),
                     static_cast<unsigned long>(g_commands.ignored()),
                     static_cast<unsigned>(st.last_request_id), static_cast<unsigned>(st.last_op),
                     static_cast<unsigned>(st.last_status), static_cast<unsigned>(st.last_detail),
                     static_cast<unsigned>(g_diag.rate()), static_cast<unsigned long>(g_diag_sent),
                     static_cast<unsigned long>(g_rx.stats().length_errors +
                                                g_rx.stats().check_errors),
                     static_cast<unsigned long>(g_ticks_skipped));
        length = n > 0 && static_cast<size_t>(n) < sizeof(status) ? static_cast<size_t>(n) : 0;
        if (length > 0 && Serial.availableForWrite() >= static_cast<int>(length)) {
            Serial.write(reinterpret_cast<const uint8_t*>(status), length);
        }
    }

    char yaw[24];
    if (s.mask & kSensorGyroZ) {
        snprintf(yaw, sizeof(yaw), "%ld", static_cast<long>(s.gyro_z));
    } else {
        snprintf(yaw, sizeof(yaw), "NO_SAMPLE");
    }
    char line[160];
    const int n = snprintf(line, sizeof(line),
                           "t_ms=%lu enc0=%ld enc1=%ld enc2=%ld imu_yaw_mdps=%s\r\n",
                           static_cast<unsigned long>(s.stamp_ms),
                           static_cast<long>(s.enc[0]), static_cast<long>(s.enc[1]),
                           static_cast<long>(s.enc[2]), yaw);
    if (n > 0 && static_cast<size_t>(n) < sizeof(line) && Serial.availableForWrite() >= n) {
        Serial.write(reinterpret_cast<const uint8_t*>(line), static_cast<size_t>(n));
    }
}

} // namespace

void setup() {
    if (cfg::kUsbDebug) {
        Serial.begin(cfg::kUsbBaud);
    }
    Serial1.setTX(board::kPiTxPin);
    Serial1.setRX(board::kPiRxPin);
    Serial1.setFIFOSize(cfg::kRxFifoLen);
    Serial1.begin(cfg::kPiBaud);

    // pico_rand mixes ROSC random bits with the timer.
    g_boot_id = pilink::makeBootId(rp2040.hwrand32(), time_us_64());

    for (uint8_t ch = 0; ch < encoder::kChannels; ++ch) {
        encoder::begin(ch, board::kEncPinA[ch], board::kEncPinB[ch]);
    }
    imu::begin();

    g_next_tick = micros();
}

void loop() {
    imu::service();
    receiveCommands();
    watchImu();

    // Signed compare so the micros rollover is handled.
    const uint32_t now = micros();
    if (static_cast<int32_t>(now - g_next_tick) >= 0) {
        g_next_tick += cfg::kTickUs;

        // Skip the tick rather than block. Nothing is sampled, so the gyro
        // interval carries into the next frame.
        if (!txFifoEmpty()) {
            ++g_ticks_skipped;
            return;
        }

        const SensorSample s = sample();
        uint8_t            buf[kMaxFrameLen];
        const uint16_t     n = encodeSensorFrame(s, buf, sizeof(buf));
        if (n == 0) {
            return;
        }
        Serial1.write(buf, n);
        ++g_seq;
        logUsb(s);
        return;
    }

    // Status, then diagnostic frames, use the idle time between sensor
    // frames only; each needs an empty FIFO, so at most one per pass.
    const bool tx_empty = txFifoEmpty();
    if (g_status.ready(now, g_next_tick, tx_empty)) {
        sendStatus(now);
    } else if (g_diag.ready(now, g_next_tick, tx_empty)) {
        sendDiag(now);
    }
}
