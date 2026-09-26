// imu_bno08x.cpp
// BNO08X over SPI1 through CEVA sh2 v1.4.0, non-blocking. Reports 0x01/0x07:
// startup gravity alignment from acceleration, then XYZ gyro projected onto
// the learned up axis. Gyro bias remains for the Pi to calibrate.
// Datasheet 1000-3927, SH-2 manual 1000-3625, SHTP 1000-3535.

#if defined(GATR2_IMU_BNO08X)

#include "imu.h"

#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>

#include "bno08x_supervisor.h"
#include "bno08x_reports.h"
#include "config.h"
#include "shtp_header.h"

extern "C" {
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_hal.h"
}

namespace imu
{
namespace
{

using bno08x::kShtpHeaderLen;

// Mode 3, 3 MHz max.
const SPISettings kSpi(3000000, MSBFIRST, SPI_MODE3);

constexpr uint32_t kResetHoldUs         = 10000;
constexpr uint32_t kWakeUs              = 2000; // twk is 150 us max
constexpr uint32_t kEdgeMaxAgeUs        = 20000;
constexpr uint32_t kReportUs            = 5000;
constexpr uint32_t kAccelReportUs       = 10000;
constexpr int      kMaxFramesPerService = 4;
constexpr uint16_t kBufLen              = bno08x::kShtpMaxRxLen;
static_assert(kBufLen <= SH2_HAL_MAX_TRANSFER_IN, "read buffer larger than sh2's");

// Held: RST low. Booting: released, no INT yet, WAKE must stay high. Up: WAKE usable.
enum class Link : uint8_t { Held, Booting, Up };

volatile uint32_t g_edge_us = 0;
volatile bool     g_edge    = false;

Link     g_link       = Link::Held;
bool     g_open       = false;
uint32_t g_release_us = 0;

const uint8_t kZeros[kBufLen] = {};
uint8_t       g_tx[kBufLen];
uint8_t       g_pending[kBufLen];
uint16_t      g_pending_len = 0;
uint32_t      g_pending_us  = 0;

sh2_Hal_t          g_hal;
bno08x::Supervisor g_sup;
bno08x::Events     g_events;
bno08x::Bno08xReports g_reports;

// Lifetime counters survive recovery, so a disconnected monitor can reconnect
// and still see why the hub has been restarting. No additional SPI reads.
uint32_t g_boots = 0, g_retries = 0, g_rx = 0, g_bad_headers = 0;
uint32_t g_accel_reports = 0, g_gyro_reports = 0, g_rejected_reports = 0;
int32_t g_last_report_age_us = 0;
uint8_t g_last_header[4] = {};
int g_last_error = SH2_OK;
const char* g_last_failure = "none";

static_assert(SH2_ACCELEROMETER == bno08x::kAccelReportId, "SH-2 acceleration report ID");
static_assert(SH2_GYROSCOPE_UNCALIBRATED == bno08x::kGyroReportId, "SH-2 gyro report ID");

void onInt() {
    g_edge_us = time_us_32();
    g_edge    = true;
}

bool intLow() { return digitalRead(board::kImuIntPin) == LOW; }

// INT assertion time for the transfer about to start, now when no recent edge.
uint32_t takeEdge() {
    noInterrupts();
    const uint32_t t = g_edge_us;
    const bool     e = g_edge;
    g_edge           = false;
    interrupts();
    const uint32_t now = time_us_32();
    return (e && now - t < kEdgeMaxAgeUs) ? t : now;
}

void holdReset() {
    digitalWrite(board::kImuCsPin, HIGH);
    digitalWrite(board::kImuWakePin, HIGH); // PS0 high through reset selects SPI
    digitalWrite(board::kImuRstPin, LOW);
    g_link        = Link::Held;
    g_pending_len = 0;
}

void releaseReset() {
    digitalWrite(board::kImuRstPin, HIGH);
    g_link       = Link::Booting;
    g_release_us = time_us_32();
}

// One CS frame: header, then clocks until both cargoes are done. Bytes past a
// cargo are padding. tx holds kBufLen bytes. Returns the inbound length.
uint16_t transfer(const uint8_t* tx, uint16_t tx_len, uint8_t* rx, bool release_wake) {
    SPI1.beginTransaction(kSpi);
    digitalWrite(board::kImuCsPin, LOW);
    if (release_wake) {
        digitalWrite(board::kImuWakePin, HIGH);
    }
    SPI1.transfer(tx, rx, kShtpHeaderLen);
    const uint16_t rx_len = bno08x::shtpRxLen(rx);
    memcpy(g_last_header, rx, sizeof(g_last_header));
    if (rx_len != 0) {
        ++g_rx;
    } else if (rx[0] != 0 || rx[1] != 0) {
        ++g_bad_headers;
    }
    const uint16_t total  = tx_len > rx_len ? tx_len : rx_len;
    if (total > kShtpHeaderLen) {
        SPI1.transfer(tx + kShtpHeaderLen, rx + kShtpHeaderLen, total - kShtpHeaderLen);
    }
    digitalWrite(board::kImuCsPin, HIGH);
    SPI1.endTransaction();
    return rx_len;
}

int halOpen(sh2_Hal_t*) {
    holdReset();
    delayMicroseconds(kResetHoldUs);
    releaseReset();
    return SH2_OK;
}

void halClose(sh2_Hal_t*) { holdReset(); }

// 0 unless INT is low or a frame captured during a write is pending.
int halRead(sh2_Hal_t*, uint8_t* buf, unsigned len, uint32_t* t_us) {
    if (g_pending_len != 0) {
        const uint16_t n = g_pending_len;
        g_pending_len    = 0;
        if (n > len) {
            return 0;
        }
        memcpy(buf, g_pending, n);
        *t_us = g_pending_us;
        return n;
    }
    if (g_link == Link::Held || len < kBufLen || !intLow()) {
        return 0;
    }
    const uint32_t t = takeEdge();
    const uint16_t n = transfer(kZeros, 0, buf, false);
    g_link           = Link::Up;
    if (n == 0) {
        return 0;
    }
    *t_us = t;
    return n;
}

// Never 0 except while a captured frame is pending, which the next read drains.
// sh2 retries a 0 return forever.
int halWrite(sh2_Hal_t*, uint8_t* buf, unsigned len) {
    if (len < kShtpHeaderLen || len > kBufLen) {
        return SH2_ERR_BAD_PARAM;
    }
    if (g_link != Link::Up) {
        return SH2_ERR_IO;
    }
    if (g_pending_len != 0) {
        return 0;
    }
    // WAKE handshake, the only busy wait.
    digitalWrite(board::kImuWakePin, LOW);
    const uint32_t t0 = time_us_32();
    while (!intLow()) {
        if (time_us_32() - t0 > kWakeUs) {
            digitalWrite(board::kImuWakePin, HIGH);
            g_last_error = SH2_ERR_TIMEOUT;
            return SH2_ERR_TIMEOUT;
        }
    }
    const uint32_t t = takeEdge();
    memcpy(g_tx, buf, len);
    memset(g_tx + len, 0, kBufLen - len);
    const uint16_t n = transfer(g_tx, static_cast<uint16_t>(len), g_pending, true);
    if (n != 0) {
        g_pending_len = n;
        g_pending_us  = t;
    }
    return static_cast<int>(len);
}

uint32_t halTime(sh2_Hal_t*) { return time_us_32(); }

void onEvent(void*, sh2_AsyncEvent_t* e) {
    if (e->eventId == SH2_RESET) {
        ++g_boots;
        g_events.reset = true;
        g_reports.reset();
    } else if (e->eventId == SH2_GET_FEATURE_RESP) {
        g_reports.acknowledge(e->sh2SensorConfigResp.sensorId,
                             e->sh2SensorConfigResp.sensorConfig.reportInterval_us);
        g_events.ack = g_reports.acknowledged();
    }
}

// Sample time is the INT edge minus the hub's reported delay, low 32 bits.
void onSensor(void*, sh2_SensorEvent_t* e) {
    const uint32_t now = time_us_32();
    g_last_report_age_us = static_cast<int32_t>(now - static_cast<uint32_t>(e->timestamp_uS));
    const bool accepted = g_reports.add(e->reportId, e->report, e->len,
                                        static_cast<uint32_t>(e->timestamp_uS), now);
    if (!accepted) {
        ++g_rejected_reports;
    } else if (e->reportId == SH2_ACCELEROMETER) {
        ++g_accel_reports;
    } else if (e->reportId == SH2_GYROSCOPE_UNCALIBRATED) {
        ++g_gyro_reports;
    }
    if (accepted && e->reportId == SH2_GYROSCOPE_UNCALIBRATED && g_reports.healthy(now)) {
        // Raw report health drives recovery even while calibration suppresses
        // the outgoing yaw. Otherwise the watchdog would reset during alignment.
        g_events.report = true;
    }
}

void enableReport() {
    sh2_SensorConfig_t c{};
    c.reportInterval_us = kReportUs;
    g_last_error = sh2_setSensorConfig(SH2_GYROSCOPE_UNCALIBRATED, &c);
    if (g_last_error != SH2_OK) {
        g_events.enable_failed = true;
        return;
    }
    c.reportInterval_us = kAccelReportUs;
    g_last_error = sh2_setSensorConfig(SH2_ACCELEROMETER, &c);
    if (g_last_error != SH2_OK) {
        g_events.enable_failed = true;
    }
}

} // namespace

void begin() {
    pinMode(board::kImuRstPin, OUTPUT);
    pinMode(board::kImuWakePin, OUTPUT);
    pinMode(board::kImuCsPin, OUTPUT);
    holdReset();
    pinMode(board::kImuIntPin, INPUT_PULLUP);

    SPI1.setSCK(board::kImuSckPin);
    SPI1.setTX(board::kImuMosiPin);
    SPI1.setRX(board::kImuMisoPin);
    SPI1.begin();
    SPI1.beginTransaction(kSpi); // SCK idles high before the first CS
    SPI1.endTransaction();

    attachInterrupt(digitalPinToInterrupt(board::kImuIntPin), onInt, FALLING);

    g_hal.open      = halOpen;
    g_hal.close     = halClose;
    g_hal.read      = halRead;
    g_hal.write     = halWrite;
    g_hal.getTimeUs = halTime;

    // Only call. Resets the hub through halOpen, then polls up to 200 ms for its boot.
    g_last_error = sh2_open(&g_hal, onEvent, nullptr);
    g_open = g_last_error == SH2_OK;
    if (!g_open) {
        g_last_failure = "open_error";
        holdReset();
        return;
    }
    sh2_setSensorCallback(onSensor, nullptr);
    g_sup.start(g_release_us);
}

void service() {
    if (!g_open) {
        return;
    }
    for (int i = 0;
         i < kMaxFramesPerService && g_link != Link::Held && (g_pending_len != 0 || intLow());
         ++i) {
        sh2_service();
    }

    const bno08x::Events ev = g_events;
    g_events                = {};
    const bno08x::State previous = g_sup.state();
    const bno08x::Action a  = g_sup.step(time_us_32(), ev);
    if (a == bno08x::Action::HoldReset) {
        ++g_retries;
        if (previous == bno08x::State::Boot) {
            g_last_failure = "boot_timeout";
        } else if (previous == bno08x::State::WaitAck) {
            g_last_failure = ev.enable_failed ? "enable_error" : "feature_timeout";
        } else {
            g_last_failure = "stream_timeout";
        }
    }
    if (bno08x::dropsSamples(a)) {
        g_reports.reset();
    }
    switch (a) {
    case bno08x::Action::None:
        break;
    case bno08x::Action::HoldReset:
        holdReset();
        break;
    case bno08x::Action::ReleaseReset:
        releaseReset();
        break;
    case bno08x::Action::Enable:
        enableReport();
        break;
    }
}

bool readGyroZ(uint32_t cut_us, int32_t& gyro_z_mdps) { return g_reports.take(cut_us, gyro_z_mdps); }

size_t formatDiagnostics(char* output, size_t capacity) {
    const char* state = "OPEN_ERROR";
    if (g_open) {
        switch (g_sup.state()) {
        case bno08x::State::Reset: state = "RESET_WAIT"; break;
        case bno08x::State::Boot: state = "WAIT_BOOT"; break;
        case bno08x::State::WaitAck: state = "WAIT_FEATURES"; break;
        case bno08x::State::Run:
            state = !g_reports.acknowledged() ? "WAIT_FEATURES" :
                    !g_reports.healthy(time_us_32()) ? "STREAM_STALE" :
                    !g_reports.aligned() ? "ALIGNING" : "READY";
            break;
        }
    }
    const int n = snprintf(output, capacity,
        "imu=BNO08X state=%s last=%s int=%u rst=%u wake=%u boot=%lu retry=%lu "
        "rx=%lu bad=%lu hdr=%02X%02X%02X%02X ack=%u a=%lu g=%lu reject=%lu cal=%lu age_us=%ld err=%d\r\n",
        state, g_last_failure, static_cast<unsigned>(digitalRead(board::kImuIntPin)),
        static_cast<unsigned>(digitalRead(board::kImuRstPin)),
        static_cast<unsigned>(digitalRead(board::kImuWakePin)),
        static_cast<unsigned long>(g_boots), static_cast<unsigned long>(g_retries),
        static_cast<unsigned long>(g_rx), static_cast<unsigned long>(g_bad_headers),
        static_cast<unsigned>(g_last_header[0]), static_cast<unsigned>(g_last_header[1]),
        static_cast<unsigned>(g_last_header[2]), static_cast<unsigned>(g_last_header[3]),
        static_cast<unsigned>(g_reports.ackMask()), static_cast<unsigned long>(g_accel_reports),
        static_cast<unsigned long>(g_gyro_reports), static_cast<unsigned long>(g_rejected_reports),
        static_cast<unsigned long>(g_reports.alignmentSamples()),
        static_cast<long>(g_last_report_age_us), g_last_error);
    return n > 0 && static_cast<size_t>(n) < capacity ? static_cast<size_t>(n) : 0;
}

} // namespace imu

#endif
