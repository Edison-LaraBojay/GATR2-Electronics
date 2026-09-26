#include <Arduino.h>
#include <hardware/uart.h>
#include <stdio.h>

#include "board.h"
#include "config.h"
#include "frame_codec.h"
#include "frames.h"
#include "imu.h"
#include "quadrature.h"

using namespace gatr2;

namespace
{

// RP2040 UART TX FIFO depth.
constexpr uint16_t kTxFifoLen = 32;

// Largest frame sent: three encoders and the gyro.
constexpr uint16_t kMaxSentLen =
    kSensorHeaderLen + kSensorWidth[0] + kSensorWidth[1] + kSensorWidth[2] + kSensorWidth[3] + 1;
static_assert(kMaxSentLen <= kTxFifoLen, "a frame must fit the empty TX FIFO");

uint8_t  g_seq       = 0;
uint32_t g_next_tick = 0;

// Serial1.availableForWrite() only reports 0 or 1 on this core.
bool txFifoEmpty() { return (uart_get_hw(uart0)->fr & UART_UARTFR_TXFE_BITS) != 0; }

// Reads every sensor that reported and sets only those bits. One time
// snapshot stamps the encoder latch and cuts the gyro interval.
SensorSample sample() {
    const uint64_t now_us = time_us_64();

    SensorSample s{};
    s.seq      = g_seq;
    s.stamp_ms = static_cast<uint32_t>(now_us / 1000);

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
        char status[256];
        const size_t length = imu::formatDiagnostics(status, sizeof(status));
        if (length > 0 && Serial.availableForWrite() >= static_cast<int>(length)) {
            Serial.write(reinterpret_cast<const uint8_t*>(status), length);
            last_status_ms = s.stamp_ms;
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
    Serial1.begin(cfg::kPiBaud);

    for (uint8_t ch = 0; ch < encoder::kChannels; ++ch) {
        encoder::begin(ch, board::kEncPinA[ch], board::kEncPinB[ch]);
    }
    imu::begin();

    g_next_tick = micros();
}

void loop() {
    imu::service();

    // Signed compare so the micros rollover is handled.
    if (static_cast<int32_t>(micros() - g_next_tick) < 0) {
        return;
    }
    g_next_tick += cfg::kTickUs;

    // Skip the tick rather than block. Nothing is sampled, so the gyro
    // interval carries into the next frame.
    if (!txFifoEmpty()) {
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
}
