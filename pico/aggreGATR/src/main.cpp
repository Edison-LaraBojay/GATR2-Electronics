#include <Arduino.h>
#include <stdio.h>

#include "board.h"
#include "config.h"
#include "imu.h"
#include "quadrature.h"

namespace
{

uint32_t g_next_tick = 0;

// Local acquisition snapshot, independent of any transport or wire format.
struct SensorSample {
    uint32_t stamp_ms = 0;
    int32_t  enc[encoder::kChannels] = {};
    int32_t  gyro_z_mdps = 0;
    bool     gyro_valid = false;
};

// One time snapshot stamps the encoder reads and cuts the gyro interval.
SensorSample sample() {
    const uint64_t now_us = time_us_64();

    SensorSample s;
    s.stamp_ms = static_cast<uint32_t>(now_us / 1000);
    for (uint8_t ch = 0; ch < encoder::kChannels; ++ch) {
        if (encoder::started(ch)) {
            s.enc[ch] = encoder::count(ch);
        }
    }
    s.gyro_valid = imu::readGyroZ(static_cast<uint32_t>(now_us), s.gyro_z_mdps);
    return s;
}

// Local USB instrumentation only. Never wait for a monitor or for space in
// its transmit buffer; logging does not consume another IMU measurement.
void logUsb(const SensorSample& s) {
    static uint32_t last_ms = 0;
    if (!cfg::kUsbDebug || !Serial || s.stamp_ms - last_ms < cfg::kUsbDebugPeriodMs) {
        return;
    }
    last_ms = s.stamp_ms;

    static uint32_t last_status_ms = 0;
    if (s.stamp_ms - last_status_ms >= 1000) {
        char         status[320];
        const size_t length = imu::formatDiagnostics(status, sizeof(status));
        if (length > 0 && Serial.availableForWrite() >= static_cast<int>(length)) {
            Serial.write(reinterpret_cast<const uint8_t*>(status), length);
            last_status_ms = s.stamp_ms;
        }
    }

    char yaw[24];
    if (s.gyro_valid) {
        snprintf(yaw, sizeof(yaw), "%ld", static_cast<long>(s.gyro_z_mdps));
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
    for (uint8_t ch = 0; ch < encoder::kChannels; ++ch) {
        encoder::begin(ch, board::kEncPinA[ch], board::kEncPinB[ch]);
    }
    imu::begin();
    g_next_tick = micros();
}

void loop() {
    imu::service();

    // Signed compare handles micros rollover. Acquisition continues with
    // USB disconnected or logging disabled.
    const uint32_t now = micros();
    if (static_cast<int32_t>(now - g_next_tick) >= 0) {
        g_next_tick += cfg::kTickUs;
        const SensorSample s = sample();
        logUsb(s);
    }
}
