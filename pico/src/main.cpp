#include <Arduino.h>

#include "config.h"
#include "frame_codec.h"
#include "frames.h"
#include "imu.h"
#include "quadrature.h"

using namespace gatr2;

namespace
{

uint8_t  g_seq       = 0;
uint32_t g_next_tick = 0;

// Reads every sensor that reported and sets only those bits.
SensorSample sample() {
    SensorSample s{};
    s.seq      = g_seq++;
    s.stamp_ms = millis();

    for (uint8_t ch = 0; ch < encoder::kChannels; ++ch) {
        if (encoder::started(ch)) {
            s.enc[ch] = encoder::count(ch);
            s.mask |= static_cast<uint16_t>(kSensorEnc0 << ch);
        }
    }

    int32_t gyro = 0;
    if (imu::readGyroZ(gyro)) {
        s.gyro_z = gyro;
        s.mask |= kSensorGyroZ;
    }

    return s;
}

} // namespace

void setup() {
    Serial1.setTX(cfg::kPiTxPin);
    Serial1.setRX(cfg::kPiRxPin);
    Serial1.begin(cfg::kPiBaud);

    for (uint8_t ch = 0; ch < encoder::kChannels; ++ch) {
        encoder::begin(ch, cfg::kEncPinA[ch], cfg::kEncPinB[ch]);
    }
    imu::begin();

    g_next_tick = micros();
}

void loop() {
    // Signed compare so the micros rollover is handled.
    if (static_cast<int32_t>(micros() - g_next_tick) < 0) {
        return;
    }
    g_next_tick += cfg::kTickUs;

    uint8_t        buf[kMaxFrameLen];
    const uint16_t n = encodeSensorFrame(sample(), buf, sizeof(buf));
    if (n == 0) {
        return;
    }

    // Drop the frame rather than block if the link is backed up.
    if (Serial1.availableForWrite() >= static_cast<int>(n)) {
        Serial1.write(buf, n);
    }
}
