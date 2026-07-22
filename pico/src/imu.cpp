// imu.cpp
// ASM330LHHG1 over SPI. Only the gyro Z axis is read.
// Datasheet: https://www.st.com/resource/en/datasheet/asm330lhhg1.pdf

#include "imu.h"

#include <Arduino.h>
#include <SPI.h>

#include "config.h"

namespace imu
{
namespace
{

constexpr uint8_t kRegWhoAmI  = 0x0F;
constexpr uint8_t kRegCtrl1Xl = 0x10;
constexpr uint8_t kRegCtrl2G  = 0x11;
constexpr uint8_t kRegCtrl3C  = 0x12;
constexpr uint8_t kRegStatus  = 0x1E;
constexpr uint8_t kRegOutZLG  = 0x26;

// Datasheet DS15066 section 9.11, fixed WHO_AM_I value.
constexpr uint8_t kWhoAmIValue = 0x6B;

// Set on the address byte to make a transfer a read.
constexpr uint8_t kReadBit = 0x80;

// BDU so a sample cannot tear across an update, IF_INC for burst reads.
constexpr uint8_t kCtrl3CInit = 0x44;

// SW_RESET, CTRL3_C bit 0. Self-clears when the reset completes.
constexpr uint8_t kCtrl3CReset = 0x01;
constexpr uint8_t kSwReset     = 0x01;

// ODR 208 Hz in bits 7:4, full scale 2000 dps in bits 3:2.
constexpr uint8_t kCtrl2GInit = 0x5C;

// Whole mdeg/s per LSB at 2000 dps, so the conversion stays integer.
constexpr int32_t kMdpsPerLsb = 70;

constexpr uint8_t kStatusGyroReady = 0x02;

SPISettings g_spi(10000000, MSBFIRST, SPI_MODE3);
bool        g_ready = false;

void select() { digitalWrite(cfg::kImuCsPin, LOW); }
void deselect() { digitalWrite(cfg::kImuCsPin, HIGH); }

uint8_t readReg(uint8_t reg) {
    SPI.beginTransaction(g_spi);
    select();
    SPI.transfer(static_cast<uint8_t>(kReadBit | reg));
    const uint8_t v = SPI.transfer(0x00);
    deselect();
    SPI.endTransaction();
    return v;
}

void writeReg(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(g_spi);
    select();
    SPI.transfer(reg);
    SPI.transfer(val);
    deselect();
    SPI.endTransaction();
}

int16_t readGyroZRaw() {
    SPI.beginTransaction(g_spi);
    select();
    SPI.transfer(static_cast<uint8_t>(kReadBit | kRegOutZLG));
    const uint8_t lo = SPI.transfer(0x00);
    const uint8_t hi = SPI.transfer(0x00);
    deselect();
    SPI.endTransaction();
    return static_cast<int16_t>(static_cast<uint16_t>(lo | (hi << 8)));
}

} // namespace

void begin() {
    pinMode(cfg::kImuCsPin, OUTPUT);
    deselect();

    SPI.setSCK(cfg::kImuSckPin);
    SPI.setTX(cfg::kImuMosiPin);
    SPI.setRX(cfg::kImuMisoPin);
    SPI.begin();

    if (readReg(kRegWhoAmI) != kWhoAmIValue) {
        g_ready = false;
        return;
    }

    // Reset to a known state. The Pico can reboot while the IMU keeps running
    // with its previous config, so never assume power-on defaults.
    writeReg(kRegCtrl3C, kCtrl3CReset);
    for (int i = 0; i < 100 && (readReg(kRegCtrl3C) & kSwReset); ++i) {
        delay(1);
    }

    writeReg(kRegCtrl3C, kCtrl3CInit);
    writeReg(kRegCtrl2G, kCtrl2GInit);
    writeReg(kRegCtrl1Xl, 0x00); // accelerometer stays off, it is unused
    g_ready = true;
}

bool readGyroZ(int32_t& gyro_z_mdps) {
    if (!g_ready) {
        return false;
    }
    if ((readReg(kRegStatus) & kStatusGyroReady) == 0) {
        return false;
    }
    gyro_z_mdps = static_cast<int32_t>(readGyroZRaw()) * kMdpsPerLsb;
    return true;
}

} // namespace imu
