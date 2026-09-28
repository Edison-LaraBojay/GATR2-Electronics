// imu_asm330.cpp
// ASM330LHHG1 over SPI1. Only the gyro Z axis is read. Bring-up, health
// checks and retries run through asm330::Supervisor without blocking.
// Datasheet: https://www.st.com/resource/en/datasheet/asm330lhhg1.pdf

#if defined(GATR2_IMU_ASM330)

#include "imu.h"

#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>

#include "asm330_supervisor.h"
#include "config.h"
#include "frames.h"

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

// ODR 208 Hz in bits 7:4, full scale 2000 dps in bits 3:2. 0 is power down.
constexpr uint8_t kCtrl2GInit = 0x5C;
constexpr uint8_t kCtrl2GOff  = 0x00;

// Whole mdeg/s per LSB at 2000 dps, so the conversion stays integer.
constexpr int32_t kMdpsPerLsb = 70;

constexpr uint8_t kStatusGyroReady = 0x02;

// GP8/9/10/11 are the SPI1 pin group, so the bus is SPI1, not SPI.
SPISettings        g_spi(10000000, MSBFIRST, SPI_MODE3);
asm330::Supervisor g_sup;

void select() { digitalWrite(board::kImuCsPin, LOW); }
void deselect() { digitalWrite(board::kImuCsPin, HIGH); }

uint8_t readReg(uint8_t reg) {
    SPI1.beginTransaction(g_spi);
    select();
    SPI1.transfer(static_cast<uint8_t>(kReadBit | reg));
    const uint8_t v = SPI1.transfer(0x00);
    deselect();
    SPI1.endTransaction();
    return v;
}

void writeReg(uint8_t reg, uint8_t val) {
    SPI1.beginTransaction(g_spi);
    select();
    SPI1.transfer(reg);
    SPI1.transfer(val);
    deselect();
    SPI1.endTransaction();
}

int16_t readGyroZRaw() {
    SPI1.beginTransaction(g_spi);
    select();
    SPI1.transfer(static_cast<uint8_t>(kReadBit | kRegOutZLG));
    const uint8_t lo = SPI1.transfer(0x00);
    const uint8_t hi = SPI1.transfer(0x00);
    deselect();
    SPI1.endTransaction();
    return static_cast<int16_t>(static_cast<uint16_t>(lo | (hi << 8)));
}

DeviceView view() {
    DeviceView d;
    d.enabled = g_sup.state() != asm330::State::Off;
    d.up      = g_sup.running();
    d.ready   = d.up;
    d.waiting = g_sup.state() == asm330::State::Wait;
    return d;
}

const char* stateName() {
    switch (g_sup.state()) {
    case asm330::State::Off:
        return "DISABLED";
    case asm330::State::Wait:
        return "WAIT_RETRY";
    case asm330::State::Probe:
        return "PROBE";
    case asm330::State::Reset:
        return "RESETTING";
    case asm330::State::Configure:
        return "CONFIGURE";
    case asm330::State::Run:
        return "READY";
    }
    return "?";
}

} // namespace

void begin() {
    pinMode(board::kImuCsPin, OUTPUT);
    deselect();

    SPI1.setSCK(board::kImuSckPin);
    SPI1.setTX(board::kImuMosiPin);
    SPI1.setRX(board::kImuMisoPin);
    SPI1.begin();

    g_sup.start(time_us_32());
    service();
}

void service() {
    const uint32_t now = time_us_32();
    switch (g_sup.step(now)) {
    case asm330::Action::None:
        break;
    case asm330::Action::Probe: {
        // Reset to a known state. The Pico can reboot while the IMU keeps
        // running with its previous config, so never assume power-on defaults.
        const bool found = readReg(kRegWhoAmI) == kWhoAmIValue;
        if (found) {
            writeReg(kRegCtrl3C, kCtrl3CReset);
        }
        g_sup.probed(now, found);
        break;
    }
    case asm330::Action::PollReset:
        g_sup.resetPolled(now, (readReg(kRegCtrl3C) & kSwReset) == 0);
        break;
    case asm330::Action::Configure:
        writeReg(kRegCtrl3C, kCtrl3CInit);
        writeReg(kRegCtrl2G, kCtrl2GInit);
        writeReg(kRegCtrl1Xl, 0x00); // accelerometer stays off, it is unused
        g_sup.configured(now,
                         readReg(kRegCtrl3C) == kCtrl3CInit && readReg(kRegCtrl2G) == kCtrl2GInit);
        break;
    case asm330::Action::Check: {
        uint8_t reason = gatr2::kPicoImuReasonNone;
        if (readReg(kRegWhoAmI) != kWhoAmIValue) {
            reason = gatr2::kPicoImuReasonNoResponse;
        } else if (readReg(kRegCtrl2G) != kCtrl2GInit) {
            reason = gatr2::kPicoImuReasonStream; // the chip lost its configuration
        }
        g_sup.checked(now, reason);
        break;
    }
    }
}

void setEnabled(bool enabled) {
    const bool on = g_sup.state() != asm330::State::Off;
    if (enabled == on) {
        return;
    }
    if (enabled) {
        g_sup.restart(time_us_32());
        return;
    }
    g_sup.stop();
    writeReg(kRegCtrl2G, kCtrl2GOff); // best effort; nothing reads it while off
}

void reinit() {
    if (g_sup.state() != asm330::State::Off) {
        g_sup.restart(time_us_32());
    }
}

Status status() {
    const DeviceView d = view();
    Status           s;
    s.enabled  = d.enabled;
    s.state    = wireState(d, g_sup.retry());
    s.reason   = wireReason(d, g_sup.retry());
    s.attempts = g_sup.retry().attempts();
    s.epoch    = g_sup.epoch();
    return s;
}

uint8_t firmware() { return gatr2::kPicoFirmwareAsm330; }

bool readGyroZ(uint32_t, int32_t& gyro_z_mdps) {
    if (!g_sup.running()) {
        return false;
    }
    if ((readReg(kRegStatus) & kStatusGyroReady) == 0) {
        return false;
    }
    gyro_z_mdps = static_cast<int32_t>(readGyroZRaw()) * kMdpsPerLsb;
    return true;
}

size_t formatDiagnostics(char* output, size_t capacity) {
    const int n = snprintf(
        output, capacity, "imu=ASM330 state=%s attempts=%u reason=%u epoch=%u hold_ms=%lu\r\n",
        stateName(), static_cast<unsigned>(g_sup.retry().attempts()),
        static_cast<unsigned>(g_sup.retry().reason()), static_cast<unsigned>(g_sup.epoch()),
        static_cast<unsigned long>(g_sup.holdUs() / 1000));
    return n > 0 && static_cast<size_t>(n) < capacity ? static_cast<size_t>(n) : 0;
}

} // namespace imu

#endif
