// spi_bus.h
// Typed contract for a shared SPI controller. Devices with different modes,
// frequencies, and chip selects share one bus safely because per-device
// settings live on the device handle and every transfer is one atomic
// transaction: lock the bus, apply the device's settings, assert its chip
// select, move the bytes, release, unlock. There is no exposed
// setChipSelect/setMode/transfer sequence for callers to interleave.
//
// Bus wiring (controller, pins) belongs to the bus resource configuration;
// per-device settings belong to the sensor or device configuration.

#pragma once
#include <cstdint>
#include <memory>

#include "resources/serial_link.h"

namespace navigatr
{

enum class SpiMode : uint8_t {
    kMode0 = 0,
    kMode1,
    kMode2,
    kMode3,
};

struct SpiDeviceConfig {
    int      chip_select   = -1;
    uint32_t frequency_hz  = 0;
    SpiMode  mode          = SpiMode::kMode0;
    int      bits_per_word = 8;
};

class SpiDevice
{
public:
    virtual ~SpiDevice() = default;

    // Full duplex, one atomic bus transaction. transmit.size and
    // receive.size must match.
    virtual bool transfer(ByteSpan transmit, MutableByteSpan receive) = 0;
};

class SpiBus
{
public:
    virtual ~SpiBus() = default;

    virtual std::shared_ptr<SpiDevice> createDevice(const SpiDeviceConfig& config) = 0;
};

} // namespace navigatr
