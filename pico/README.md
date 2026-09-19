# Pico acquisition firmware

RP2040 firmware reads three A/B quadrature encoder channels and the
ASM330LHHG1 gyroscope's Z-axis yaw rate over SPI1. It sends timestamped sensor
frames to the Pi over UART at 50 Hz; localization runs on the Pi.

Encoder counts accumulate through GPIO interrupts. The IMU driver polls for
new yaw-rate samples and converts them to millidegrees per second. A missing
sample leaves that sensor's bit clear in the outgoing frame. The accelerometer
is disabled, and the firmware does not produce roll, pitch, or quaternion
attitude measurements.

[Pin assignments and rates](src/config.h), [the IMU driver](src/imu.cpp), and
[the shared wire format](../common/frames.h) define the current hardware and
protocol. The IMU interrupt pin is reserved in the pin map but is not used by
the polling driver.

Build with PlatformIO from this directory:

```sh
pio run
```

The [PlatformIO configuration](platformio.ini) selects the Pico board and
Arduino core. Upload with `pio run --target upload` when the board is connected.
