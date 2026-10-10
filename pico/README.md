# pico

RP2040 sensor acquisition software retained for the future MCU rewrite.

- [`aggreGATR/`](aggreGATR/README.md): the PlatformIO acquisition firmware. It reads three encoder channels and an IMU at 50 Hz, with optional local USB diagnostics. Build it from `pico/aggreGATR` with `pio run`.

The retired Pi UART transport and commands are not included here. The active firmware has no Pi
or Brain transport and performs no localization. Existing board pin assignments
remain available for sensor bring-up; the old board design is in
[`pcb/PiHat/`](../pcb/PiHat/).

Component names: [docs/naming.md](../docs/naming.md).
