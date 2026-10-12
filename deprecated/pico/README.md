# Retired Pico communication code

`aggreGATR/` preserves the former Pi UART helpers and their tests, plus the
original communication-heavy `main.cpp`, configuration, board map, PlatformIO
settings, test CMake file and README. Paths within that project are retained
so the old implementation can be inspected alongside the active firmware.

This is a partial historical snapshot, not a standalone build target. The
complete firmware that sends Pi telemetry is `pico/aggreGATR` at commit `10c568e`;
[FALLBACK.md](../FALLBACK.md) shows how to build and flash it. Its
README describes the retired Pi architecture and may contain historical links.
The active acquisition code, IMU drivers and sensor tests remain in
[`pico/aggreGATR`](../../pico/aggreGATR/README.md). The archived files are excluded
from that project's firmware and host test builds.

The current firmware retains acquisition and optional local USB instrumentation.
It has no Pi UART commands, framed sensor/status/diagnostic stream, Brain
transport, or localization implementation.
