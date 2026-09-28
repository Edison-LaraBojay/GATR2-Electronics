# Vendored third-party code

Copied into the tree so the firmware builds with no package beyond the
PlatformIO platform. PlatformIO picks up each folder here as a library
through `lib_extra_dirs`.

## sh2

- Upstream: CEVA SH-2 sensor hub driver, https://github.com/ceva-dsp/sh2
- Version: tag `v1.4.0`, commit `b514b1e2586ddc195e553dac89fc94c637b25298`.
- License: Apache-2.0, stated in the header of each source file, plus
  `sh2/NOTICE.txt`. Upstream ships no separate LICENSE file.
- Kept: `sh2.c`, `sh2.h`, `sh2_err.h`, `sh2_hal.h`, `sh2_util.c`,
  `sh2_util.h`, `shtp.c`, `shtp.h`, unmodified. `sh2_SensorValue.*` and
  `euler.*` are not vendored; the driver decodes accelerometer and uncalibrated
  gyro XYZ reports in `src/bno08x_reports.cpp`.
- Used by: `src/imu_bno08x.cpp`, the `hat2_bno08x` environment only.
