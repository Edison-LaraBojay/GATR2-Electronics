# Vendored third-party code

Everything under `third_party/` is copied into the tree so the Pi build
needs no network and no package beyond the toolchain (and libcamera-dev for
the camera backend). Each entry names the upstream, the license as recorded
in the vendored files, and what this project keeps of it.

## apriltag

- Upstream: AprilRobotics `apriltag` (APRIL Robotics Lab, University of
  Michigan).
- License: BSD 2-Clause, `apriltag/LICENSE.md`, copyright 2013-2016 The
  Regents of The University of Michigan. The same notice heads every source
  file.
- Version: the vendored copy records no version or commit; nothing in the
  files names one.
- Kept: the detector core (`apriltag.c`, `apriltag_pose.c`,
  `apriltag_quad_thresh.c`), the `common/` support library, and the
  families `tag16h5`, `tag25h9`, `tag36h10`, `tag36h11`, `tagCircle21h7`,
  `tagStandard41h12`. The larger families (`tagCircle49h12`,
  `tagStandard52h13`, `tagCustom48h12`) are not vendored.
- Build notes (`CMakeLists.txt`): compiled as C99 with warnings off and
  `NDEBUG` (the upstream pose solver prints debug output otherwise). On
  MinGW the upstream Windows shim `common/pthreads_cross.c` is dropped and
  the library is compiled with `-D__CPTHREAD_H__ -include pthread.h`, so the
  real winpthreads is used; the shim would define global `pthread_*`
  symbols that override it for the whole process.

## stb

- Upstream: Sean Barrett's `stb` single-file libraries.
- Files: `stb/stb_image.h` v2.27, `stb/stb_image_write.h` v1.16.
- License: dual, at the end of each file: MIT (alternative A) or public
  domain via the Unlicense (alternative B), at the user's choice.
- Used by: `src/inspection/jpeg_encoder.cpp` (`stb_image_write`) for the
  greyscale JPEG previews served by the inspection service.

## three

- Upstream: three.js.
- Version: r170 (`three/three.module.min.js` header, `Copyright 2010-2024
  Three.js Authors`, module constant `"170"`); `three/OrbitControls.js` is
  the matching `examples/jsm/controls/OrbitControls.js`.
- License: MIT, `three/LICENSE`, copyright 2010-2024 three.js authors.
- Used by: the browser viewer. The two files and the license are compiled
  into the binary and served under `/vendor/` by the inspection service.

## tinyxml2

- Upstream: Lee Thomason's TinyXML-2.
- Version: 11.0.0 (`TIXML2_MAJOR_VERSION`/`MINOR`/`PATCH` in
  `tinyxml2/tinyxml2.h`).
- License: zlib, `tinyxml2/LICENSE.txt` and the header of each file.
- Used by: configuration parsing (`src/config/`, `src/runtime/system.cpp`).
