---
name: reference-navigatr-windows-build
description: "How to build and run navigatr tests on the user's Windows box (MinGW Makefiles, ucrt64 g++, PATH order fix for 0xc0000139)"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 49e6b669-b8d0-4e92-bf7d-ecc6aa6098e2
  modified: 2026-09-14T08:51:52.805Z
---

On this Windows dev box (2026-09-14): cmake 4.2 at C:/Program Files/CMake, g++ 15.2 from MSYS2 ucrt64 (C:/msys64/ucrt64/bin), no ninja/make/cl/python. Git Bash puts its own /mingw64/bin ahead of ucrt64, so a test exe built by ucrt64 g++ dies at gtest discovery with exit 0xc0000139 (STATUS_ENTRYPOINT_NOT_FOUND, wrong libstdc++ DLL) and CMake deletes it.

Working recipe (from pi/navigatr, Bash tool):

```
export PATH=/c/msys64/ucrt64/bin:$PATH
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j8
ctest --test-dir build --output-on-failure
```

Full build ~3-4 min (FetchContent googletest v1.14 needs network once). build*/ is gitignored. Do not add -static or env hacks to the project CMake for this; it is a local PATH issue.

See [[project-current-state]], [[feedback-conventions]].
