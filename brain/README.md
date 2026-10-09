# Brain libraries

Portable C++17 libraries, with host tests and optional PROS motor adapters:

- [investiGATR](investiGATR/README.md): geometry, references, pose and field
  interfaces, and direct or obstacle-avoiding planning.
- [actuGATR](actuGATR/README.md): path following, PID, tank and mecanum control.
- [communiGATR](communiGATR/README.md): retained legacy Pi-link client and
  transports, pending restructure; it does not implement the future MCU link.
- `gatr2_brain.mk`: compiles selected libraries into a PROS project.

Pose comes from a `StateSource`; field geometry comes separately from a
`FieldSource`. The intended MCU supplies pose without field knowledge; the
Brain application owns the field used for planning.

The former operaGATR, locaGATR and robot-specific configuration are in
[deprecated/brain](../deprecated/brain/). No replacement hardware test app
exists yet.

```sh
cmake -S brain -B build-brain
cmake --build build-brain --parallel
ctest --test-dir build-brain --output-on-failure
```

See [planning](../docs/investigatr.md), [control](../docs/actugatr.md),
and [legacy communications](../docs/communigatr.md).
