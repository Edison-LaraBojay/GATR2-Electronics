# Brain libraries

Portable C++17 libraries, with host tests and optional PROS motor adapters:

- [investiGATR](investiGATR/README.md): geometry, references, pose and field
  interfaces, and direct or obstacle-avoiding planning.
- [actuGATR](actuGATR/README.md): path following, PID, tank and mecanum control.
- [spectaGATR](spectaGATR/README.md): publishes a caller-supplied pose to the
  spectaGATR laptop viewer; example PROS project in
  [spectaGATR-demo](spectaGATR-demo/README.md).
- `gatr2_brain.mk`: compiles selected libraries into a PROS project.

Pose comes from a `StateSource`; field geometry comes separately from a
`FieldSource`. The intended MCU supplies pose without field knowledge; the
Brain application owns the field used for planning.

The former operaGATR, locaGATR, communiGATR/translaGATR Pi link and
robot-specific configuration are not distributed in the active tree.
Previous versions remain in Git history.

```sh
cmake -S brain -B build-brain
cmake --build build-brain --parallel
ctest --test-dir build-brain --output-on-failure
```

See [planning](../docs/investigatr.md) and [control](../docs/actugatr.md).
