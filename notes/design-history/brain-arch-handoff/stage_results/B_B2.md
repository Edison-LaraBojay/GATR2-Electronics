Stage B2 is complete. The Pi now has explicit collision data in the field definition, serves the field map and field estimate documents over READ_DOC, and generates `brain/testing/include/field_references.h`. A clean rebuild of `$S/wB-nav` (`--clean-first`, `-Wall -Wextra`) exited 0 with no warnings, and ctest passed 403/403.

**Commands** (Git Bash, `/c/msys64/ucrt64/bin` first on PATH, `$S` = scratchpad):
- `cmake -S C:/GATR2-Electronics/pi/navigatr -B $S/wB-nav -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=$S/nav-base/_deps/googletest-src`
- `cmake --build $S/wB-nav -j8 --clean-first`: exit 0, 0 warnings.
- `ctest --test-dir $S/wB-nav -j8`: 403/403 passed. That is 377 from B1, plus 19 new document tests, 7 new field-asset tests and 1 new brain_io test; one parallel-wheels test was reworked and renamed.
- `cmake --build $S/wB-nav --target field_references` writes the header. A test regenerates it and requires the checked-in file to match byte for byte.
- The generated header compiles cleanly with host g++ `-std=c++17` and with the PROS arm-none-eabi-g++ 13.3.1 (gnu++20).
- Mutation check in a scratch copy (`$S/wB-mut`, not the repo): 21 mutants, all caught. Three survived the first round (heading-only change, publisher reset, object sort); I added a test for each.

**What changed** (all files are mine):
- **Parser** (`src/config/field_map.{h,cpp}`, plus strict-schema helpers in `config_node.{h,cpp}`):
  - adds `revision` on the root, a `Boundary`, a `wire_id` on each `Landmark` and `Obstacle`, and at most one `CollisionBox` per landmark or obstacle;
  - unknown children and attributes on the root, `Landmark`, `Obstacle`, `Boundary` and `CollisionBox` are errors;
  - wire ids must be 1..65535 and unique across landmarks and obstacles.
- **Documents** (new `src/impl/publishing/field_documents.{h,cpp}`):
  - The map is built once and sorted by wire id; `map_id` is its CRC-32. A field with no revision, no Boundary, a landmark without `wire_id`, or more than 128 objects fails the build with the reason.
  - Estimates are rebuilt each reporting cycle. A landmark is "observed" only when its estimate comes from the robot's current odometry epoch, and its pose is composed with the current anchor; everything else is nominal, so noop world estimation gives all nominal.
  - A new estimate id is taken only when content changes and at least `estimate_period_ms` has passed. The last three are kept for chunked reads.
- **Publisher** (`brain_link_publisher.{h,cpp}`): takes `<Field resource_id=".." estimate_period_ms="200"/>`, fills `map_id`/`estimate_id` in the state block, and answers READ_DOC. It returns Stale, Unavailable or InvalidArgument as spec section 3 describes; `reset()` drops the stored estimates.
- **Generator**: new tool `navigatr_field_refs` (`src/app/field_refs.cpp`) and the `field_references` CMake target. Current output: `kMapId = 0xF6B3F2C9`, `kRevision = 1`, nine references such as `NeutralGoal0Center` (object 5).
- **`config/override/field.xml`**:
  - revision 1; Boundary from 0 to 3.5664 m on both axes;
  - goals are wire ids 1 to 9 in file order, each with a collision box;
  - the four loaders (101 to 104) and four toggles (111 to 114) are fixed obstacles, each box matching its display footprint;
  - tape is not an obstacle; every value carries a source note and is marked provisional.
- **Checked-in brain_link pipelines**: all of them now publish the Override field, including the current USB bench `bench_vex_imu_usb.xml`. I also added the field include to the `three_wheel_imu.xml.in` template, which lacked it.
- **Docs**: `field_assets.md` has a new planning-data section; the Brain output section of `landmarks.md` is rewritten for v4; the `field_map` entry in `docs/navigatr_resources.md` is updated. I also fixed the passages that still described single-landmark selection in `architecture.md`, `coordinates.md`, `setup.md`, `parallel_wheel_bringup.md`, `calibration_inventory.md` and the navigatr README.

**Deviations and contract notes (none blocking):**
- **Goal box is 0.1543 m, not the spec's 0.1542.** The octagon's enclosing circle is 0.15424 m, so 0.1542 would fail the required "box contains the goal base" test by 0.04 mm. Both round to 155 mm on the wire, so nothing downstream changes.
- **Rounding on the wire**: box sizes round up and the boundary rounds inward, so the wire never shrinks an obstacle or grows the drivable area.
- **Estimate age**: `age_ms` is the age when the estimate was taken, and aging alone does not produce a new estimate. Worker A should add the time since receipt when judging observation age.
- **Estimate ids** restart at 1 after a Pi process restart but never repeat within a process, even across `System::reset`. Key estimates by `(pi_instance, estimate_id)`. `map_id` depends only on content, so a Brain map cache keyed by it survives Pi restarts.
- **Amendment 8.2 changed `common/` during this stage.** It broke 7 of my brain_io tests, and I fixed them:
  - Profile document lengths are now taken from the contract constants instead of the literals 72 and 96.
  - The "undefined op" test uses op 12, because op 11 (READ_WHEELS) now exists.
  - Until stage B3/B4, READ_WHEELS answers Unavailable without a profile host and NotReady with one. CONTROL accepts actions 1 to 4 (all still NotReady); action 5 and above is InvalidArgument.

**Not done / for others:**
- Inspection and viewer output for the boundary, boxes and `map_id` (spec 5.5) is still to do; the builder is ready for it.
- The collision boxes are derived from the manual and CAD, not measured. Nothing here has run on hardware.
- Brain code can include `field_references.h`. After any planning edit to `field.xml`: bump `revision`, run the `field_references` target, and rebuild the Brain apps.
- Worker D: my edits to both navigatr CMakeLists files were small insertions with no reformatting.
- I did not touch `pi/navigatr/src/resources/pico_control.h` (the lead's contract file).

Full details are in `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/brain-arch-handoff/progress/wB.md` (Stage B2 section).