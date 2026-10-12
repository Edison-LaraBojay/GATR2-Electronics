---
name: project-testing-strategy
description: "GATR2 testing/simulation philosophy (tentative, subject to change) — seam-boundary injection, log/replay, synthetic vs real truth, eyeball-first"
metadata: 
  node_type: memory
  type: project
  originSessionId: 0693582d-d417-490c-8cb4-1a0c07d0b810
  modified: 2026-07-20T02:13:17.064Z
---

Testing and simulation direction worked out with the user (2026-07-19). TENTATIVE, subject to change, none of it built yet. It is a process borrowed in spirit from aerospace SIL/HIL (user referenced a Lockheed tracker pipeline as inspiration, process only, no code taken).

**Core idea: the frame boundaries ARE the injection points.** Because everything talks through frames over a byte stream, a "simulated Pico" is anything that emits valid sensor frames, a "simulated brain" is anything that consumes pose frames. Real code on the other side runs unmodified. This is a payoff of the narrow-seam design (see [[project-wire-format]]).

**One enabling abstraction, triple duty:** put the transport behind a small byte-source/sink interface in the Pi io layer (pico_link/brain_link read from an interface, not a hardcoded UART). That one seam then serves logging (tee frames to disk), replay (source frames from a file), and simulation (source synthetic frames). This is the "generic where it's free" abstraction worth having; everything else stays concrete. The codec is already sim-ready by mandate (common/ has no hardware deps).

**Testability == push I/O to the edges, keep the core pure.** Fusion should be `frames in -> pose out` with no hidden I/O. Pico firmware: split hardware-access (PIO/IMU reads) behind a mockable seam from the pure packing/checksum/framing logic (which then runs on a host). If one function reads+fuses+sends, nothing is injectable.

**Two data sources, complementary:** synthetic (motion model generates ground-truth pose, then synthesizes encoder counts / gyro rate / tag observations) gives PERFECT truth but clean noise; recorded real logs give TRUE sensor noise/timing/dropouts but NO ground truth. Pipeline consumes both identically. Log raw frame bytes (lossless, replayable through real codec); stamp logs with wire-format version or they rot when frames.h changes.

**Ground truth for real runs comes from constrain / measure / invariant, NEVER from the command.** "I told it to spin" is not truth. Ranked cheap-to-expensive: invariant/closure < physical constraint < external measurement (mocap, overkill). Key subtlety: instantaneous invariants (heading flat, x/y zero) require a physical constraint to be valid; closure invariants (returned to a marked start pose) survive sloppy execution. Physical fixtures: ride against a wall/rail for straight lines (constrains true path), center pivot pin for pure rotation, square for 2D loop closure. Straight runs at 0/45/90/180 isolate heading vs cross-track vs forward-scale and catch axis mismatch.

**Output of the sim = the trajectory:** pose (x, y, heading — NO z, ever) over time, plus intermediate signals (innovations, gating decisions, status byte, uncertainty). Not endpoint-only.

**Later phases (user is keen on these, explicitly deferred, NOT to build now):**
- *Signal-injection HIL:* an always-on box with a Pico wired in, registered as a GitHub Actions self-hosted runner, so firmware tests run on real silicon from anywhere. A second RP2040 can synthesize precise quadrature edges via PIO to drive the encoder path (very doable); faking the IMU over SPI is not practical, so stub it. Real payoff is putting the REAL Pico inside the simulated loop (synthetic world -> real firmware -> real UART -> host fusion -> grade vs ground truth); the byte-source seam makes that a config swap, not a rewrite.
- *Mechanical motion rig / rate table:* motor-driven tracking wheel and rotating IMU platform. Tests the WHOLE sensing chain (magnet-to-AS5047P coupling, mounting slop, real angular rate) which signal injection cannot. Primary value is deriving encoder counts-to-mm scale and gyro scale/bias constants. Treat as an on-demand CALIBRATION bench, not a CI gate (moving parts = flaky + wear).
- *Cost caveats:* HIL rigs need physical babysitting; keep them non-blocking. Self-hosted runners on a PUBLIC repo are a real security risk (arbitrary PR code executes on your machine) - matters given the open-source intent.

**Cost/value ladder (cheapest and most automatable first):** host sim + replay -> signal-injection HIL -> mechanical rig -> robot on field with straightedge/closure fixtures.

**Sequencing (matches extract-don't-invent):** SEE IT FIRST. Minimal first artifact = synthetic scripted frames -> real fusion -> matplotlib x-y path plot with heading ticks. Feed a KNOWN shape (line, square) so the eyeball test has a reference. Grading (closure, drift metrics) bolts on later when tuning, not before. Do not build the grading harness before you can see output.

See [[project-overview]], [[project-wire-format]], [[feedback-design-principles]].
