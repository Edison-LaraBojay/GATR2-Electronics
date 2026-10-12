---
name: project-encoder-localization-demo
description: "2026-10-09 encoder-to-Brain localization demo (Pico hat2_encoder_demo -> pi/encoder-relay -> brain/localization-demo -> spectaGATR); built + host tested, nothing on hardware; uncommitted"
metadata:
  node_type: memory
  type: project
  originSessionId: b09f0a6f-981b-4ac6-87b0-d1f7091a1001
  modified: 2026-10-09T23:21:14.982Z
---

Built 2026-10-09 for the 2026-10-10 demo, separate from naviGATR/communiGATR. Uncommitted.

**Path:**
- The Pico env `hat2_encoder_demo` (`pico/aggreGATR/src/encoder_demo/`) sends a 16-byte record at 50 Hz on UART0 (GP16/17, 115200).
- `pi/encoder-relay` forwards each record as an `ENC1:`+hex line to the Brain USB user port (2888:0501 if02). It has a systemd unit with `Conflicts=navigatr.service`.
- `brain/localization-demo` (PROS, slot 4) reads stdin in one task, using a seqlock slot. It combines the counts with the VEX IMU on port 20 and publishes through spectaGATR `viz.updatePose`.
- The record codec lives in `encoder-record/`, shared by all three.

**Why hex lines:** the PROS serial daemon parses 'p','R' sequences from host bytes as kernel commands. Raw binary must never reach stdin. The spectaGATR session (d0) pointed this out.

**Ownership:** the spectaGATR session owns spectaGATR/, brain/spectaGATR* and brain/gatr2_brain.mk (spectagatr entry). deprecated/ is local only and ignored by Git, a user-authorized cleanup on 2026-10-09; the fallback is pinned commit 10c568e. The demo Makefile appends the codec to GATR2_SRC instead of editing gatr2_brain.mk.

**Unverified:**
- the relay was never built on Linux; it was only syntax-checked with stub headers;
- nothing has run on hardware;
- the controller viewing route next to the Pi USB link is unknown (single shared stdout FIFO?). The fallback is `encoder-relay --echo-brain | spectagatr --source file --replay -`.

**Calibration:** the archived 23 in pushes read about half the distance, so the scale is about 2. The cause, CPR or diameter, is unresolved. Related: [[project-spectagatr-standalone]], [[project-wire-format]].
