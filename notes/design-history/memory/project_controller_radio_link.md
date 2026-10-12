---
name: project-controller-radio-link
description: 2026-10-10 measured V5 controller USER_READ link; pit channel ~235 ms dead time per poll (bursty, 3 updates/s), download channel 30-50 ms RTT and clean 10 Hz; receiver flags and Brain FIFO gate
metadata:
  type: project
---

Measured on the user's bench, 2026-10-10, Brain -> radio -> controller -> USB -> spectagatr receiver.

**Pit channel (default radio):** every USER_READ costs about 235 ms dead time plus ~0.7 ms/byte, whatever the read length or pipelining. The controller serves one request at a time (two outstanding doubles the RTT). Best case ~570 B/s: 10 Hz arrives but as 4 poses per reply every ~400 ms, so the viewer updates ~3 times a second.

**Download channel (`spectagatr --radio-download`):** RTT 28-47 ms, 10 Hz one pose per reply, gaps 110-180 ms, no loss. This is the demo setting. Unverified: joystick latency while driving, legality under field control. The link drops for seconds during the switch; the receiver switches back to pit on Ctrl+C.

**Read length:** length 0 (vexide style) returns up to 224 bytes per reply; pros-cli's 64 limits replies to 64. Receiver default is now 0.

**Brain side:** `ProsPublisher` writes only while at most 128 B wait in the VEXos stdio FIFO (`vexSerialWriteFree(1)`), with the empty size learned as the largest free count seen. Without it, a slow link made poses ~14 s old.

**Why:** the user saw 500 ms updates and STALE periods; the chain was Brain backlog + pit-channel dead time.

**How to apply:** for controller viewing, run `--source controller --debug-link --radio-download`; read the `delivery` line (rate_hz, max_gap_ms, seq_lost, poses_per_reply). Don't expect pipelining or read length to beat the pit channel's ~3 transfers/s. Related: [[project-spectagatr-standalone]], [[project-encoder-localization-demo]].
