---
name: feedback-design-principles
description: "GATR2 design principles that shape how to write code — extract-don't-invent interfaces, no speculative abstraction, degrade-don't-hang"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 0693582d-d417-490c-8cb4-1a0c07d0b810
  modified: 2026-07-19T04:59:31.256Z
---

Design principles the user wants held when writing code for this project.

**Rules:**
- Interfaces are extracted, not invented: write ONE concrete implementation end to end before freezing an abstraction. Name stages now, freeze signatures after TWO implementations exist.
- Generic only where it's free (sensor mask, landmark abstraction, no game knowledge in Chomp); concrete everywhere else. No speculative abstraction layers, no XML config, no plugin framework on the Pico.
- Every consumer treats its upstream as possibly dead and degrades instead of hanging. Frames stateless; any device can reboot mid-match and recover by scanning for sync.
- The narrow seam is the product. If a change widens the seam or leaks hardware knowledge across it, push back.
- Prefer paraphrase of the project principles in code comments over restating the context doc.
- Localization specifics: pose is (x, y, heading) only — NO z estimate (flat field). Gyro bias calibrated at init while stationary; raw rate on wire enables later moving bias into EKF state. Multiple landmark observations = sequential independent EKF updates. Prefer tag bearing over tag orientation (planar pose ambiguity). Targets = anchor + offset (anchor = odom origin or landmark ID); landmark-anchored targets re-resolve each cycle.

**Why:** User is building a system where the wire contract and boundaries ARE the product (potential MIT open-source library). Speculative abstraction and boundary leaks are the main failure modes they want avoided.

**How to apply:** When implementing anything, resist adding abstraction before a second concrete use exists. Check that changes don't blur the electronics/programming or Pico/Pi/brain boundaries. Never add z, never add game knowledge to Chomp, never add intelligence to the Pico.

See [[project-overview]], [[project-wire-format]], [[feedback-conventions]].
