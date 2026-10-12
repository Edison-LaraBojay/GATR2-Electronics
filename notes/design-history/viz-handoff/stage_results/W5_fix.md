# W5 review fixes: Brain telemetry scheduling and docs

Date 2026-09-28. Host builds, the Pi e2e harness and PROS builds only. Nothing was committed, flashed, uploaded or run on hardware.

## Finding 1 (medium): telemetry starves when round trip plus gap reaches state_period. FIXED

**Confirmed before fixing.** I reconstructed the pre-fix scheduler (`$S/w5-fix-probe/client_prefix.cpp`) and ran it on the real Pi runtime harness (a copy of `brain_link_e2e_gtest.cpp` with `brain_profile_usb.xml`, 10 s, one report every 100 ms). At 3 or 4 ms per line direction it sent 1 of 100 reports; at 5 or 6 ms it sent 0. `telemetry_dropped` stayed 0 and only `telemetry_replaced` grew, as the reviewer found.

**Change** (`brain/communiGATR/src/client.cpp`, `Client::choose()`):
- A report half a period past its grid time takes the first slot after a state reply. It takes that slot from the due poll, or from a waiting transfer.
- It never takes the slot from a transfer that already yielded its four polls. So at most one exchange sits between two polls.
- After such a send the grid restarts from the send, so there is at most one late send per 1.5 periods.
- New counter `ClientStats::telemetry_overdue`. It is visible through `ProsLinkStatus::stats`.
- The outage drop (report unsent two periods after it was made) now runs before the state poll step, so it also applies when the poll is always due.
- The on-time path is unchanged: the poll not due, nothing else waiting, first slot after a state reply. On fast links nothing changes: the default fakes and the 2 ms e2e harness show 0 overdue sends and the same counts as before.
- New private helpers `telemetrySlot(now, late)` and `startTelemetry()`.

**Tests** (`brain/communiGATR/tests/client_telemetry_gtest.cpp`):
- `SlowTelemetry.*`, on RS-485 with the bus reply after 8 ms and on USB with an 11 ms turnaround, a report every 100 ms:
  - `LateButNeverStarved`: at least 30 sends in 5 s; no wait over 250 ms, including from the start of the run and to its end; sends at least 49 ms apart; at most one non-poll request between two polls; `telemetry_overdue > 0`; no drops or timeouts.
  - `StatePollsKeepMostOfTheirRate`: at least 80% of the polls without telemetry.
  - `SharesTheLinkWithAWaitingTransfer`: with a path report pending on every step, at most one exchange between polls; at least 25 telemetry sends and 25 path reports; no telemetry wait over 250 ms.
- `TelemetryBus.OnASlowBusTakesTheFirstSlotHalfAPeriodLate`, from the bus log:
  - every TELEMETRY is in the first slot after a state reply, with one per poll gap;
  - the longest poll gap is at most the gap without telemetry, plus one TELEMETRY exchange, plus the 5 ms gap, plus 2 ms of rig steps.
- `TelemetryBus.NeverTakesTheTurnOfADuePoll` is renamed `OnTheDefaultBusNeverTakesTheTurnOfADuePoll`. It now also checks `telemetry_overdue == 0`.
- Mutation checks, run on scratch libraries in `$S/w5-fix-mut`:

  | Mutation | New tests that fail |
  |---|---|
  | Pre-fix scheduler | 5 |
  | Transfer exception removed | 2 |
  | Overdue threshold 0 instead of half a period | 2 |
  | First-slot rule removed | 5 |

**Measured**, host only, one report every 100 ms. Polls are counted over the same length of time without and with telemetry.

| Setup | Pre-fix sent | Fixed: sent | Fixed: state polls | Fixed: longest wait for a send |
|---|---|---|---|---|
| Fake USB, default | 50/50 | 50/50 (on time) | 243 to 224 | 114 ms |
| Fake USB, turnaround 8 ms | 50/50 (reviewer) | 50/50 (on time) | 243 to 205 | 118 ms |
| Fake USB, turnaround 11 ms | 0/50 (reviewer) | 32/50 (overdue) | 237 to 205 | 166 ms |
| Fake USB, turnaround 25 ms | 0/50 (reviewer) | 29/50 | 142 to 113 | 180 ms |
| RS-485 bus, default | 50/50 | 50/50 | 243 to 217 | 116 ms |
| RS-485 bus, reply after 8 ms | 0/50 (reviewer) | 32/50 | 232 to 200 | 171 ms |
| e2e harness, 2 ms per direction, 10 s | 100/100 | 100/100 | 497 and 500 | 124 ms |
| e2e harness, 3 or 4 ms per direction | 1/100 | 63/100 | 489 to 437 | 160 ms |
| e2e harness, 5 or 6 ms per direction | 0/100 | 63/100 | 488 to 437 | 160 ms |
| e2e harness, 8 ms per direction | not run | 59/100 | 324 to 274 | 180 ms |
| e2e harness, random 1 to 4 ms per line | 67/100, longest 370 ms, 5 waits over 250 ms | 79/100 | 490 to 431 | 162 ms |
| e2e harness, random 2 to 5 ms per line | 83/100, longest 240 ms | 87/100 | 488 to 413 | 162 ms |
| e2e harness, random 0 to 8 ms per line | 82/100, longest 290 ms, 1 wait over 250 ms | 86/100 | 458 to 399 | 168 ms |

- Every e2e run had at most one non-poll request between two GET_STATE requests, 0 timeouts and 0 drops.
- The probes are in `$S/w5-fix-probe`: `make_probe.sh` builds the e2e variants from the repo's e2e test, and `host_starve.cpp` runs the fake rows.
- The e2e harness links the real Pi runtime from my own build, `$S/w5-fix-e2e`.

## Finding 2 (low): docs understate the poll delay and overstate the rate limit. FIXED
- `docs/brain_setup.md` section 10 and `brain/robot/gatr2_robot.h` now say that each exchange can push the next state poll back by about 10 to 15 ms (one exchange plus the 5 ms gap).
- `docs/communigatr.md` now says: "On average at most one send per `telemetry_period` ... two sends can be as close as half a period".
- Also updated for finding 1:
  - `communigatr.md`: the Telemetry rules (on time and overdue), a cost table (host fakes and the e2e harness), the scheduling note, the stats list, the tests row and the limitations;
  - `brain_setup.md`: a "Slow link" paragraph (V5 USB timing plausible but unmeasured);
  - the `client.h` comments.

## Files changed (all W5-owned)
- `brain/communiGATR/src/client.cpp`
- `brain/communiGATR/include/communigatr/client.h`
- `brain/communiGATR/tests/client_telemetry_gtest.cpp`
- `brain/robot/gatr2_robot.h` (a comment only)
- `docs/communigatr.md`
- `docs/brain_setup.md`

## Test commands and results
Git Bash, with `/c/msys64/ucrt64/bin` first on PATH.
```
cmake -S brain -B $S/w5-fix-brain -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=$S/nav-base/_deps/googletest-src
cmake --build $S/w5-fix-brain -j16
ctest --test-dir $S/w5-fix-brain -j16
```
- Brain host suite, fresh build: 0 warnings, 451/451 passed. Before the fix it was 444; there are 7 new tests and 1 renamed.
- The telemetry, attitude and timing subset passed 41/41 on each of 3 repeats.
- Pi e2e: `cmake -S pi/naviGATR -B $S/w5-fix-e2e ...` and `--target brain_link_e2e_tests` on the current tree with the fixed client: 0 warnings, 9/9 passed.
- PROS, using the spec section 0 recipe: operaGATR and locaGATR both link with 0 warnings and 0 errors, and `client.cpp` was recompiled in both.
- ARM: `arm-none-eabi-g++ -std=c++17 -fsyntax-only -Wall -Wextra` on `client.cpp` is clean.

## Contract issues
1. **The fix refines amendment 12.1.** "It never takes the turn of a poll that is due" now holds for on-time sends only. A report that is half a period late takes the first slot after a state reply from the due poll or from a waiting transfer.
   - It is still only in that first slot.
   - It still means at most one exchange between two polls.
   - Late sends happen at most once per 1.5 periods.

   12.1 names starvation as a finding, and starvation cannot be avoided without this change: when a state exchange plus the gap fills the 20 ms period, no idle slot exists. The lead should confirm or amend 12.1. To go back to the strict rule, delete the overdue block in `choose()`; the reviewer's strict-rule alternative would then be needed (counters, a "Telemetry waiting" screen text, and docs).
2. **For the lead's `brain_link_e2e_gtest.cpp`:** a TELEMETRY case at a 3 ms line delay is ready in `$S/w5-fix-probe/e2e_telemetry_case.cpp.inc`.
   - It needs `kLineUs` to become a Rig/Wire parameter.
   - I verified it in my scratch copy of the harness: it passes with the fix and fails on the pre-fix scheduler.
   - At 2 ms it would fail its `telemetry_overdue > 0` check by design, so run it only at 3 ms.

## Open risks
- On a slow link telemetry drops to about 6 per second, and state polls lose about 11 to 20% (host numbers). The V5 USB round trip is unmeasured; every number here comes from host fakes or the Pi e2e harness with modeled line delays.
- The program screens still show whether telemetry is on, off or refused; they do not show whether it is flowing. That is now bounded by the fix, and `telemetry_overdue` and `telemetry_reports` are in the status.
