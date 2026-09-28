# Diagnostic capture and export

A short, bounded recording of what the Pi runtime already sees: robot state
publications, Pico frames, Brain requests and telemetry, paths, lifecycle
events and, when asked, raw link bytes. It runs on the Pi, so the browser and
the network never decide which samples are kept, and it ends in a ZIP bundle
of CSV files plus metadata that any tool can open.

Capture records more of the data that exists. It never changes a sensor
rate, a localization setting or control timing, and it never sends anything
to the Brain or the Pico.

## How it works

- Producers (the localization feed, the Pico telemetry resource, the Brain
  link command and publisher slots, `System::noteEvent`, the link monitors)
  post small fixed-size records to the System's `DiagnosticsHub`
  (`src/diagnostics/hub.h`). Posting is one short mutex and one copy into a
  bounded ring; a stream nobody wants costs one atomic load.
- The recorder (`src/capture/capture_recorder.h`) is the hub's only
  consumer. Its own thread drains the ring every 20 ms.
- While idle it keeps a rolling window (`rolling_s`) of the default streams.
- A trigger copies the last `pre_s` of that window, records the selected
  streams until `post_s` after the trigger, then builds the bundle on the
  same thread and keeps it.
- If the recorder ever falls behind, the hub drops new records and counts
  them per stream. A producer never waits, and the drops are reported in the
  bundle.

## Configuration

`<Capture>` is an optional child of `<System>` or of a `<Configuration>`
profile (every shipped config). Without it, capture is enabled with the
defaults below, which is enough for manual captures. Automatic triggers and
directory writes need the element.

The defaults:

```xml
<Capture enabled="true" rolling_s="10" max_pre_s="30" max_post_s="60"
         default_pre_s="10" default_post_s="15" max_records="400000"
         max_mb="64" keep="3" directory="" auto=""
         auto_cooldown_s="120" auto_max_per_hour="6"/>
```

| Attribute | Range | Meaning |
|---|---|---|
| `enabled` | bool | `false`: no rolling window, no hub traffic, every start refused |
| `rolling_s` | 0..120 | seconds of default streams kept while idle; 0 turns the window off |
| `max_pre_s` | 0..120 | largest pre-trigger interval a request may ask for |
| `max_post_s` | 1..600 | largest post-trigger interval |
| `default_pre_s`, `default_post_s` | up to the maxima | used when a request (or an automatic trigger) gives none |
| `max_records` | 1000..5000000 | records one capture may hold |
| `max_mb` | 1..1024 | records held (`max_mb` MiB of `sizeof(DiagRecord)`, before allocator overhead), and the CSV text of one bundle; see "Memory and storage bounds" |
| `keep` | 1..20 | finished bundles kept, newest first |
| `directory` | path | empty: memory only. Otherwise each bundle is also written to `<directory>/<id>.zip` |
| `auto` | list | automatic triggers: `link_lost`, `continuity_lost`, `pico_reboot`, `profile_changed`; empty is off |
| `auto_cooldown_s` | 10..86400 | minimum time between two automatic captures |
| `auto_max_per_hour` | 1..60 | automatic captures in any hour |

The pre-trigger interval can only come from the rolling window: a capture
never holds more than `rolling_s` before its trigger, whatever it asks for.
The status `limits.max_pre_s` reports the effective bound.

Apply a change by restarting naviGATR. Nothing on the Brain or Pico changes.

## Behavior

**Timeline.** trigger, then `pre_s` from the rolling window, then record
`post_s`, then finalize (build the ZIP) on the capture thread, then `ready`.

**One capture at a time.** A start while one is recording or finalizing is
refused as busy (HTTP 409). Every viewer sees the same status, and any viewer
can cancel it.

**Streams.**
- The default is every stream except raw `bytes`.
- `bytes` needs an explicit choice. While such a capture records, the link
  monitors feed the bytes they already see to the hub. Nothing reads a
  serial port a second time.
- Streams outside the default start at the trigger: the rolling window holds
  only the default streams.

**Limits.** Reaching `max_records` or `max_mb` stops recording early. The
bundle is then marked `truncated` with `stop_reason` `record_limit` or
`memory_limit`, and the refused records are counted per stream.

`truncated` is true whenever a capture bound left records out, and
`truncated_by` names each cause: `record_limit`, `memory_limit`, `shutdown`,
or `bundle_size_limit` (CSV rows over `max_mb`). A capture can end at its
post window (`stop_reason` `post_window`) and still be truncated by the
bundle size. `metadata.json` and the status give the same verdict. Records
the hub ring dropped are counted in `drops` (and the status `dropped`), not
in `truncated`.

**Cancel.** A cancelled capture is discarded, and nothing is kept.

**Viewers.** A browser disconnecting, or every browser leaving, changes
nothing: the capture finishes on the Pi and waits in the list.

**Restarts during a capture.** A Pi `reset()`, a new odometry epoch or
placement, a Pico reboot or acquisition restart, a new Brain session or a new
profile never stops a capture.
- Every row carries the identifiers that say which segment it belongs to.
- `metadata.json` lists the segments of each domain.
- Nothing is joined across them. For example, encoder changes are never
  computed across a Pico reboot, and the unwrapped heading restarts in each
  robot segment.

**Shutdown.** naviGATR stops the inspection service, then the recorder, then
the workers. A capture still recording at that point is closed as `truncated`
with `stop_reason` `shutdown`. With `directory` set, it is written there; a
memory-only bundle ends with the process.

**Automatic triggers.** An automatic trigger starts a capture only when the
recorder is idle, the cooldown has passed and fewer than `auto_max_per_hour`
started in the last hour. Otherwise the trigger is counted as suppressed
(`busy`, `cooldown`, `hourly_cap`). An automatic capture uses the default
pre/post and streams, `reason` is the trigger name, `requester` is `auto`,
and `detail` holds the event text.

| Trigger | Fires on |
|---|---|
| `link_lost` | "Brain link quiet for 1 s", or "Pico frames lost" |
| `continuity_lost` | localization ends pose continuity (a sensor or motion loss while placed, not the warn-only case) |
| `pico_reboot` | a new Pico boot seen on the link |
| `profile_changed` | a Brain robot profile applied at the boundary |

Each fault edge is noted once, and the cooldown and the hourly cap bound the
rest, so a persistent fault cannot fill the Pi with captures.

**Directory writes.**
- Each bundle is written to `<directory>/<id>.zip.part`, then renamed.
- A failed write is reported in the status (`file_error`), and the bundle
  stays downloadable from memory.
- The directory keeps the same newest `keep` bundles as memory. The recorder
  deletes only files it wrote in this run; files from earlier runs are never
  touched.

## Memory and storage bounds

| What | Bound |
|---|---|
| Records held (rolling window plus the active capture) | `min(max_records, max_mb MiB / sizeof(DiagRecord))`, shared; the rolling window gives way first. Both are deques that grow in small blocks, so there is no doubling past the bound; the allocator adds its per-block overhead |
| One bundle's CSV text | `max_mb` MiB, written in chunks of at most 1 MiB. Rows past it are left out and counted (`bundle_size_limit`), and the bundle is marked truncated |
| Bundles in memory | `keep` |
| Files on disk from one run | `keep` |
| While a bundle is built | the records and the CSV text, then the CSV text and the ZIP: the records are released once the rows are written. About two times `max_mb` plus allocator overhead |
| Hub ring | 16384 records; drops counted per stream |
| Host metadata samples | one small JSON text per second of `rolling_s` (plus one) |
| Rolling-window eviction counts | one 16-byte entry per millisecond in which the record bound evicted something, over `rolling_s` |

Measured on the development host (Windows 11, MinGW GCC 15.2, Debug -O0, the
UCRT heap; a scratch probe, not a repo test, not the Pi). With the defaults
(`max_mb` 64, cap 399457 records of 168 bytes, 64.0 MiB) and only an active
capture holding records:
- at the cap, process private memory grew by 72.9 MiB (block overhead of
  this heap, about 14%);
- through the bundle build, the peak was 132 MiB above the start;
- for comparison, the first version's `std::vector` of records measured
  87.9 MiB at the cap (a reviewer's probe, same host), and the deque with
  one growing CSV string still peaked 242 MiB above the start while
  building.

glibc on the Pi has a different per-block overhead; it was not measured.
With the defaults and a typical Brain-profile run (hundreds of records per
second), a 25 s capture holds a few MB. The bounds only matter if something
floods the hub.

## HTTP

The inspection service routes these to `src/capture/capture_http.h`.

| Route | Result |
|---|---|
| `POST /api/capture/start?pre_s=&post_s=&streams=&requester=` | `{"ok":true,"id":"..."}`; 409 busy; 400 bad input or disabled |
| `POST /api/capture/cancel?id=` | `{"ok":true}`; 404 when that id is not recording |
| `GET /api/capture/status` | the status object below |
| `GET /api/capture/<id>.zip` | `application/zip`; 404 until ready or after it left the `keep` list |

- `streams` is a comma list of stream names, `default` or `all`. Omitted, it
  means `default`.
- Omitted `pre_s` or `post_s` take the configured defaults. Values over the
  maxima are clamped, and the metadata shows both the requested and the
  clamped values.
- `requester` is a free label (printable ASCII, at most 48 characters).
- Stream names: `robot_state`, `pico_sensor`, `pico_status`, `pico_diag`,
  `vex_imu`, `brain_request`, `brain_telemetry`, `path`, `event`, `bytes`.

## Status object

`CaptureService::writeStatus`, also sent to viewers as the `capture`
message whenever `version()` changes. The version changes on every state
change and at most once per second while recording, for progress.

```text
{ "available": bool, "schema": "gatr2.capture/1",
  "state": "idle" | "recording" | "finalizing" | "disabled" | "stopped",
  "pi_host_us": n,
  "active": null | { "id", "state", "reason", "requester", "detail"?, "streams": [...],
                     "pre_s_requested", "post_s_requested", "pre_s", "post_s",
                     "trigger_pi_host_us", "end_pi_host_us", "elapsed_s", "remaining_s",
                     "records", "limit_dropped" },
  "captures": [ { "id", "state": "ready", "url", "reason", "requester", "detail"?,
                  "streams": [...], "pre_s", "post_s", "trigger_pi_host_us",
                  "end_pi_host_us", "truncated", "truncated_by": [...], "stop_reason", "records",
                  "rows": {stream: n}, "dropped": {stream: n}, "size_bytes",
                  "file": path | null, "file_error": text | null } ],   // newest first
  "last": null | { "id", "outcome": "ready" | "cancelled" | "failed", "error" },
  "limits": { "rolling_s", "max_pre_s", "max_post_s", "default_pre_s", "default_post_s",
              "max_records", "max_mb", "record_cap", "keep", "directory" },
  "streams": { "default": [...], "all": [...], "rolling": [...] },
  "auto": { "triggers": [...], "cooldown_s", "max_per_hour", "fired", "fired_last_hour",
            "suppressed": {"busy", "cooldown", "hourly_cap"}, "seen": {trigger: n},
            "last_reason", "last_pi_host_us", "notices_dropped" },
  "hub": { "capacity", "queued", "dropped": {stream: n} } }
```

`rows` and `dropped` list only nonzero streams. `dropped` adds the hub ring,
capture limit and bundle size losses.

## Bundle (`gatr2.capture/1`)

A store-only ZIP. There is no compression; CRC-32 is the zlib CRC,
`translagatr::crc32`. It holds:
- `metadata.json` and `README.txt`;
- one CSV per selected stream.

A selected stream with no rows still has its file, with the header only. A
stream that was not selected has no file.

| File | Stream | One row per |
|---|---|---|
| `events.csv` | `event` | runtime event (`kind` runtime), plus the capture's own markers (`kind` capture: trigger, end, Pi resets) |
| `robot_state.csv` | `robot_state` | localization publication |
| `pico_sensor.csv` | `pico_sensor` | decoded Pico sensor frame |
| `pico_status.csv` | `pico_status` | Pico status frame |
| `pico_diag.csv` | `pico_diag` | Pico diagnostic frame |
| `vex_imu.csv` | `vex_imu` | Brain VEX IMU sample carried by GET_STATE |
| `brain_requests.csv` | `brain_request` | Brain request and the result the Pi answered |
| `brain_telemetry.csv` | `brain_telemetry` | Brain TELEMETRY report |
| `paths.csv` | `path` | PATH_REPORT vertex (field frame); an empty path is one row |
| `transport_bytes.csv` | `bytes` | chunk of raw link bytes (at most 64), hex |

### CSV conventions

**Format.**
- There is a header row, and units are in the column names: `_m`, `_deg`,
  `_deg_s`, `_m_s`, `_ms`, `_us`, `_counts`, `_bytes`, `_rpm`.
- An empty field means missing (not measured, not valid, not applicable). It
  never means zero.
- Booleans are `0`/`1`, numbers use a decimal point, and lines end with `\n`.
- Text is quoted per RFC 4180 when it holds a comma, a quote or a line break.

**Identifiers and clocks.**
- Every row starts with `pi_host_us, pi_session, reset_count`, and every
  record row has `source`, the producer's name.
- `pi_host_us` is when the Pi posted the record. It is the Pi steady clock,
  microseconds since naviGATR started, and it is the one clock shared by
  every file.
- Rows whose source has its own clock add `source_clock` (`pico` or
  `brain`) and `source_ms`. Never subtract timestamps of different clocks.
- The identifiers that apply are on each row: `odometry_epoch`,
  `anchor_revision`, `boot_id`, `acq_epoch`, `imu_epoch`, `brain_session`,
  `request_id`, `seq` and `publication`.

**Units and precision.**
- Headings are degrees, counterclockwise from +x, wrapped to (-180, 180].
  `*_heading_unwrapped_deg` is continuous within one robot segment only.
- Positions are written to 1e-6 m, speeds to 1e-5 m/s, angles to 1e-4 deg,
  and wire-derived values to their wire resolution (mm, centidegrees).

**Frames.**
- `odom_*` values are in the odometry frame, which is continuous within one
  `odometry_epoch`.
- `field_*` values are in the field frame. They are empty while the robot is
  not placed.
- Body-frame values say so (`cmd_body_*`). +x is forward, +y left, +z up.
- Roll and pitch are given only when `attitude_status` is `measured`. Roll
  is about +x (left side up is positive), pitch is about +y (nose down is
  positive). Height is never estimated.

**Honest readings.**
- `robot_state.csv` has one row per localization publication, and
  localization publishes every loop cycle. Between measurements the rows
  repeat the pose. `new_measurement` is 1 when the estimator took a new
  measurement for that publication (`DiagRobotState::advanced`, the same
  flag that lets the state into pose history), and 0 when the row repeats
  the held pose: a loop cycle without a new measurement, a reset, or a
  continuity loss. It is never empty. Filter on it before treating rows as
  samples.
- `pin_*_level` in `pico_diag.csv` are logic levels read back from the pad:
  1 HIGH, 0 LOW, empty when not sampled.
  - They are not voltages, and a sample can miss fast transitions.
  - UART RX idles HIGH, so a HIGH there proves nothing about the wire.
- `enc*_delta_counts` is one frame's change: against the same port in the
  frame right before it (`seq` - 1) of the same boot and acquisition epoch.
  It is empty after a lost frame, at a reboot or restart, and when the port
  was missing from the frame before. A zero cannot tell a stationary encoder
  from a disconnected one.
- `seq_gap_frames` in `pico_sensor.csv` counts the Pico sensor frames
  between the previous recorded frame of that acquisition and this row that
  are not in the file: lost on the wire, refused by the frame reader, or
  dropped by the hub ring. 0 means consecutive; empty for the first frame of
  an acquisition. It is `seq` modulo 256, so a loss of a multiple of 256
  frames cannot be seen. Across a gap, the `enc*_counts` difference is the
  travel over several frames.
- `pico_diag.csv` and `brain_telemetry.csv` keep `payload_hex` next to the
  decoded columns. `decoded` 0 means the translaGATR codec refused the
  payload, and the other columns are then empty.
- `target_valid` in `brain_telemetry.csv` is TELEMETRY flag bit 3
  (`translagatr::kTelemetryTarget`). `target_field_x_m`,
  `target_field_y_m` and `target_field_heading_deg` hold a resolved
  destination only when it is 1; when it is 0 they are empty, whatever the
  wire carried. Like the rest of the motion group, `target_valid` is empty
  when `motion_present` is 0.

The exact header of every file is in the bundle's `README.txt`, generated
from the same code that writes the rows.

### `metadata.json`

| Key | Content |
|---|---|
| `schema`, `id` | `gatr2.capture/1`, the capture id |
| `created` | `pi_host_us` of the trigger; `unix_ms` from the Pi system clock (wrong without network time or RTC) or null |
| `pi` | `session`, `resets` (from which `pi_host_us` each reset count applies) |
| `configuration` | configuration id, name, digest, loop rate, commands slot, estimator, warnings |
| `profile` | the running Brain robot profile: id, generation, summary, topology, IMU source and port, footprint, calibration settings, wheels (port, CPR, gear, radius, position, angle, polarity, travel scale, sensor), cameras; null when none |
| `localization` | estimator, readiness, stationary, clock mapping, continuity breaks, and per observation function: calibration state and attempts, gyro bias, dropped intervals |
| `pico_link` | boot id, epochs, reboots and restarts seen, firmware; null without a Pico link |
| `software` | program, capture module build time, compiler, contract versions |
| `host_sampled_pi_host_us` | when `configuration`, `profile`, `localization`, `pico_link` and `software` above were read: when the bundle was built |
| `host_at_start` | the same keys read at or before the window start, with their own `sampled_pi_host_us`; null without a host |
| `trigger` | reason, requester, detail, pre/post requested, clamped and actual, window start, trigger and end times, `truncated`, `truncated_by`, `stop_reason` |
| `streams` | per stream: selected, file, rows, first/last `pi_host_us` |
| `drops` | per stream: `hub_ring_full`, `capture_limit`, `not_selected`, `bundle_size_limit`, `malformed` (payload not matching its stream, skipped), `hub_posted`; `rolling_evicted` (records stamped inside this window that the record bound pushed out of the rolling window before the trigger, counted per millisecond of their stamp), `fault_notices_dropped`, `hub_capacity`, `record_cap`, and a note on each |
| `segments` | one entry per continuous segment of each domain: `robot` (reset_count, odometry_epoch, anchor_revision), `pico` (identity, boot_id, acq_epoch, imu_epoch, per source), `brain` (brain_session, per source), `pi` (reset_count); first/last `pi_host_us` and rows |
| `conventions` | frames, axes, headings, attitude, height, clocks, missing values |
| `markers` | the capture's own timeline notes |

The host keys are read twice.
- At the top level they are read when the bundle is built, after the post
  window (`host_sampled_pi_host_us`).
- `host_at_start` holds the same keys as they were at the window start. While
  a rolling window runs, the recorder reads them once a second and keeps the
  newest reading at or before the window start. Without a rolling window,
  they are read when the capture begins.

So a profile applied, a calibration finished or a Pico reboot during the
capture shows in both, and `events.csv` says when it happened. For example,
a `profile_changed` or `pico_reboot` automatic capture has the old profile or
boot in `host_at_start` and the new one at the top level. Changes between two
readings are not seen there. The rows carry the identifiers in force when each
record was posted.

## Analysis outside the viewer

The bundle is plain CSV. For example, in Python:

```python
import zipfile, io, pandas as pd
z = zipfile.ZipFile("gatr2_capture_<id>.zip")
robot = pd.read_csv(io.BytesIO(z.read("robot_state.csv")))
for seg, rows in robot.groupby("segment"):     # never join across segments
    print(seg, rows["odom_heading_unwrapped_deg"].iloc[-1])
```

Join streams on `pi_host_us`, for example with `pandas.merge_asof`. Do not
resample them onto a grid you then treat as measured.

## What is tested, and what is not

The host tests are in `tests/capture_gtest.cpp`: windows, limits, cancel,
busy refusal, automatic trigger cooldown and cap, segments, drop accounting,
CSV escaping and missing values, a ZIP round trip with an independent
reader, replay consistency, HTTP routes, directory writes, shutdown, and a
synthetic-rig System run. Tests added after review:
- the bundle size limit marks `metadata.json` truncated, as the status does;
- the host metadata at the window start next to the values at the end;
- `rolling_evicted` for this window only;
- encoder deltas only between consecutive frames, with `seq_gap_frames`;
- a System with the Brain link over a memory link: going quiet fires the
  `link_lost` capture with the "Brain link quiet for 1 s" event in
  `events.csv`, and a second quiet edge inside the cooldown is suppressed;
- the shipped `brain_profile_usb.xml` with memory links and a `<Capture>`
  element added to its resolved document: a new Pico `boot_id` fires the
  `pico_reboot` capture, whose `host_at_start` has the old boot and whose top
  level has the new one, and "Pico frames lost" while it records is
  suppressed as busy.

They are host tests only. Nothing here has been run on the robot, the Pi or
the Pico hardware. The `continuity_lost` and `profile_changed` edges in
System are not driven by a test yet; the recorder side of every trigger is.
