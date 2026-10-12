# Pi inventory (explorer report, 2026-09-28, before any change)

Paths are relative to `pi/naviGATR/` unless noted. Verify before relying on a line number.

**A. Inspection feed**
- **Service thread:** `InspectionService::Impl::run` (src/inspection/inspection_service.cpp:521-560) wakes at the next snapshot or preview deadline and does not catch up.
- **Rates:** `snapshot_hz` defaults to 20 (src/runtime/inspection_config.h:40), range (0,200].
- **Snapshot path:** `publishSnapshot` (434-445) skips work with no clients. It builds `snapshotDocument` once and calls `broadcastText` (src/inspection/http_server.cpp:925-936), which calls `sendText` for each client. `/api/snapshot` and `/api/hello` are built on the HTTP thread (350-355).
- **`snapshotDocument` keys** (inspection_document.cpp:985-1421):
  - Fast-changing: `type`, `contract`, `host_ms`, `session`, `cycle`, `running`, `robot`, `localization`.
  - `trail`: up to 300 entries, resent every snapshot.
  - Slow-changing: `field_snapshot`, `field_objects`, `detection_frames`, `target`, `command`, `brain_link`, `pico`, `events`, `sources`, `workers`, `diagnostics`.
- **`helloDocument` keys** (719-983): `type`, `contract`, `host_ms`, `session`, `configuration`, `inspection`, `robot_body`, `fields`, `tag_families`, `cameras`, `camera_sensors`, `localization`, `warnings`. Sent on open (inspection_service.cpp:401).
- **Trail:** `feed->snapshot(300)` (inspection_document.h:40), copied under the feed mutex (src/state/robot_state_feed.cpp:50-53). History is capped at 1024 entries and 5000 ms (src/state/pose_history.h:36-37).
- **Outgoing buffer** (http_server.cpp):
  - Structure: `std::string out` plus an offset (368-369). It is a FIFO: `c.out += bytes` (539).
  - When full, the newest message is refused (844-847), so old snapshots stay queued ahead of newer ones.
  - Partial writes are tracked by offset only (509); the buffer compacts when over 64 KB and more than half (525-527).
  - `queueBytes` calls `flush()` immediately (540), so the socket send runs on the caller's thread under the server mutex (837).
  - Stall close: `close_after_stalled_ms` (812-816), default 10000. `client_buffer_kb` defaults to 1024 (inspection_config.h:45-46).
- **JPEG preview:**
  - Frames come from `system->detectionFrames()` (inspection_service.cpp:470), published by the field worker `fieldCycle` → `publishDetectionFrames` (src/runtime/system.cpp:1084, 1153-1258).
  - Encoding runs on the inspection thread, cached for 16 entries (39, 254-317).
  - Rate: default 5 Hz; a client can set 0..30 (416-424).
  - Binary message: `[u32 LE len][JSON header][JPEG]` (285-295).
- **Stats:** `InspectionServiceStats` (src/inspection/inspection_stats.h:12-30). The server's `messages_refused`, `clients_closed_stalled` and `refused_connections` are not merged into it (inspection_service.cpp:238-242).
- **Clocks:**
  - `HostClock::now()` is `steady_clock` milliseconds since process start.
  - `RobotState.measuredAt` is the source clock; `measuredAtHost` is mapped.
  - The document `host_ms` is its build time.
  - The feed `publication` counter exists, but there is no publish-time stamp.

**B. Recorder taps** (E = estimation worker, F = field worker)
- **Robot state:** `feed_->publish` (src/runtime/localization_stage.cpp:128); also `loseContinuity` 149, `reset` 171, `continueFrom` 187. `RobotStateFeed::publish` (robot_state_feed.cpp:8-27). History is appended when `advanced && valid && measuredAtHost` (17). (E)
- **Resource and sensor records:** `storePoll` (src/runtime/record_bookkeeping.h:22-46), called at resource_stage.cpp:57 and sensor_stage.cpp:25. (E)
- **Pico:** `PicoTelemetry::refresh` (src/impl/resources/pico_telemetry.cpp:131-189): read 144, decode 167, `applyPacket` 191-283, `applyStatus` 285-312.
  - Outputs publish only on change (480-537), so encoders are coalesced per drained batch; gyro is integrated per packet (261-279).
- **VEX IMU:** GET_STATE → `bench_imu_->accept` (src/impl/commands/brain_link_commands.cpp:344-349); `BrainImuBench::accept` (src/resources/brain_imu_bench.h:22-36); consumed at src/impl/localization/brain_imu_wheel_bench.cpp:229-346. (E)
- **Brain link:**
  - Requests: `BrainLinkCommands::run` (203-293; decode 248; `process` 295-338); PATH_REPORT 370-384; CONTROL 359-369; SET_POSE 395.
  - Replies: `BrainLinkPublisher::run` (src/impl/publishing/brain_link_publisher.cpp:323-424; encode 400; write 405).
- **Events:**
  - `RuntimeEvent` (src/runtime/system.h:177-181).
  - `noteEvent` (system.cpp:935-941) keeps a ring of 32 (`kEventsKept` at 67).
  - Producers: `losePlacement` 699-714, `watchSensors` 669-688, `watchModels` 690-697, `noteRecovery` 716-789, profile 403-453 and 802-857, controls 457-534.
  - main.cpp polls every 20 ms (src/app/main.cpp:136-143, 162-168).
- **Field:** `fieldCycle` (system.cpp:1062-1096). (F)
- **Reporting:** `reportingCycle` 1022-1060; `publishSourceHealth` 1098-1151. (E)

**C. Serial transports**
- **Interface:** src/resources/serial_link.h:58-81.
- **LinuxSerialLink:** read serial_links.cpp:33-43 → `SerialPort::read` src/transport/serial_port.cpp:132-141. Write 109-126 → `SerialPort::write` 143-150, or `transmitHalfDuplex` (src/transport/half_duplex.cpp:86-).
- **ReopeningLink:** serial_links.cpp:45-103. Its counters (serial_links.h:97-101) are never read.
- **MemoryLink** 128-148; **FileReplayLink** 150-156.
- **ProsUsbLink:** read pros_usb_link.cpp:50-71, write 77-121 (5 ms bound at 88). The NG1 line decode (208-260) drops bad lines without counting them, and there are no counters.
- **FrameReader** (translaGATR/frame_codec.h:125-154, scan frame_codec.cpp:840-863): bad sync, length and CRC are all `drop(1)` with no counter.
- **Link stats:** `LinkStats.decode_errors` (src/core/diagnostics.h:23-41) counts only frames that pass the CRC but fail typed decode.
  - bytes, packets and seq_gaps: pico_telemetry.cpp:154-155, 208-212; brain_link_commands.cpp:233-243.
  - Reply counters: brain_link_publisher.cpp:406-418.
  - Shown in `diagnostics.links` (inspection_document.cpp:558-579).
- **No byte tap or hex dump exists.** There are no UART error counters on either side.

**D. Pico firmware** (pico/aggreGATR/src)
- **USB console** (`logUsb`, main.cpp:165-215, every 200 ms):
  - A counts line.
  - Once per second:
    - BNO08X diagnostics (imu_bno08x.cpp:406-420): state, failure, attempts, epoch, int/rst/wake levels, boots, retries, rx, bad headers, last header, ack mask, report counts, alignment samples, report age, sh2 error; counters declared at 75-80.
    - ASM330 diagnostics (imu_asm330.cpp:212-219).
    - A link line: boot, epochs, state, command counters.
- **Not tracked:** UART errors and bad frames from the Pi.
- **Pins** (board.h:17-33):
  - Encoders: GP0/1, GP2/3, GP4/5.
  - IMU on SPI1: INT GP22, MISO GP8, MOSI GP11, SCK GP10, CS GP9; BNO08X RST GP12 and WAKE GP13.
  - UART0 to the Pi: TX GP16, RX GP17.
- **Frame sizes:** sensor v2 is 31 bytes, asserted at most 32 (TX FIFO) (main.cpp:20-25). Status is 26 bytes. Sensor frames run at 50 Hz, 115200 baud. A tick is skipped when the FIFO is busy (251-253).
- **Status scheduling:** `StatusSchedule::ready` (pi_link.cpp:24-36) needs an empty FIFO plus airtime and 1 ms before the next tick; every 200 ms, or on request.

**E. Attitude**
- `Attitude` at src/state/attitude.h:24-41. The only producer is the synthetic rig (synthetic_rig.cpp:31-34, 310-328; docs/attitude_firmware_followup.md).
- **Chain:** `attitude_channel` → `attitude_reference` → `AttitudeFold::apply` (src/impl/localization/motion_step.cpp:125-186) → `RobotState.attitude` → feed plus `appendAttitude`.
- **Inspection:** `robot.attitude` (inspection_document.cpp:132-157, 1039); trail roll/pitch; `attitude_at_exposure` (619-623).
- **Sources today:**
  - VEX sends rotation only (translaGATR/frames.h: `imu_flags`, `imu_stamp_ms`, `imu_rotation_mdeg`).
  - The Pico sends `gyro_z` only; the accel bit is reserved and never emitted.

**F. Camera**
- **Configs with a camera:** gatr2_front_camera.xml.in, gatr2_front_camera_uncalibrated.xml, synthetic_rig*.xml. Pipelines: camera_inspection_only.xml, parallel_wheels_bno08x_camera.xml, three_wheel_imu_camera_diagnostic.xml.
- **Resources:**
  - `libcamera_camera` (needs `NAVIGATR_WITH_LIBCAMERA`; runs its own capture thread).
  - `cameraResource` (src/impl/resources/cameras.cpp:180-230).
  - `idle_camera` exists only as a test fixture (tests/end_to_end_gtest.cpp:170-195).
- **Sensor:** `camera_frame` (src/impl/sensors/camera_channel.cpp:44-65).
- **What preview needs:** `fieldCycle` has to run. With the noop world estimator, raw frames are still published with `has_observations=false` (system.cpp:1222-1250; tested at tests/inspection_document_gtest.cpp:467). No shipped config does this. The closest is config/override/diagnostics/live_camera_inspection.xml plus camera_inspection_only.xml, which runs the AprilTag detector.

**G. Tests and tools**
- **tests/inspection_gtest.cpp:** routes 663, WebSocket feed 738, preview budget 853, slow client 910, server bounds 955, reconnect 1045, stop 1090, URL 1131, config 1154, static root 1188.
- **tests/inspection_document_gtest.cpp:** 303, 336, 367, 404, 467.
- **tools/viewer_smoke.sh:** headless Chrome/Edge with `--dump-dom` and a screenshot; it checks the `#status` data attributes. There is no CDP or puppeteer script.

**H. Threads and locks**
- **Threads:**
  - main: profiles and events.
  - Estimation worker (system.cpp:1362-1385): all serial I/O, sensors, commands, localization, reporting and publishing, at the loop rate (default 100 Hz).
  - Field worker (1387-1399).
  - Inspection thread.
  - HTTP server thread (20 ms `select`).
  - libcamera threads.
- **Locks:**
  - `System::snapshot_mutex_`;
  - `RobotStateFeed::mutex_` (the trail copy contends with `publish`);
  - `PicoTelemetry::state_mutex_`;
  - inspection `mutex`/`cache_mutex`/`wake_mutex`;
  - the `HttpServer` mutex (held during socket sends);
  - `lifecycle_mutex_`, `profile_mutex_`, `wake_mutex_`.

## Lead additions before stage 1 (2026-09-28)
- `src/diagnostics/{records.h, hub.h, hub.cpp, link_monitor.h, link_monitor.cpp, instrumentation.h, instrumentation.cpp}`.
- `src/capture/{capture_service.h, capture_service.cpp}` (stub).
- `System::diagHub()` and `System::capture()`.
- The hub is passed to resources and slots through `BuildOptions::diagnostics`, `ResourceInitializationContext::diagnostics` and `SlotInitializationContext::diagnostics`.
- `HostClock::nowUs()`.
- `translaGATR/frames.h`: `BrainTelemetry`, `kOpTelemetry`, `PicoDiag`, `kFramePicoDiag`, `kPicoOpDiagnostics`, `PicoCommand::diag_hz`.
- `translaGATR/frame_codec.h`: `FrameReaderStats` inside `FrameReader` (not yet counted).
