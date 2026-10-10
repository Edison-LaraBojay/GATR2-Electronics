# Receiver pipeline

One source thread, one server thread, one main loop. Every source hands on raw bytes; one path
turns them into poses.

```text
source (controller | brain | sim | file) -> SourceChunk bytes + SourceStatus
  -> PoseFeed: LineScanner -> decodeLine -> PoseTracker -> TrackedPose
  -> LiveFeed: pose JSON to every browser (latest-wins), status on change or 250 ms
  -> HttpServer: /ws frames, static web/ and fields/
```

## Input
- controller CDC2 replies, Brain user port bytes, sim lines, file or stdin bytes
- options (`app/command_line.h`)

## Output
- browser feed of PROTOCOL.md: hello, pose, status; `/`, `/fields/<name>.json`, `/api/fields`

## Folders
| Folder | Holds |
|---|---|
| `platform/` | the only OS code: serial ports, USB listing, VEX roles, browser launch, Ctrl+C, stdin |
| `sources/` | serial loop (`serial_source`), Brain user port session, controller session and CDC2, port pick, replay; `fakes/` for tests |
| `sim/` | deterministic trajectories (pure) and the sim source |
| `feed/` | tracker rules, bytes to poses, status, JSON (pure, protocol only) |
| `server/` | HTTP and WebSocket server, static site |
| `app/` | options, asset folders, the live hub, main |
| `tests/` | host tests |

## Rules
- Read-only toward the robot: the controller gets only SYSTEM_VERSION, USER_READ, RADIO_STATUS.
- Same session and epoch: only a newer seq (int32 difference) is accepted. New session or
  epoch: accepted with `restart`.
- A change of link state resets the line scanner; a replay rewind resets ordering.
- Receiver clock only for ages; the publisher's t_ms is never compared with it.

## Threads
- source thread: reads, calls LiveFeed (mutex); the stdin replay may block in a read
- server thread: select loop, handlers outside its lock
- main loop: LiveFeed::tick every 50 ms, Ctrl+C flag
