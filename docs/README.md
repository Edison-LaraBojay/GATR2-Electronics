# Documentation

The repository is being reconsidered around a VEX sensing subsystem: robot
localization plus a temporary estimate of one selected landmark or scoring face.
The Brain owns the behavior that uses those measurements. Existing code and
architecture documents are material to evaluate, not requirements to preserve.

## Start here for the redesign

1. [Restructure context](restructure-context.md) records the problem, the user's
   reasoning and decisions, alternatives discussed, open questions, repository
   maturity, and a source map. Read this first to understand why the design is
   changing and distinguish user direction from recommendations.
2. [Reporting proposal](reporting-proposal.md) develops the proposed report
   semantics, coordinate frames, selection and propagation behavior, geometry
   and association, camera-processing options, implementation sequence, and
   validation scenarios. Use it to scope implementation. It is a working proposal,
   not a finalized API, wire format, or claim of implemented behavior.

The current direction is one active selected estimate, supported by a static
seasonal geometry catalog where useful. Multiple tag observations may support
that estimate. There is no requirement to propagate every object in the field.
The detailed documents identify which remaining choices affect implementation;
their proposed defaults must not be presented as decisions the user already made.

## Existing implementation references

Read these selectively after the two documents above. Their descriptions of
the current implementation can help locate reusable mechanisms and integration
constraints. Their older target, pipeline, and output contracts do not supersede
the redesign context.

| Reference | Use it for | Qualification |
|---|---|---|
| [Navigatr README](../pi/navigatr/README.md) | Runtime overview, configuration inventory, build and replay commands. | Describes the existing fixed pipeline and Pi-owned navigation targets, not the proposed landmark interface. |
| [Navigatr architecture](navigatr.md) | Current transforms, timing, pose history, lifecycle, and association mechanics. | XML, mandatory pipeline stages, `acquire_once`, and target resolution are existing design choices to review. |
| [Resources](navigatr_resources.md) | Shared links/decoders, static geometry, camera calibration, and current resource schemas. | `target_set` is legacy behavior; existing approach frames point outward and must not silently acquire a different convention. |
| [Sensors](navigatr_sensors.md) | Measurement payloads, timestamps, freshness, calibration ownership, and shared-source behavior. | The catalog describes today's registered implementations, not the only sensors or runtime structure allowed. |
| [Interfaces](interfaces.md) | Existing framing and device interoperability; compare with [frames.h](../common/frames.h) and the [codec](../common/frame_codec.cpp). | Current absolute-object fields, bearing entries, and status bits do not define the proposed report. Binary redesign is deferred. |
| [Hardware](hardware.md) | Component rationale, acquisition settings, and wiring leads. | Verify the relevant schematic and firmware revision. Historical drift, vision-correction, and performance claims are not validation evidence. |
| [Pi setup](pi_setup.md) | Access, provisioning, and UART bring-up. | This is not a complete camera or production deployment guide; some setup details remain placeholders. |
| [RS-485 bench test](../bench/rs485_link/README.md) | Isolated Pi-to-Brain byte-transfer bring-up. | It does not prove command return traffic, the production protocol, or estimator accuracy. |

The [Pico README](../pico/README.md), [Pi README](../pi/README.md),
[Brain README](../brain/README.md), and [PCB README](../pcb/README.md) provide
directory orientation. In particular, the Brain README describes an intended
library; the directory currently contains no implementation. Real camera capture
and the AprilTag adapter also remain integration work, so synthetic vision tests
are not evidence of camera performance or physical alignment accuracy.

## Build and validation references

The checked-in [CI workflow](../.github/workflows/ci.yaml) identifies the current
host tests and Pico firmware build. Supporting entry points are
[Navigatr CMake](../pi/navigatr/CMakeLists.txt),
[common tests](../common/tests/CMakeLists.txt), and
[Pico PlatformIO configuration](../pico/platformio.ini).

Use existing tests to understand and verify mechanisms that are retained. Tests
encoding superseded target behavior should change with the behavior. Hardware
validation of observation, turnaround, propagation, reacquisition, and scoring
precision is described in the reporting proposal; a passing host build cannot
establish those physical properties.
