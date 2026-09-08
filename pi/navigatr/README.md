# navigatr

`navigatr` is the Raspberry Pi sensing runtime for the GATR2 VEX robot. It
combines measurements into a robot pose and an estimate of one requested
physical landmark or scoring face. The Brain owns destinations, desired contact
geometry, alignment control, and motor commands.

Robot position and reported landmark position use the configured field
coordinates. Robot heading is its orientation in that field. The landmark's
reported `heading_error` is its estimated rotation away from its nominal field
orientation, modulo any declared object symmetry. Internal transforms use a
consistent full orientation representative and associated face/mount geometry.

## Runtime design

These documents define runtime requirements and data meaning. They are not
evidence of completed hardware integration.

| Document | Responsibility |
|---|---|
| [Architecture](docs/architecture.md) | SensorMap, nested pipeline stages and their I/O, interchangeable implementations, independent workers, and pose history. |
| [Coordinates](docs/coordinates.md) | Field and robot axes, heading error, square symmetry, side selection, camera mounting, and measurement-time transforms. |
| [Landmarks](docs/landmarks.md) | One requested report, static field definitions, measured-object caching, association, and processing scope. |

One executable hosts independently scheduled localization and landmark-estimation
pipelines. Each pipeline runs its own stages in order and publishes complete
snapshots. Neither assumes the other's sensor inventory or execution rate.
Localization publishes recent motion history as well as its latest state, so
landmark estimation can interpret a delayed observation at measurement time.

`SensorMap` owns configured sensor producers. `SensorResultsMap` provides their
standardized results. Consumers bind to declared payload types and source IDs.
Shared devices and immutable calibration belong to resources. Each stage may
contain a private pipeline behind its declared input/output contract.

The field definition supplies nominal geometry, reference identities, and the
coordinate convention. Measured estimates remain separate from that definition.
The recommended design caches accepted object estimates, including incidental
observations within the processing budget, and reports only the requested
reference. See the [retention policy](docs/landmarks.md#retention-policy).

## Implementation coverage

The C++ implementation includes typed sensor records and bindings, shared
resources, wheel/IMU motion estimation, transforms, pose history, association,
configuration validation, replay, and host tests.

[`System::step`](src/runtime/system.cpp) executes the stages synchronously.
Independent workers and the time-window ring-buffer lookup described in the
architecture are implementation work. The existing history is a bounded deque
with a linear lookup. The current field estimator retains a map of objects, and
the publisher uses configured target semantics; the landmark report defined here
is not implemented by those interfaces. Grouping equivalent symmetric poses and
the measured-cache lifecycle specified here also require implementation.

The proposed PreparedMeasurementMap, CandidateSet, LandmarkHypothesisSet,
LandmarkEvidenceMap, and LandmarkStateMap contracts describe the implementation
boundaries to build. The current FieldMap parser loads landmark poses and mount
geometry; field dimensions and the axis convention appear in the field XML comment
rather than structured parser fields. A structured FieldDefinition with dimensions,
reference conventions, and declared symmetries also needs implementation.

Real camera capture and the AprilTag backend also need integration:
[`cameras.cpp`](src/impl/resources/cameras.cpp) constructs a configured device
without live capture, and [`tag_detectors.cpp`](src/impl/resources/tag_detectors.cpp)
rejects construction of the missing backend. Synthetic tests establish software
behavior, not camera latency or physical alignment accuracy.

## Configuration and calibration

XML selects registered implementations and supplies their configuration. IDs are
opaque references. Startup checks reject missing producers, incompatible payloads,
duplicate outputs, and invalid calibration.

- [`config/shared/robots/`](config/shared/robots/): robot geometry, sensor channels,
  and a separate camera calibration fragment.
- [`config/shared/pipelines/`](config/shared/pipelines/): two- and three-wheel
  diagnostic processing configurations.
- [`config/override/field.xml`](config/override/field.xml): nominal seasonal geometry
  for nine goals and their physical tag mounts.
- [`config/override/diagnostics/`](config/override/diagnostics/): composed diagnostic
  profile templates.

Files ending in `.xml.in` contain unmeasured or unresolved values. Complete their
placeholders with measured configuration before using them as profiles.
`calibration_status="UNCONFIGURED"` fails startup; `provisional` requires
`--allow-provisional`. Camera intrinsics describe the actual optics and image
mode. Camera mounting and robot contact geometry are separate measurements.

## Build and bring-up

Run from `pi/navigatr`:

```text
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Runtime arguments:

```text
./build/navigatr <complete-config.xml> [--cycles <n>]
./build/navigatr <complete-config.xml> --replay <resource_id>=<capture.bin>
./build/navigatr <complete-config.xml> --allow-provisional
```

Replay uses the acquisition decoder and estimation implementations used by live
input. Validate acquisition timestamps, publication latency, and alignment error
on the robot before assigning operating limits.
