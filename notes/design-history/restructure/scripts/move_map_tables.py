# Tables for plan/move_map.md: one table per new folder, with includers.
#
#   py -3 move_map_tables.py <include_graph.json> <out.md>

import json
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(__file__))
import move_map as mm  # noqa: E402

R = {
    # cli / main / tools
    "src/app/build_config.h.in": "configure_file template for the default config path; only main.cpp reads it (generated navigatr_build_config.h)",
    "src/app/command_line.cpp": "command-line parsing (SPEC cli/)",
    "src/app/command_line.h": "command-line parsing (SPEC cli/)",
    "src/app/field_refs.cpp": "separate generator program (SPEC: tools/field_refs/)",
    "src/app/main.cpp": "tiny entry (SPEC: src/main.cpp)",
    # core
    "src/runtime/system.cpp": "engine; SPEC maps it to core/system.cpp until the split step",
    "src/runtime/system.h": "engine facade; SPEC maps it to core/system.h until the split step",
    "src/runtime/handoff.h": "LatestSlot/WorkerStats: the run loop's latest-wins handoff between the two workers",
    # pipeline level
    "src/runtime/register_all.cpp": "calls every stage's register_* in run order; plug-in index for the whole pipeline",
    "src/runtime/register_all.h": "declares every register_* plus registerOrDie; included by every plug-in register file",
    # framework
    "src/core/diagnostics.h": "per-slot FunctionStats/LinkStats sink of ExecutionContext; stage machinery",
    "src/core/execution_context.h": "SPEC framework: execution context",
    "src/core/function_registry.h": "SPEC framework: function registry",
    "src/core/function_status.h": "SPEC framework: function status",
    "src/core/ids.h": "SPEC framework: ids",
    "src/core/payload_descriptor.h": "SPEC framework: payload descriptors",
    "src/core/records.h": "SPEC framework: records (ResourceMap/SensorMap envelopes)",
    "src/core/typed_payload.h": "SPEC framework: typed payloads",
    "src/contracts/slot_init.h": "SlotInitializationContext: build context every slot factory receives",
    "src/runtime/build_options.h": "BuildOptions: build-time policy from the CLI, consumed by make_resources; keeps stages from including core/",
    "src/runtime/record_bookkeeping.h": "poll-to-record rules shared by the resource and sensor stages",
    # time
    "src/core/clock_sync.h": "SPEC time/: clock sync",
    "src/core/host_clock.h": "SPEC time/: host clock",
    "src/core/time.h": "SPEC time/: time types",
    # io
    "src/transport/byte_stream.h": "SPEC io/ (from transport/)",
    "src/transport/file_stream.cpp": "SPEC io/ (from transport/)",
    "src/transport/file_stream.h": "SPEC io/ (from transport/)",
    "src/transport/half_duplex.cpp": "SPEC io/ (from transport/)",
    "src/transport/half_duplex.h": "SPEC io/ (from transport/)",
    "src/transport/serial_port.cpp": "SPEC io/ (from transport/)",
    "src/transport/serial_port.h": "SPEC io/ (from transport/)",
    "src/resources/serial_link.h": "byte-link contract (ByteSpan, TransmitWindow); io/half_duplex.h and commands/command_state.h need it",
    "src/resources/spi_bus.h": "SPI bus contract beside serial_link.h (includes it); only tests/resources_gtest.cpp uses it",
    # resources stage
    "src/contracts/resource.h": "resources stage contract (SPEC contracts/<x>.h -> contract.h)",
    "src/runtime/resource_stage.cpp": "make_resources builder (SPEC stage.cpp)",
    "src/runtime/resource_stage.h": "make_resources builder (SPEC stage.h)",
    "src/runtime/resource_catalog.h": "catalog of resource outputs the stage owns",
    "src/resources/resource_store.cpp": "store the stage owns",
    "src/resources/resource_store.h": "store the stage owns",
    "src/resources/resource_instance.h": "type-erased resource holder the store owns",
    "src/resources/brain_imu_bench.h": "resource type brain_imu_bench (registered inline in register_resources.cpp); produced by the resources stage",
    "src/resources/camera.h": "Camera resource contract; produced by camera resources",
    "src/resources/pico_control.h": "PicoControl resource contract; produced by pico_telemetry",
    "src/resources/robot_frames.h": "RobotFrameMap resource data; produced by robot_frame_map",
    "src/resources/tag_detector.h": "TagDetector resource contract; produced by apriltag_detector",
    "src/resources/target_set.h": "TargetSet resource data; produced by target_set",
    "src/resources/wheel_geometry.h": "WheelGeometryMap resource data; produced by wheel_geometry",
    "src/runtime/pico_control_ref.h": "resolves <Pico resource_id> to a PicoControl from the store; resource-reference helper",
    "src/payloads/pico_telemetry_samples.h": "pico.* payloads the pico_telemetry resource produces",
    # sensors stage
    "src/contracts/sensor.h": "sensors stage contract",
    "src/runtime/sensor_stage.cpp": "make_sensors builder",
    "src/runtime/sensor_stage.h": "make_sensors builder",
    "src/runtime/sensor_catalog.h": "SensorCatalog the stage owns (bind-by-id for consumers)",
    "src/payloads/sensor_samples.h": "sensor.encoder_sample / sensor.imu_sample: SensorMap output payloads",
    "src/payloads/attitude_samples.h": "sensor.attitude_sample: SensorMap output payload (attitude_channel produces the body-frame sample)",
    "src/payloads/camera_frames.h": "sensor.camera_frame: SensorMap output payload perception binds by sensor id",
    # commands stage
    "src/contracts/commands.h": "commands stage contract",
    "src/state/command_state.h": "CommandState, produced by the commands stage",
    # localization stage
    "src/contracts/localization.h": "localization stage contract",
    "src/runtime/localization_stage.cpp": "make_localization builder",
    "src/runtime/localization_stage.h": "make_localization builder",
    "src/state/attitude.h": "Attitude, part of the robot estimate localization produces",
    "src/state/pose_history.cpp": "pose history localization owns and publishes",
    "src/state/pose_history.h": "pose history localization owns and publishes",
    "src/state/robot_state.h": "RobotState, localization's output",
    "src/state/robot_state_feed.cpp": "publication side of localization",
    "src/state/robot_state_feed.h": "publication side of localization",
    "src/state/stillness.h": "StillnessStatus, part of the localization contract",
    "src/payloads/robot_observations.h": "localization.* payloads its observation functions produce",
    "src/impl/localization/register_localization.cpp": "registers both <Observation> and <Estimator> plug-ins, so it sits above both folders",
    # world estimation stage
    "src/contracts/field_estimation.h": "world estimation stage contract (SPEC)",
    "src/contracts/association.h": "generic step contract (SPEC)",
    "src/contracts/perception.h": "generic step contract (SPEC)",
    "src/payloads/field_object_evidence.h": "SPEC evidence.h; tag trace types leave it in step 3",
    "src/state/field_state.h": "field state (SPEC)",
    "src/impl/world_estimation/landmark_estimator.cpp": "generic evidence folding, sensor-agnostic (SPEC stage level)",
    "src/impl/world_estimation/landmark_estimator.h": "generic evidence folding, sensor-agnostic (SPEC stage level)",
    "src/runtime/world_estimation_stage.cpp": "make_world_estimation builder",
    "src/runtime/world_estimation_stage.h": "make_world_estimation builder",
    "src/impl/world_estimation/register_world_estimation.cpp": "registers apriltag and noop estimators, so it sits above both group folders",
    "src/impl/world_estimation/apriltag_tag_observation.cpp": "apriltag group: tag observation extraction (SPEC)",
    "src/impl/world_estimation/apriltag_tag_observation.h": "apriltag group: tag observation extraction (SPEC)",
    "src/impl/world_estimation/apriltag_world_estimator.cpp": "apriltag group: the world estimator (SPEC)",
    "src/impl/world_estimation/apriltag_world_estimator.h": "apriltag group: the world estimator (SPEC)",
    "src/impl/world_estimation/tag_mount_association.cpp": "apriltag group: tag mount association (SPEC)",
    "src/impl/world_estimation/tag_mount_association.h": "apriltag group: tag mount association (SPEC)",
    "src/payloads/tag_observations.h": "apriltag group: tag observation payloads (SPEC)",
    "src/impl/noop/noops.cpp": "git mv target; split into five stage noops (see Noop split)",
    "src/impl/noop/noops.h": "git mv target; split into five stage noops (see Noop split)",
    # targets stage
    "src/contracts/target_resolution.h": "targets stage contract (no builder: slot built by System::build)",
    "src/state/target_state.h": "TargetState, produced by target resolution",
    # publishing stage
    "src/contracts/publishing.h": "publishing stage contract (no builder: slot built by System::build)",
    "src/impl/publishing/field_documents.cpp": "Brain wire field documents the brain_link publisher sends; also used by tools/field_refs and inspection",
    "src/impl/publishing/field_documents.h": "Brain wire field documents the brain_link publisher sends; also used by tools/field_refs and inspection",
    # brain_profile
    "src/contracts/brain_profile.h": "BrainProfileHost/ProfileBinding: the profile host interface (SPEC brain_profile/)",
    "src/runtime/brain_profile_builder.cpp": "profile build (SPEC brain_profile/)",
    "src/runtime/brain_profile_builder.h": "profile build (SPEC brain_profile/)",
    "src/runtime/sensor_loss.cpp": "sensor loss and recovery (SPEC brain_profile/)",
    "src/runtime/sensor_loss.h": "sensor loss and recovery (SPEC brain_profile/)",
    "src/runtime/stationary_precheck.cpp": "stationary precheck (SPEC brain_profile/)",
    "src/runtime/stationary_precheck.h": "stationary precheck (SPEC brain_profile/)",
    # debug
    "src/runtime/inspection_config.h": "parses the <Inspection> root element; inspection policy",
    "src/runtime/inspection_state.h": "snapshots the workers hand to inspection; inspection's data contract",
}

PLUGIN_REASON = {
    "resources/resources/": "<Resource type> plug-in (resource_stage.cpp:99)",
    "sensors/sensors/": "<Sensor type> plug-in (sensor_stage.cpp:96)",
    "commands/collectors/": "<CommandCollection type> plug-in (system.cpp:276)",
    "localization/observations/": "<Observation type> plug-in (localization_stage.cpp:223)",
    "localization/estimators/": "<Estimator type> plug-in under <Localization> (localization_stage.cpp:268)",
    "targets/resolvers/": "<TargetResolution type> plug-in (system.cpp:329)",
    "publishing/publishers/": "<Publishing type> plug-in (system.cpp:335)",
}
SPECIAL = {
    "src/impl/localization/stationary_window.cpp": "stillness/bias helper shared by the observation models only",
    "src/impl/localization/stationary_window.h": "stillness/bias helper shared by the observation models only",
    "src/impl/localization/motion_step.cpp": "step core shared by the two planar estimators only",
    "src/impl/localization/motion_step.h": "step core shared by the two planar estimators only",
    "src/impl/resources/apriltag_c.c": "C adapter to vendored apriltag, used by tag_detectors.cpp",
    "src/impl/resources/apriltag_c.h": "C adapter to vendored apriltag, used by tag_detectors.cpp",
    "src/impl/resources/camera_capture.h": "capture timing helpers for libcamera_camera.cpp",
}


def reason(o):
    if o in R:
        return R[o]
    if o in SPECIAL:
        return SPECIAL[o]
    n = mm.new_of(o)
    for k, v in PLUGIN_REASON.items():
        if ("pipeline/" + k) in n:
            if os.path.basename(n).startswith("register_"):
                return "registers the " + v.split(" plug-in")[0] + " plug-ins"
            return v
    if n.startswith("src/debug/"):
        return "SPEC debug/%s/ moved as a subfolder" % n.split("/")[2]
    if mm.MAP[o] == "same":
        return "unchanged (SPEC)"
    return "?"


def includers(g, o):
    key = o[len("src/"):]
    who = sorted(set(w.replace("pi/naviGATR/", "") for w in g["includers"].get(key, [])))
    if not who:
        return "0"
    s = sum(1 for w in who if w.startswith("src/"))
    t = sum(1 for w in who if w.startswith("tests/"))
    x = len(who) - s - t
    parts = []
    if s:
        parts.append("src %d" % s)
    if t:
        parts.append("tests %d" % t)
    if x:
        parts.append("tools %d" % x)
    head = "%d (%s)" % (len(who), ", ".join(parts))
    if len(who) <= 3:
        head += ": " + ", ".join(os.path.basename(w) for w in who)
    return head


def main():
    g = json.load(open(sys.argv[1]))
    by = defaultdict(list)
    for o in mm.MAP:
        n = mm.new_of(o)
        by[os.path.dirname(n)].append(o)
    order = ["src", "src/cli", "tools/field_refs", "src/core", "src/pipeline", "src/pipeline/framework"]
    stages = ["resources", "sensors", "commands", "localization", "world_estimation", "targets", "publishing"]
    for st in stages:
        base = "src/pipeline/" + st
        order.append(base)
        for d in sorted(k for k in by if k.startswith(base + "/")):
            order.append(d)
    order += ["src/brain_profile", "src/config", "src/io", "src/time", "src/math",
              "src/debug/inspection", "src/debug/capture", "src/debug/diagnostics"]
    missing = [k for k in by if k not in order]
    assert not missing, missing
    out = []
    for d in order:
        if d not in by:
            continue
        out.append("### %s/ (%d)" % (d, len(by[d])))
        out.append("")
        out.append("| old (src/...) | new file | included by | reason |")
        out.append("|---|---|---|---|")
        for o in sorted(by[d], key=lambda x: os.path.basename(mm.new_of(x))):
            n = mm.new_of(o)
            newcell = "same" if mm.MAP[o] == "same" else os.path.basename(n)
            out.append("| %s | %s | %s | %s |" % (o[len("src/"):], newcell, includers(g, o), reason(o)))
        out.append("")
    unknown = [o for o in mm.MAP if reason(o) == "?"]
    assert not unknown, unknown
    open(sys.argv[2], "w", newline="\n").write("\n".join(out) + "\n")
    print("tables:", len([d for d in order if d in by]), "rows:", len(mm.MAP))


if __name__ == "__main__":
    main()
