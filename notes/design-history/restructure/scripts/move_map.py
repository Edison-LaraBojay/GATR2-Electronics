# naviGATR src move map: source of truth for plan/move_map.tsv, plus checks
# and the reference report (includes, CMake, docs, comments).
#
#   py -3 move_map.py            writes plan/move_map.tsv and logs/move_map_refs.txt
#
# Paths are relative to pi/naviGATR/.

import os
import re
import sys
from collections import defaultdict

WT = r"C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/wt-navi"
NAV = os.path.join(WT, "pi", "naviGATR")
RS = r"C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/restructure"

P = "src/pipeline/"
RES = P + "resources/"
SEN = P + "sensors/"
CMD = P + "commands/"
LOC = P + "localization/"
WE = P + "world_estimation/"
TGT = P + "targets/"
PUB = P + "publishing/"
FW = P + "framework/"

MAP = {
    # app/
    "src/app/build_config.h.in": "src/cli/build_config.h.in",
    "src/app/command_line.cpp": "src/cli/command_line.cpp",
    "src/app/command_line.h": "src/cli/command_line.h",
    "src/app/field_refs.cpp": "tools/field_refs/field_refs.cpp",
    "src/app/main.cpp": "src/main.cpp",
    # capture/
    "src/capture/capture_bundle.cpp": "src/debug/capture/capture_bundle.cpp",
    "src/capture/capture_bundle.h": "src/debug/capture/capture_bundle.h",
    "src/capture/capture_config.h": "src/debug/capture/capture_config.h",
    "src/capture/capture_http.cpp": "src/debug/capture/capture_http.cpp",
    "src/capture/capture_http.h": "src/debug/capture/capture_http.h",
    "src/capture/capture_recorder.h": "src/debug/capture/capture_recorder.h",
    "src/capture/capture_service.cpp": "src/debug/capture/capture_service.cpp",
    "src/capture/capture_service.h": "src/debug/capture/capture_service.h",
    "src/capture/zip_writer.cpp": "src/debug/capture/zip_writer.cpp",
    "src/capture/zip_writer.h": "src/debug/capture/zip_writer.h",
    # config/ unchanged
    "src/config/composition.cpp": "same",
    "src/config/composition.h": "same",
    "src/config/config_node.cpp": "same",
    "src/config/config_node.h": "same",
    "src/config/field_map.cpp": "same",
    "src/config/field_map.h": "same",
    "src/config/pose3_config.h": "same",
    # contracts/
    "src/contracts/association.h": WE + "association.h",
    "src/contracts/brain_profile.h": "src/brain_profile/brain_profile.h",
    "src/contracts/commands.h": CMD + "contract.h",
    "src/contracts/field_estimation.h": WE + "contract.h",
    "src/contracts/localization.h": LOC + "contract.h",
    "src/contracts/perception.h": WE + "perception.h",
    "src/contracts/publishing.h": PUB + "contract.h",
    "src/contracts/resource.h": RES + "contract.h",
    "src/contracts/sensor.h": SEN + "contract.h",
    "src/contracts/slot_init.h": FW + "slot_init.h",
    "src/contracts/target_resolution.h": TGT + "contract.h",
    # core/ (old)
    "src/core/clock_sync.h": "src/time/clock_sync.h",
    "src/core/diagnostics.h": FW + "diagnostics.h",
    "src/core/execution_context.h": FW + "execution_context.h",
    "src/core/function_registry.h": FW + "function_registry.h",
    "src/core/function_status.h": FW + "function_status.h",
    "src/core/host_clock.h": "src/time/host_clock.h",
    "src/core/ids.h": FW + "ids.h",
    "src/core/payload_descriptor.h": FW + "payload_descriptor.h",
    "src/core/records.h": FW + "records.h",
    "src/core/time.h": "src/time/time.h",
    "src/core/typed_payload.h": FW + "typed_payload.h",
    # diagnostics/
    "src/diagnostics/hub.cpp": "src/debug/diagnostics/hub.cpp",
    "src/diagnostics/hub.h": "src/debug/diagnostics/hub.h",
    "src/diagnostics/instrumentation.cpp": "src/debug/diagnostics/instrumentation.cpp",
    "src/diagnostics/instrumentation.h": "src/debug/diagnostics/instrumentation.h",
    "src/diagnostics/link_monitor.cpp": "src/debug/diagnostics/link_monitor.cpp",
    "src/diagnostics/link_monitor.h": "src/debug/diagnostics/link_monitor.h",
    "src/diagnostics/records.h": "src/debug/diagnostics/records.h",
    # impl/commands  (<CommandCollection type>)
    "src/impl/commands/brain_link_commands.cpp": CMD + "collectors/brain_link_commands.cpp",
    "src/impl/commands/brain_link_commands.h": CMD + "collectors/brain_link_commands.h",
    "src/impl/commands/register_commands.cpp": CMD + "collectors/register_commands.cpp",
    # impl/localization  (<Observation type>, <Estimator type>)
    "src/impl/localization/attitude_reference.cpp": LOC + "observations/attitude_reference.cpp",
    "src/impl/localization/attitude_reference.h": LOC + "observations/attitude_reference.h",
    "src/impl/localization/brain_imu_wheel_bench.cpp": LOC + "observations/brain_imu_wheel_bench.cpp",
    "src/impl/localization/brain_imu_wheel_bench.h": LOC + "observations/brain_imu_wheel_bench.h",
    "src/impl/localization/imu_heading_increment.cpp": LOC + "observations/imu_heading_increment.cpp",
    "src/impl/localization/imu_heading_increment.h": LOC + "observations/imu_heading_increment.h",
    "src/impl/localization/motion_step.cpp": LOC + "estimators/motion_step.cpp",
    "src/impl/localization/motion_step.h": LOC + "estimators/motion_step.h",
    "src/impl/localization/planar_motion_integrator.cpp": LOC + "estimators/planar_motion_integrator.cpp",
    "src/impl/localization/planar_motion_integrator.h": LOC + "estimators/planar_motion_integrator.h",
    "src/impl/localization/register_localization.cpp": LOC + "register_localization.cpp",
    "src/impl/localization/stationary_window.cpp": LOC + "observations/stationary_window.cpp",
    "src/impl/localization/stationary_window.h": LOC + "observations/stationary_window.h",
    "src/impl/localization/tracking_wheel_motion.cpp": LOC + "observations/tracking_wheel_motion.cpp",
    "src/impl/localization/tracking_wheel_motion.h": LOC + "observations/tracking_wheel_motion.h",
    "src/impl/localization/weighted_planar_fusion.cpp": LOC + "estimators/weighted_planar_fusion.cpp",
    "src/impl/localization/weighted_planar_fusion.h": LOC + "estimators/weighted_planar_fusion.h",
    # impl/noop: git mv target is the world estimation piece; split into 5 (see md)
    "src/impl/noop/noops.cpp": WE + "estimators/noop/noop.cpp",
    "src/impl/noop/noops.h": WE + "estimators/noop/noop.h",
    # impl/publishing  (<Publishing type>)
    "src/impl/publishing/brain_link_publisher.cpp": PUB + "publishers/brain_link_publisher.cpp",
    "src/impl/publishing/brain_link_publisher.h": PUB + "publishers/brain_link_publisher.h",
    "src/impl/publishing/field_documents.cpp": PUB + "publishers/field_documents.cpp",
    "src/impl/publishing/field_documents.h": PUB + "publishers/field_documents.h",
    "src/impl/publishing/register_publishers.cpp": PUB + "publishers/register_publishers.cpp",
    # impl/resources  (<Resource type>)
    "src/impl/resources/apriltag_c.c": RES + "resources/apriltag_c.c",
    "src/impl/resources/apriltag_c.h": RES + "resources/apriltag_c.h",
    "src/impl/resources/camera_capture.h": RES + "resources/camera_capture.h",
    "src/impl/resources/cameras.cpp": RES + "resources/cameras.cpp",
    "src/impl/resources/cameras.h": RES + "resources/cameras.h",
    "src/impl/resources/field_map_resource.cpp": RES + "resources/field_map_resource.cpp",
    "src/impl/resources/field_map_resource.h": RES + "resources/field_map_resource.h",
    "src/impl/resources/libcamera_camera.cpp": RES + "resources/libcamera_camera.cpp",
    "src/impl/resources/libcamera_camera.h": RES + "resources/libcamera_camera.h",
    "src/impl/resources/pico_telemetry.cpp": RES + "resources/pico_telemetry.cpp",
    "src/impl/resources/pico_telemetry.h": RES + "resources/pico_telemetry.h",
    "src/impl/resources/pros_usb_link.cpp": RES + "resources/pros_usb_link.cpp",
    "src/impl/resources/pros_usb_link.h": RES + "resources/pros_usb_link.h",
    "src/impl/resources/register_resources.cpp": RES + "resources/register_resources.cpp",
    "src/impl/resources/robot_frame_map.cpp": RES + "resources/robot_frame_map.cpp",
    "src/impl/resources/robot_frame_map.h": RES + "resources/robot_frame_map.h",
    "src/impl/resources/serial_links.cpp": RES + "resources/serial_links.cpp",
    "src/impl/resources/serial_links.h": RES + "resources/serial_links.h",
    "src/impl/resources/synthetic_rig.cpp": RES + "resources/synthetic_rig.cpp",
    "src/impl/resources/synthetic_rig.h": RES + "resources/synthetic_rig.h",
    "src/impl/resources/tag_detectors.cpp": RES + "resources/tag_detectors.cpp",
    "src/impl/resources/tag_detectors.h": RES + "resources/tag_detectors.h",
    "src/impl/resources/target_set_resource.cpp": RES + "resources/target_set_resource.cpp",
    "src/impl/resources/target_set_resource.h": RES + "resources/target_set_resource.h",
    "src/impl/resources/wheel_geometry_resource.cpp": RES + "resources/wheel_geometry_resource.cpp",
    "src/impl/resources/wheel_geometry_resource.h": RES + "resources/wheel_geometry_resource.h",
    # impl/sensors  (<Sensor type>)
    "src/impl/sensors/attitude_channel.cpp": SEN + "sensors/attitude_channel.cpp",
    "src/impl/sensors/attitude_channel.h": SEN + "sensors/attitude_channel.h",
    "src/impl/sensors/camera_channel.cpp": SEN + "sensors/camera_channel.cpp",
    "src/impl/sensors/camera_channel.h": SEN + "sensors/camera_channel.h",
    "src/impl/sensors/pico_channels.cpp": SEN + "sensors/pico_channels.cpp",
    "src/impl/sensors/pico_channels.h": SEN + "sensors/pico_channels.h",
    "src/impl/sensors/register_sensors.cpp": SEN + "sensors/register_sensors.cpp",
    # impl/target_resolution  (<TargetResolution type>)
    "src/impl/target_resolution/configured_targets.cpp": TGT + "resolvers/configured_targets.cpp",
    "src/impl/target_resolution/configured_targets.h": TGT + "resolvers/configured_targets.h",
    "src/impl/target_resolution/register_target_resolution.cpp": TGT + "resolvers/register_target_resolution.cpp",
    # impl/world_estimation  (<Estimator type> under <WorldEstimation>)
    "src/impl/world_estimation/apriltag_tag_observation.cpp": WE + "estimators/apriltag/apriltag_tag_observation.cpp",
    "src/impl/world_estimation/apriltag_tag_observation.h": WE + "estimators/apriltag/apriltag_tag_observation.h",
    "src/impl/world_estimation/apriltag_world_estimator.cpp": WE + "estimators/apriltag/apriltag_world_estimator.cpp",
    "src/impl/world_estimation/apriltag_world_estimator.h": WE + "estimators/apriltag/apriltag_world_estimator.h",
    "src/impl/world_estimation/landmark_estimator.cpp": WE + "landmark_estimator.cpp",
    "src/impl/world_estimation/landmark_estimator.h": WE + "landmark_estimator.h",
    "src/impl/world_estimation/register_world_estimation.cpp": WE + "estimators/register_world_estimation.cpp",
    "src/impl/world_estimation/tag_mount_association.cpp": WE + "estimators/apriltag/tag_mount_association.cpp",
    "src/impl/world_estimation/tag_mount_association.h": WE + "estimators/apriltag/tag_mount_association.h",
    # inspection/
    "src/inspection/http_server.cpp": "src/debug/inspection/http_server.cpp",
    "src/inspection/http_server.h": "src/debug/inspection/http_server.h",
    "src/inspection/inspection_document.cpp": "src/debug/inspection/inspection_document.cpp",
    "src/inspection/inspection_document.h": "src/debug/inspection/inspection_document.h",
    "src/inspection/inspection_service.cpp": "src/debug/inspection/inspection_service.cpp",
    "src/inspection/inspection_service.h": "src/debug/inspection/inspection_service.h",
    "src/inspection/inspection_stats.h": "src/debug/inspection/inspection_stats.h",
    "src/inspection/jpeg_encoder.cpp": "src/debug/inspection/jpeg_encoder.cpp",
    "src/inspection/jpeg_encoder.h": "src/debug/inspection/jpeg_encoder.h",
    "src/inspection/json_writer.h": "src/debug/inspection/json_writer.h",
    "src/inspection/viewer_assets.h": "src/debug/inspection/viewer_assets.h",
    # math/ unchanged
    "src/math/angles.h": "same",
    "src/math/camera_model.h": "same",
    "src/math/quaternion.h": "same",
    "src/math/se3.h": "same",
    "src/math/transforms.h": "same",
    # payloads/
    "src/payloads/attitude_samples.h": SEN + "attitude_samples.h",
    "src/payloads/camera_frames.h": SEN + "camera_frames.h",
    "src/payloads/field_object_evidence.h": WE + "evidence.h",
    "src/payloads/pico_telemetry_samples.h": RES + "pico_telemetry_samples.h",
    "src/payloads/robot_observations.h": LOC + "robot_observations.h",
    "src/payloads/sensor_samples.h": SEN + "sensor_samples.h",
    "src/payloads/tag_observations.h": WE + "estimators/apriltag/tag_observations.h",
    # resources/ (top level)
    "src/resources/brain_imu_bench.h": RES + "brain_imu_bench.h",
    "src/resources/camera.h": RES + "camera.h",
    "src/resources/pico_control.h": RES + "pico_control.h",
    "src/resources/resource_instance.h": RES + "resource_instance.h",
    "src/resources/resource_store.cpp": RES + "resource_store.cpp",
    "src/resources/resource_store.h": RES + "resource_store.h",
    "src/resources/robot_frames.h": RES + "robot_frames.h",
    "src/resources/serial_link.h": "src/io/serial_link.h",
    "src/resources/spi_bus.h": "src/io/spi_bus.h",
    "src/resources/tag_detector.h": RES + "tag_detector.h",
    "src/resources/target_set.h": RES + "target_set.h",
    "src/resources/wheel_geometry.h": RES + "wheel_geometry.h",
    # runtime/
    "src/runtime/brain_profile_builder.cpp": "src/brain_profile/brain_profile_builder.cpp",
    "src/runtime/brain_profile_builder.h": "src/brain_profile/brain_profile_builder.h",
    "src/runtime/build_options.h": FW + "build_options.h",
    "src/runtime/handoff.h": "src/core/handoff.h",
    "src/runtime/inspection_config.h": "src/debug/inspection/inspection_config.h",
    "src/runtime/inspection_state.h": "src/debug/inspection/inspection_state.h",
    "src/runtime/localization_stage.cpp": LOC + "stage.cpp",
    "src/runtime/localization_stage.h": LOC + "stage.h",
    "src/runtime/pico_control_ref.h": RES + "pico_control_ref.h",
    "src/runtime/record_bookkeeping.h": FW + "record_bookkeeping.h",
    "src/runtime/register_all.cpp": P + "register_all.cpp",
    "src/runtime/register_all.h": P + "register_all.h",
    "src/runtime/resource_catalog.h": RES + "resource_catalog.h",
    "src/runtime/resource_stage.cpp": RES + "stage.cpp",
    "src/runtime/resource_stage.h": RES + "stage.h",
    "src/runtime/sensor_catalog.h": SEN + "sensor_catalog.h",
    "src/runtime/sensor_loss.cpp": "src/brain_profile/sensor_loss.cpp",
    "src/runtime/sensor_loss.h": "src/brain_profile/sensor_loss.h",
    "src/runtime/sensor_stage.cpp": SEN + "stage.cpp",
    "src/runtime/sensor_stage.h": SEN + "stage.h",
    "src/runtime/stationary_precheck.cpp": "src/brain_profile/stationary_precheck.cpp",
    "src/runtime/stationary_precheck.h": "src/brain_profile/stationary_precheck.h",
    "src/runtime/system.cpp": "src/core/system.cpp",
    "src/runtime/system.h": "src/core/system.h",
    "src/runtime/world_estimation_stage.cpp": WE + "stage.cpp",
    "src/runtime/world_estimation_stage.h": WE + "stage.h",
    # state/
    "src/state/attitude.h": LOC + "attitude.h",
    "src/state/command_state.h": CMD + "command_state.h",
    "src/state/field_state.h": WE + "field_state.h",
    "src/state/pose_history.cpp": LOC + "pose_history.cpp",
    "src/state/pose_history.h": LOC + "pose_history.h",
    "src/state/robot_state.h": LOC + "robot_state.h",
    "src/state/robot_state_feed.cpp": LOC + "robot_state_feed.cpp",
    "src/state/robot_state_feed.h": LOC + "robot_state_feed.h",
    "src/state/stillness.h": LOC + "stillness.h",
    "src/state/target_state.h": TGT + "target_state.h",
    # transport/
    "src/transport/byte_stream.h": "src/io/byte_stream.h",
    "src/transport/file_stream.cpp": "src/io/file_stream.cpp",
    "src/transport/file_stream.h": "src/io/file_stream.h",
    "src/transport/half_duplex.cpp": "src/io/half_duplex.cpp",
    "src/transport/half_duplex.h": "src/io/half_duplex.h",
    "src/transport/serial_port.cpp": "src/io/serial_port.cpp",
    "src/transport/serial_port.h": "src/io/serial_port.h",
}

# Files the noop split creates beside the git mv target (step "split").
NOOP_PIECES = [
    (CMD + "collectors/noop.h", CMD + "collectors/noop.cpp", "makeNoopCommands", "NoopCommands"),
    (LOC + "estimators/noop.h", LOC + "estimators/noop.cpp", "makeNoopStateEstimator", "NoopStateEstimator"),
    (WE + "estimators/noop/noop.h", WE + "estimators/noop/noop.cpp", "makeNoopFieldEstimation", "NoopFieldEstimation"),
    (TGT + "resolvers/noop.h", TGT + "resolvers/noop.cpp", "makeNoopTargetResolution", "NoopTargetResolution"),
    (PUB + "publishers/noop.h", PUB + "publishers/noop.cpp", "makeNoopPublishing", "NoopPublishing"),
]

OLD_DIRS = ["app", "contracts", "payloads", "state", "runtime", "impl", "resources", "transport", "core",
            "capture", "diagnostics", "inspection"]


def new_of(old):
    n = MAP[old]
    return old if n == "same" else n


def src_rel(p):
    return p[len("src/"):] if p.startswith("src/") else None


def main():
    errors = []
    # inventory
    inv = []
    for root, _, files in os.walk(os.path.join(NAV, "src")):
        for f in files:
            inv.append(os.path.relpath(os.path.join(root, f), NAV).replace("\\", "/"))
    inv_set = set(inv)
    for f in sorted(inv_set - set(MAP)):
        errors.append("unmapped: " + f)
    for f in sorted(set(MAP) - inv_set):
        errors.append("mapped but missing: " + f)
    # collisions
    dest = defaultdict(list)
    for o in MAP:
        dest[new_of(o)].append(o)
    for d, os_ in dest.items():
        if len(os_) > 1:
            errors.append("collision: %s <- %s" % (d, ", ".join(os_)))
    # noop pieces must not collide with other destinations (except the git mv target)
    for h, c, _, _ in NOOP_PIECES:
        for x in (h, c):
            if x in dest and not dest[x][0].startswith("src/impl/noop/"):
                errors.append("noop piece collides: " + x)
    # old folders empty
    for d in OLD_DIRS:
        left = [o for o in MAP if new_of(o).startswith("src/%s/" % d)]
        if d == "core":
            left = [x for x in left if not new_of(x).startswith("src/core/")]  # new core/ is allowed
            olds = [o for o in MAP if o.startswith("src/core/") and new_of(o).startswith("src/core/")]
            left += olds
        if d == "resources":
            left = [x for x in left if not new_of(x).startswith("src/pipeline/")]
        if left:
            errors.append("old folder %s not empty: %s" % (d, left))
    if any(new_of(o).split("/")[-2] == "impl" or "/impl/" in new_of(o) for o in MAP):
        errors.append("a destination uses a folder named impl")

    # TSV
    with open(os.path.join(RS, "plan", "move_map.tsv"), "w", newline="\n") as t:
        for o in sorted(MAP):
            t.write("%s\t%s\n" % (o, MAP[o]))

    out = []
    w = out.append
    w("# move_map reference report (generated by scripts/move_map.py)")
    w("files mapped: %d, inventory: %d, moving: %d, same: %d" % (
        len(MAP), len(inv), sum(1 for o in MAP if MAP[o] != "same"), sum(1 for o in MAP if MAP[o] == "same")))
    w("errors: %d" % len(errors))
    for e in errors:
        w("  " + e)

    # duplicate basenames among destinations
    base = defaultdict(list)
    for o in MAP:
        base[os.path.basename(new_of(o))].append(new_of(o))
    for h, c, _, _ in NOOP_PIECES:
        for x in (h, c):
            if x not in base[os.path.basename(x)]:
                base[os.path.basename(x)].append(x)
    w("")
    w("## duplicate basenames after the move (no path collision)")
    for b in sorted(base):
        if len(base[b]) > 1:
            w("%s x%d: %s" % (b, len(base[b]), ", ".join(sorted(base[b]))))

    # include rewrites: every file in the worktree that includes a moved src header
    inc_re = re.compile(r'^(\s*#\s*include\s*")([^"]+)(")')
    moved_src = {src_rel(o): src_rel(new_of(o)) for o in MAP
                 if MAP[o] != "same" and new_of(o).startswith("src/")}
    per_file = defaultdict(list)
    comment_hits = defaultdict(list)
    path_tokens = {}
    for o in MAP:
        if MAP[o] == "same":
            continue
        so = src_rel(o)
        path_tokens[so] = new_of(o)
    tok_re = re.compile(r'(?<![A-Za-z0-9_./-])(?:\.\./)*(?:src/)?(' + "|".join(
        re.escape(k) for k in sorted(path_tokens, key=len, reverse=True)) + r')(?![A-Za-z0-9_])')
    stem_map = {}
    for so, nw in path_tokens.items():
        stem = os.path.splitext(so)[0]
        if stem.endswith(".h"):
            stem = stem[:-2]
        stem_map.setdefault(stem, set()).add(os.path.splitext(nw)[0])
    stem_re = re.compile(r'(?<![A-Za-z0-9_./-])(?:src/)?(' + "|".join(
        re.escape(k) for k in sorted(stem_map, key=len, reverse=True)) + r')(?![A-Za-z0-9_.])')
    text_ext = (".h", ".hpp", ".c", ".cpp", ".in", ".md", ".txt", ".cmake", ".yaml", ".yml", ".sh",
                ".py", ".js", ".html", ".json", ".xml")
    doc_hits = defaultdict(list)
    for root, dirs, files in os.walk(WT):
        for skip in (".git",):
            if skip in dirs:
                dirs.remove(skip)
        rel_root = os.path.relpath(root, WT).replace("\\", "/")
        if "/third_party/" in "/" + rel_root + "/" and not rel_root.endswith("third_party"):
            continue
        for f in files:
            if not f.endswith(text_ext) and f != "CMakeLists.txt":
                continue
            p = os.path.join(root, f)
            rel = os.path.relpath(p, WT).replace("\\", "/")
            try:
                lines = open(p, encoding="utf-8", errors="replace").read().splitlines()
            except Exception:
                continue
            is_code = f.endswith((".h", ".hpp", ".c", ".cpp", ".in"))
            for i, ln in enumerate(lines, 1):
                m = inc_re.match(ln)
                if m and is_code:
                    inc = m.group(2)
                    if inc in moved_src:
                        per_file[rel].append((i, inc, moved_src[inc]))
                    continue
                hits = [(mm.group(1), path_tokens[mm.group(1)]) for mm in tok_re.finditer(ln)]
                shits = []
                if not hits:
                    shits = [(mm.group(1), "/".join(sorted(stem_map[mm.group(1)]))) for mm in stem_re.finditer(ln)]
                if hits or shits:
                    entry = (i, ln.strip()[:160], hits + shits)
                    if is_code:
                        comment_hits[rel].append(entry)
                    else:
                        doc_hits[rel].append(entry)

    w("")
    w("## #include rewrites per file (file: count)")
    total = 0
    groups = defaultdict(list)
    for rel in sorted(per_file):
        total += len(per_file[rel])
        top = rel.split("/")[0] if not rel.startswith("pi/naviGATR/") else "/".join(rel.split("/")[:3])
        groups[top].append(rel)
    w("total include lines to rewrite: %d in %d files" % (total, len(per_file)))
    for g in sorted(groups):
        w("### %s (%d files, %d lines)" % (g, len(groups[g]), sum(len(per_file[r]) for r in groups[g])))
        for rel in groups[g]:
            w("%s: %d" % (rel, len(per_file[rel])))
    w("")
    w("## #include rewrites, full list")
    for rel in sorted(per_file):
        for i, inc, nw in per_file[rel]:
            w("%s:%d\t%s\t->\t%s" % (rel, i, inc, nw))
    w("")
    w("## path mentions in code comments/strings (not includes)")
    for rel in sorted(comment_hits):
        for i, txt, hits in comment_hits[rel]:
            w("%s:%d\t%s\t[%s]" % (rel, i, txt, "; ".join("%s -> %s" % h for h in hits)))
    w("")
    w("## path mentions in docs, CMake, scripts, yaml")
    for rel in sorted(doc_hits):
        for i, txt, hits in doc_hits[rel]:
            w("%s:%d\t%s\t[%s]" % (rel, i, txt, "; ".join("%s -> %s" % h for h in hits)))

    with open(os.path.join(RS, "logs", "move_map_refs.txt"), "w", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    print("\n".join(out[:4 + len(errors)]))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
