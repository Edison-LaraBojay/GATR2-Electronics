#!/usr/bin/env python3
"""Split naviGATR System (system.cpp + system.h) into per-concern files.

Pure move: every definition body and every declaration is copied verbatim.
Only new boilerplate (file label, includes, namespace lines, section labels)
is generated. A coverage check proves each original non-blank line lands in
exactly one output (or in the explicit drop list).

Usage:
  py -3 split_system.py --src <naviGATR/src> [--rename <tsv>] [--dry-run]
      [--cpp <rel path of system.cpp>] [--h <rel path of system.h>]

--rename: TSV of old<TAB>new include paths (relative to src/, or full
  pi/naviGATR/src/... paths; the prefix is stripped). Applied to every
  generated #include line. Use step 1's move map after the move; omit it on
  the ae457ee layout (trial).
Outputs are written under --src. The old system.cpp is deleted (unless
--dry-run); system.h is rewritten in place.

  py -3 split_system.py --src <naviGATR/src> --verify <dir> [--rename <tsv>]
checks an already split tree: <dir> holds the original system.cpp and
system.h; every segment must appear (whitespace-insensitive) in its
destination, and the leftover lines of each file are printed.
"""

import argparse
import collections
import os
import re
import sys

# ---- destinations (paths relative to src/, target layout) -----------------

BUILD = "core/build.cpp"
RUN = "core/run.cpp"
PROFILE_HOST_H = "brain_profile/profile_host.h"
PROFILE_HOST = "brain_profile/profile_host.cpp"
CONTROL = "brain_profile/control.cpp"
PICO_OP_H = "brain_profile/pico_operation.h"
PICO_OP = "brain_profile/pico_operation.cpp"
RECOVERY = "brain_profile/recovery.cpp"
EVENTS_H = "debug/diagnostics/events.h"
EVENTS = "debug/diagnostics/events.cpp"
INSPECTION = "debug/inspection/inspection_state.cpp"

# includes use ae457ee paths; --rename maps them to the post-move layout
SYSTEM_H = "runtime/system.h"
INSPECTION_STATE_H = "runtime/inspection_state.h"

# ---- system.cpp segments, in file order ------------------------------------
# (name, exact first line). A segment runs to the line before the next one.
CPP_SEGMENTS = [
    ("preamble", "// system.cpp"),
    ("Tag", "// Carries a factory signature type into the generic slot builder."),
    ("checkChildren", "// Only the listed children may appear, each at most once for singular ones."),
    ("newSessionId", "std::string newSessionId() {"),
    ("elapsedMs", "double elapsedMs(std::chrono::steady_clock::time_point since) {"),
    ("kEventsKept", "constexpr std::size_t kEventsKept = 32;"),
    ("hexId", "std::string hexId(uint32_t id) {"),
    ("anon_close", "} // namespace"),
    ("ProfileHost", "// The System as the brain_link commands slot sees it."),
    ("ctor", "System::System() = default;"),
    ("buildFromFile", "std::unique_ptr<System> System::buildFromFile(const std::string& path,"),
    ("buildFromString", "std::unique_ptr<System> System::buildFromString(const char*             xml,"),
    ("dtor", "System::~System() {"),
    ("build", "bool System::build(const char* xml, const FunctionRegistry& functions,"),
    ("buildProfileHost", "bool System::buildProfileHost(const ConfigNode& pipeline, const ConfigNode& profile,"),
    ("buildWaitingLocalization", "std::optional<LocalizationExecutor> System::buildWaitingLocalization(std::string& err) {"),
    ("requestsFrom", "LocalizationRequests System::requestsFrom(const CommandState& command) const {"),
    ("banner_brain_profile", None),  # "// ---- Brain profile ---..."
    ("prepareProfile", "bool System::prepareProfile(const translagatr::RobotProfileDoc& profile, uint32_t id, uint8_t& reason,"),
    ("controlProfile", "uint8_t System::controlProfile(uint8_t action, uint8_t, MonotonicTime now, uint8_t& detail) {"),
    ("submitPico", "uint32_t System::submitPico(uint8_t op, uint8_t arg, MonotonicTime now, uint8_t& detail) {"),
    ("controlProgress", "uint8_t System::controlProgress(uint8_t action, MonotonicTime now, uint8_t& detail) {"),
    ("updatePicoOperation", "void System::updatePicoOperation(MonotonicTime now) {"),
    ("readWheels", "uint8_t System::readWheels(MonotonicTime now, uint8_t& count, translagatr::WheelReading* wheels) const {"),
    ("watchSensors", "void System::watchSensors(const SensorMap& sensors, MonotonicTime now) {"),
    ("watchModels", "void System::watchModels(MonotonicTime now) {"),
    ("losePlacement", "void System::losePlacement(MonotonicTime now, const std::string& why, bool note) {"),
    ("noteRecovery", "void System::noteRecovery(MonotonicTime now) {"),
    ("applyPendingProfile", "bool System::applyPendingProfile() {"),
    ("swapProfile", "void System::swapProfile() {"),
    ("swapToWaiting", "void System::swapToWaiting() {"),
    ("pauseWorkers", "bool System::pauseWorkers() {"),
    ("resumeWorkers", "void System::resumeWorkers(bool was_running) {"),
    ("profileBinding", "std::shared_ptr<const ProfileBinding> System::profileBinding() const {"),
    ("publishBindingView", "void System::publishBindingView() {"),
    ("bindingView", "std::shared_ptr<const BindingView> System::bindingView() const {"),
    ("noteEvent", "void System::noteEvent(MonotonicTime at, std::string text) {"),
    ("noteFault", "void System::noteFault(CaptureFault fault, const std::string& text) {"),
    ("events", "std::vector<RuntimeEvent> System::events() const {"),
    ("banner_snapshots", None),  # "// ---- snapshot readers ---..."
    ("fieldSnapshot", "std::shared_ptr<const FieldSnapshot> System::fieldSnapshot() const {"),
    ("reportingSnapshot", "std::shared_ptr<const ReportingSnapshot> System::reportingSnapshot() const {"),
    ("sourceHealth", "std::shared_ptr<const SourceHealthSnapshot> System::sourceHealth() const {"),
    ("diagnosticsSnapshot", "std::shared_ptr<const DiagnosticsSnapshot> System::diagnosticsSnapshot() const {"),
    ("detectionFrames", "std::map<SensorId, std::shared_ptr<const DetectionFrameSnapshot>>"),
    ("banner_cycles", None),  # "// ---- cycles ---..."
    ("step", "void System::step(MonotonicTime now) {"),
    ("estimationCycle", "void System::estimationCycle(MonotonicTime now) {"),
    ("reportingCycle", "void System::reportingCycle(MonotonicTime now) {"),
    ("fieldCycle", "void System::fieldCycle(const SensorMap& sensors, MonotonicTime now) {"),
    ("publishSourceHealth", "void System::publishSourceHealth(MonotonicTime now) {"),
    ("publishDetectionFrames", "void System::publishDetectionFrames(const SensorMap& sensors, const FieldSnapshot& snapshot,"),
    ("banner_lifecycle", None),  # "// ---- lifecycle ---..."
    ("resetStages", "void System::resetStages() {"),
    ("reset", "void System::reset() {"),
    ("start", "bool System::start(std::string& err) {"),
    ("startWorkers", "bool System::startWorkers(std::string& err) {"),
    ("requestStop", "void System::requestStop() {"),
    ("stop", "void System::stop() {"),
    ("estimationWorker", "void System::estimationWorker() {"),
    ("fieldWorker", "void System::fieldWorker() {"),
    ("ns_close", "} // namespace navigatr"),
]

BANNER_PREFIX = {
    "banner_brain_profile": "// ---- Brain profile",
    "banner_snapshots": "// ---- snapshot readers",
    "banner_cycles": "// ---- cycles",
    "banner_lifecycle": "// ---- lifecycle",
}

# structural segments regenerated per file, or dropped
CPP_DROPPED = {"preamble", "anon_close", "ns_close", "banner_brain_profile"}

# dest -> (own header, std includes, project includes, anon helpers, body segments)
# own header first, then <std>, then project headers sorted (clang-format order)
CPP_FILES = collections.OrderedDict([
    (BUILD, dict(
        own=SYSTEM_H,
        std=["chrono", "cstdio", "random", "set"],
        proj=[PROFILE_HOST_H, "config/composition.h", "config/config_node.h",
              "tinyxml2/tinyxml2.h"],
        anon=["Tag", "checkChildren", "newSessionId"],
        body=["ctor", "buildFromFile", "buildFromString", "dtor", "build"])),
    (RUN, dict(
        own=SYSTEM_H,
        std=["array", "chrono", "stdexcept"],
        proj=["core/host_clock.h"],
        anon=["elapsedMs"],
        body=["banner_cycles", "step", "estimationCycle", "requestsFrom", "reportingCycle",
              "fieldCycle", "banner_lifecycle", "resetStages", "reset", "start", "startWorkers",
              "requestStop", "stop", "estimationWorker", "fieldWorker", "pauseWorkers",
              "resumeWorkers", "banner_snapshots", "fieldSnapshot", "reportingSnapshot",
              "diagnosticsSnapshot", "publishBindingView", "bindingView"])),
    (PROFILE_HOST, dict(
        own=PROFILE_HOST_H,
        std=["cstdio"],
        proj=["config/config_node.h", "runtime/pico_control_ref.h", "tinyxml2/tinyxml2.h"],
        anon=["hexId"],
        body=["buildProfileHost", "buildWaitingLocalization", "prepareProfile",
              "applyPendingProfile", "swapProfile", "swapToWaiting", "profileBinding"])),
    (CONTROL, dict(
        own=SYSTEM_H,
        std=["algorithm", "cmath"],
        proj=["payloads/pico_telemetry_samples.h", "payloads/sensor_samples.h"],
        anon=[],
        body=["controlProfile", "readWheels"])),
    (PICO_OP, dict(
        own=PICO_OP_H,
        std=[],
        proj=[SYSTEM_H],
        anon=[],
        body=["submitPico", "controlProgress", "updatePicoOperation"])),
    (RECOVERY, dict(
        own=SYSTEM_H,
        std=[],
        proj=[],
        anon=[],
        body=["watchSensors", "watchModels", "losePlacement", "noteRecovery"])),
    (EVENTS, dict(
        own=EVENTS_H,
        std=["algorithm", "cstring"],
        proj=[SYSTEM_H],
        anon=["kEventsKept"],
        body=["noteEvent", "noteFault", "events"])),
    (INSPECTION, dict(
        own=INSPECTION_STATE_H,
        std=[],
        proj=["core/host_clock.h", SYSTEM_H],
        anon=[],
        body=["sourceHealth", "detectionFrames", "publishSourceHealth",
              "publishDetectionFrames"])),
])

# ---- system.h segments (ae457ee line numbers, inclusive) -------------------
# Verified by first-line text; shifted by the offset of "class System" (class
# body) or "struct FieldSnapshot {" (namespace-scope structs).
H_STRUCTS = [
    ("FieldSnapshot", 131, 142, "// What the field stage publishes per invocation: the estimate plus the"),
    ("PicoOperation", 144, 154, "// A CONTROL 3 or 4 running on the Pico, wire codes."),
    ("ReportingSnapshot", 156, 167, "// Command and target state as the estimation worker last left them, and"),
    ("BindingView", 169, 177, "// What the running configuration binds, for readers on any thread. Replaced"),
    ("RuntimeEvent", 179, 184, "// A lifecycle event worth a line in a log or the inspection page."),
]
H_STRUCT_ANCHOR = ("struct FieldSnapshot {", 134)

H_CLASS = [
    ("P_constants", 189, 190, "static constexpr int64_t kPicoOperationMs = 15000;   // CONTROL 3/4 bound"),
    ("P_buildFromFile", 192, 195, "static std::unique_ptr<System> buildFromFile(const std::string& path,"),
    ("P_buildFromString", 196, 202, "// Inline/resolved System XML only. Section file references require"),
    ("P_dtor", 203, 203, "~System();"),
    ("P_step", 205, 207, "// One full pipeline cycle, every stage inline. now is the host"),
    ("P_estimationCycle", 209, 211, "// The estimation cycle alone: resources, sensors, commands,"),
    ("P_fieldCycle", 213, 215, "// World estimation alone against a sensor snapshot; publishes a new"),
    ("P_reportingCycle", 217, 218, "// Target resolution and publishing against the newest field snapshot."),
    ("P_workers", 220, 224, "// Workers. start() spawns the estimation and field workers; stop()"),
    ("P_reset", 226, 231, "// Back to power-on: implementations reset, states cleared, counters"),
    ("P_applyPending", 233, 238, "// Worker mode profile boundary, called periodically by the thread that"),
    ("P_profileBinding", 240, 241, "// The running Brain profile, null when none; any thread."),
    ("P_events", 243, 244, "// Newest lifecycle events, oldest first, bounded; any thread."),
    ("P_picoControl", 246, 248, "// The Pico link the brain_link CommandCollection names, null when none."),
    ("P_rate_cycle", 250, 251, "double   loopRateHz() const { return loop_rate_hz_; }"),
    ("P_commandsType", 253, 255, "// Configured CommandCollection type, e.g. brain_link, which needs the"),
    ("P_configuration", 257, 262, "// Identity of the running profile: the Configuration id (or the file"),
    ("P_session", 264, 267, "// Identity of this process instance, for inspection clients to detect"),
    ("P_retained", 269, 278, "// Latest stage results, retained between cycles. Inline mode or"),
    ("P_debug", 280, 289, "// Diagnostic records for capture and live instrumentation. Internally"),
    ("P_fieldDiagnostics", 290, 290, "Diagnostics&        fieldDiagnostics() { return field_diagnostics_; }"),
    ("P_readers_head", 292, 295, "// Thread-safe readers, for the other worker and inspection. The feed"),
    ("P_executors", 296, 298, "// Inline mode or stopped workers only: a profile boundary replaces it."),
    ("P_readers_tail", 299, 305, "std::shared_ptr<const FieldSnapshot>       fieldSnapshot() const;"),
    ("P_inspection", 307, 307, "const InspectionConfig& inspection() const { return inspection_; }"),
    ("P_warnings", 309, 311, "// Non-fatal build notes, e.g. a serial device that failed to open."),
    ("P_tooling", 313, 317, "// Initialized objects and declared outputs, for tests and tooling."),
    ("Q_ProfileHost_fwd", 320, 320, "class ProfileHost;"),
    ("Q_ProfileCandidate", 322, 328, "// A built profile waiting for the boundary."),
    ("Q_ctor", 330, 330, "System();"),
    ("Q_build", 332, 333, "bool build(const char* xml, const FunctionRegistry& functions,"),
    ("Q_buildProfileHost", 334, 335, "bool buildProfileHost(const ConfigNode& pipeline, const ConfigNode& profile,"),
    ("Q_buildWaiting", 336, 336, "std::optional<LocalizationExecutor> buildWaitingLocalization(std::string& err);"),
    ("Q_requestsFrom", 338, 338, "LocalizationRequests requestsFrom(const CommandState& command) const;"),
    ("Q_host_group", 340, 345, "// BrainProfileHost, on the estimation worker."),
    ("Q_pico_group", 347, 353, "// Pico commands and recovery bookkeeping, on the estimation worker."),
    ("Q_boundary_comment", 355, 355, "// Boundary work: workers stopped or inline."),
    ("Q_swaps", 356, 357, "void swapProfile();"),
    ("Q_pause_resume", 358, 359, "bool pauseWorkers();   // lifecycle mutex held; true when they were running"),
    ("Q_publishBindingView", 360, 360, "void publishBindingView();"),
    ("Q_noteEvent_noteFault", 361, 362, "void noteEvent(MonotonicTime at, std::string text);"),
    ("Q_workers", 364, 368, "void resetStages();"),
    ("Q_inspection", 369, 371, "void publishSourceHealth(MonotonicTime now);"),
    ("D_data", 373, 476, "double loop_rate_hz_ = 100.0;"),
]
H_CLASS_ANCHOR = ("class System", 186)

# new class layout: labels are added lines; everything else is a segment name
H_LAYOUT = [
    "class System",
    "{",
    "public:",
    "// Build",
    "P_buildFromFile", "P_buildFromString", "P_dtor",
    "",
    "// Run",
    "P_step", "", "P_estimationCycle", "", "P_fieldCycle", "", "P_reportingCycle", "",
    "P_workers", "", "P_reset",
    "",
    "// Brain profile",
    "P_constants", "", "P_applyPending", "", "P_profileBinding", "", "P_picoControl",
    "",
    "// Configuration and identity",
    "P_rate_cycle", "", "P_commandsType", "", "P_configuration", "", "P_session", "",
    "P_inspection", "", "P_warnings",
    "",
    "// Snapshots",
    "P_readers_head", "P_readers_tail", "", "P_events",
    "",
    "// Retained state",
    "P_retained", "P_fieldDiagnostics", "", "P_executors", "", "P_tooling",
    "",
    "// Debug",
    "P_debug",
    "",
    "private:",
    "Q_ProfileHost_fwd", "", "Q_ProfileCandidate", "", "Q_ctor",
    "",
    "// Build",
    "Q_build",
    "",
    "// Run",
    "Q_requestsFrom", "Q_workers", "Q_pause_resume", "Q_publishBindingView",
    "",
    "// Brain profile",
    "Q_buildProfileHost", "Q_buildWaiting", "", "Q_host_group", "", "Q_boundary_comment",
    "Q_swaps", "", "Q_pico_group",
    "",
    "// Events and capture faults",
    "Q_noteEvent_noteFault",
    "",
    "// Inspection snapshots",
    "Q_inspection",
    "",
    "D_data",
    "};",
]
H_ADDED_LABELS = collections.Counter(l for l in H_LAYOUT if l.startswith("// "))


def die(msg):
    sys.stderr.write("split_system: " + msg + "\n")
    sys.exit(1)


def load_rename(path):
    ren = {}
    if not path:
        return ren

    def strip(p):
        for prefix in ("pi/naviGATR/src/", "src/"):
            if p.startswith(prefix):
                return p[len(prefix):], True
        return p, False

    with open(path, encoding="utf-8") as f:
        for raw in f:
            raw = raw.rstrip("\r\n")
            if not raw or raw.startswith("#"):
                continue
            parts = raw.split("\t")
            if len(parts) < 2:
                continue
            old, old_src = strip(parts[0].strip())
            new, new_src = strip(parts[1].strip())
            if old_src and new_src:   # include paths are relative to src/
                ren[old] = new
    return ren


def strip_trailing_blank(lines):
    while lines and not lines[-1].strip():
        lines = lines[:-1]
    while lines and not lines[0].strip():
        lines = lines[1:]
    return lines


def cut_cpp(lines):
    starts = []
    pos = 0
    for name, first in CPP_SEGMENTS:
        found = None
        for i in range(pos, len(lines)):
            text = lines[i].rstrip("\n")
            if first is None:
                if text.startswith(BANNER_PREFIX[name]):
                    found = i
                    break
            elif text == first:
                found = i
                break
        if found is None:
            die("cpp anchor not found after line %d: %s" % (pos + 1, name))
        starts.append((name, found))
        pos = found + 1
    segs = collections.OrderedDict()
    for k, (name, s) in enumerate(starts):
        e = starts[k + 1][1] if k + 1 < len(starts) else len(lines)
        segs[name] = (s + 1, e, strip_trailing_blank(lines[s:e]))
    return segs


def cut_h(lines):
    def find(text):
        hits = [i for i, l in enumerate(lines) if l.rstrip("\n") == text]
        if len(hits) != 1:
            die("header anchor %r found %d times" % (text, len(hits)))
        return hits[0] + 1

    ds = find(H_STRUCT_ANCHOR[0]) - H_STRUCT_ANCHOR[1]
    dc = find(H_CLASS_ANCHOR[0]) - H_CLASS_ANCHOR[1]
    segs = collections.OrderedDict()
    for table, d in ((H_STRUCTS, ds), (H_CLASS, dc)):
        for name, a, b, first in table:
            block = lines[a - 1 + d:b + d]
            if block[0].strip() != first.strip():
                die("header segment %s: line %d is %r, expected %r"
                    % (name, a + d, block[0].strip(), first))
            segs[name] = (a + d, b + d, block)
    class_line = find(H_CLASS_ANCHOR[0])
    ns_line = find("namespace navigatr") if any(
        l.rstrip("\n") == "namespace navigatr" for l in lines) else None
    if ns_line is None:
        die("header: no namespace navigatr line")
    return segs, ds, dc, class_line, ns_line


INCLUDE_NOTES = {
    (BUILD, PROFILE_HOST_H): "   // complete ProfileHost: ctor, dtor, build",
}


def inc(path, ren, note=""):
    return '#include "%s"%s\n' % (ren.get(path, path), note)


def emit_cpp(dest, spec, segs, ren):
    out = ["// %s\n" % os.path.basename(dest), "\n"]
    out.append(inc(spec["own"], ren))
    if spec["std"]:
        out.append("\n")
        for s in spec["std"]:
            out.append("#include <%s>\n" % s)
    if spec["proj"]:
        out.append("\n")
        for p in sorted(spec["proj"], key=lambda x: ren.get(x, x)):
            out.append(inc(p, ren, INCLUDE_NOTES.get((dest, p), "")))
    out += ["\n", "namespace navigatr\n", "{\n", "\n"]
    if spec["anon"]:
        out += ["namespace\n", "{\n", "\n"]
        for k, name in enumerate(spec["anon"]):
            out += segs[name][2]
            out.append("\n")
        out += ["} // namespace\n", "\n"]
    for name in spec["body"]:
        out += segs[name][2]
        out.append("\n")
    out.append("} // namespace navigatr\n")
    return out


def emit_small_header(dest, includes_std, includes_proj, blocks, ren):
    out = ["// %s\n" % os.path.basename(dest), "\n", "#pragma once\n"]
    for s in includes_std:
        out.append("#include <%s>\n" % s)
    if includes_proj:
        if includes_std:
            out.append("\n")
        for p in includes_proj:
            out.append(inc(p, ren))
    out += ["\n", "namespace navigatr\n", "{\n", "\n"]
    for b in blocks:
        out += b
        out.append("\n")
    out.append("} // namespace navigatr\n")
    return out


def emit_system_h(lines, segs, class_line, ns_line, ren, rename_preamble=False):
    # preamble: everything before "namespace navigatr", plus the two new includes
    pre = lines[:ns_line - 1]
    if rename_preamble:
        pre = [inc(m.group(1), ren) if m else l
               for l, m in ((l, re.match(r'#include "([^"]+)"\s*$', l)) for l in pre)]
    new_incs = [inc(PICO_OP_H, ren), inc(EVENTS_H, ren)]
    # insert into the project include block, keeping it sorted
    inc_idx = [i for i, l in enumerate(pre) if l.startswith('#include "')]
    if not inc_idx:
        die("header: no project includes")
    block = [pre[i] for i in inc_idx] + new_incs
    block.sort()
    first, last = inc_idx[0], inc_idx[-1]
    if last - first + 1 != len(inc_idx):
        die("header: project includes are not one contiguous block")
    pre = pre[:first] + block + pre[last + 1:]

    out = list(pre)
    out += ["namespace navigatr\n", "{\n", "\n"]
    for name in ("FieldSnapshot", "ReportingSnapshot", "BindingView"):
        out += segs[name][2]
        out.append("\n")
    for item in H_LAYOUT:
        if item in segs:
            out += segs[item][2]
        elif item == "":
            out.append("\n")
        elif item in ("class System", "{", "};"):
            out.append(item + "\n")
        elif item in ("public:", "private:"):
            out.append(item + "\n")
        elif item.startswith("// "):
            out.append("    " + item + "\n")
            out.append("\n")
        else:
            die("layout item unknown: " + item)
    out += ["\n", "} // namespace navigatr\n"]
    return out


def nonblank(lines):
    return collections.Counter(l.rstrip() for l in lines if l.strip())


def verify(orig_dir, src, ren):
    """Each original segment appears, whitespace-insensitively, in its
    destination file; prints what is left in each file (the residue)."""
    with open(os.path.join(orig_dir, "system.cpp"), encoding="utf-8") as f:
        cpp = f.read().splitlines(keepends=True)
    with open(os.path.join(orig_dir, "system.h"), encoding="utf-8") as f:
        hdr = f.read().splitlines(keepends=True)
    segs = cut_cpp(cpp)
    hsegs = cut_h(hdr)[0]
    h_rel = ren.get("runtime/system.h", "runtime/system.h")
    want = collections.defaultdict(list)
    for dest, spec in CPP_FILES.items():
        for n in spec["anon"] + spec["body"]:
            want[dest].append(("cpp:" + n, segs[n][2]))
    want[PROFILE_HOST_H].append(("cpp:ProfileHost", segs["ProfileHost"][2]))
    for n, (s, e, b) in hsegs.items():
        dest = {"PicoOperation": PICO_OP_H, "RuntimeEvent": EVENTS_H}.get(n, h_rel)
        want[dest].append(("h:" + n, b))
    bad = 0
    for dest, items in want.items():
        with open(os.path.join(src, dest), encoding="utf-8") as f:
            text = f.read()
        # longest first, so a short segment never eats part of a long one
        for name, block in sorted(items, key=lambda x: -len("".join(x[1]))):
            flat = re.sub(r"\s+", "", "".join(block))
            pat = r"\s*".join(re.escape(c) for c in flat)
            m = re.search(pat, text)
            if m is None:
                print("MISSING %s in %s" % (name, dest))
                bad += 1
                continue
            text = text[:m.start()] + "\n" + text[m.end():]
        residue = [l for l in text.splitlines() if l.strip()]
        print("== %s: %d segments, residue %d lines" % (dest, len(items), len(residue)))
        for l in residue:
            print("   | " + l)
    if bad:
        die("%d segments missing" % bad)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--cpp", default=None)
    ap.add_argument("--h", default=None)
    ap.add_argument("--cpp-in", default=None,
                    help="read system.cpp from here (e.g. the ae457ee text) instead of --src")
    ap.add_argument("--h-in", default=None,
                    help="read system.h from here; its include block is renamed too")
    ap.add_argument("--rename", default=None)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--report", default=None)
    ap.add_argument("--verify", default=None,
                    help="dir with the original system.cpp and system.h; checks --src")
    a = ap.parse_args()

    ren = load_rename(a.rename)
    if a.verify:
        verify(a.verify, a.src, ren)
        return
    cpp_rel = a.cpp or ren.get("runtime/system.cpp", "runtime/system.cpp")
    h_rel = a.h or ren.get("runtime/system.h", "runtime/system.h")
    cpp_path = a.cpp_in or os.path.join(a.src, cpp_rel)
    h_path = a.h_in or os.path.join(a.src, h_rel)
    old_cpp_in_src = os.path.join(a.src, cpp_rel)
    with open(cpp_path, encoding="utf-8", newline="") as f:
        cpp = f.read().splitlines(keepends=True)
    with open(h_path, encoding="utf-8", newline="") as f:
        hdr = f.read().splitlines(keepends=True)
    if any("\r" in l for l in cpp + hdr):
        die("CRLF input; expected LF")

    segs = cut_cpp(cpp)
    hsegs, ds, dc, class_line, ns_line = cut_h(hdr)

    # every cpp segment used exactly once or dropped
    used = collections.Counter()
    for spec in CPP_FILES.values():
        for n in spec["anon"] + spec["body"]:
            used[n] += 1
    used_ph = ["ProfileHost"]
    for n in used_ph:
        used[n] += 1
    for name in segs:
        if name in CPP_DROPPED:
            if used[name]:
                die("dropped segment used: " + name)
            continue
        if used[name] != 1:
            die("segment %s used %d times" % (name, used[name]))
    for n in used:
        if n not in segs:
            die("unknown segment: " + n)

    outputs = collections.OrderedDict()
    for dest, spec in CPP_FILES.items():
        outputs[dest] = emit_cpp(dest, spec, segs, ren)
    outputs[PROFILE_HOST_H] = emit_small_header(
        PROFILE_HOST_H, [], [SYSTEM_H], [segs["ProfileHost"][2]], ren)
    outputs[PICO_OP_H] = emit_small_header(
        PICO_OP_H, ["cstdint"], ["core/time.h"], [hsegs["PicoOperation"][2]], ren)
    outputs[EVENTS_H] = emit_small_header(
        EVENTS_H, ["cstdint", "string"], ["core/time.h"], [hsegs["RuntimeEvent"][2]], ren)
    outputs[h_rel] = emit_system_h(hdr, hsegs, class_line, ns_line, ren,
                                   rename_preamble=bool(a.h_in))

    # coverage: cpp bodies
    orig_cpp = nonblank(l for name, (s, e, b) in segs.items()
                        if name not in CPP_DROPPED for l in b)
    moved_cpp = collections.Counter()
    for dest, spec in CPP_FILES.items():
        for n in spec["anon"] + spec["body"]:
            moved_cpp += nonblank(segs[n][2])
    moved_cpp += nonblank(segs["ProfileHost"][2])
    if orig_cpp != moved_cpp:
        die("cpp coverage mismatch")
    # coverage: header, class and structs
    h_body_orig = nonblank(hdr[ns_line - 1:])
    h_body_new = nonblank(outputs[h_rel][len(hdr[:ns_line - 1]) + 2:])
    h_body_new += nonblank(hsegs["PicoOperation"][2]) + nonblank(hsegs["RuntimeEvent"][2])
    for lab, n in H_ADDED_LABELS.items():
        h_body_new["    " + lab] -= n
    h_body_new = +h_body_new
    if h_body_orig != h_body_new:
        diff_a = h_body_orig - h_body_new
        diff_b = h_body_new - h_body_orig
        die("header coverage mismatch:\n  missing %r\n  extra %r" % (dict(diff_a), dict(diff_b)))

    rep = []
    rep.append("system.cpp segments (%s):" % cpp_rel)
    where = {}
    for dest, spec in CPP_FILES.items():
        for n in spec["anon"]:
            where[n] = dest + " (anonymous namespace)"
        for n in spec["body"]:
            where[n] = dest
    where["ProfileHost"] = PROFILE_HOST_H
    for name, (s, e, b) in segs.items():
        last = s + len(b) - 1 if b else s
        rep.append("  %-26s %5d-%-5d %4d  %s" % (name, s, last, len(b),
                                                where.get(name, "(regenerated or dropped)")))
    rep.append("system.h segments (%s), struct offset %+d, class offset %+d:" % (h_rel, ds, dc))
    for name, (s, e, b) in hsegs.items():
        rep.append("  %-26s %5d-%-5d" % (name, s, e))
    rep.append("outputs:")
    for dest, out in outputs.items():
        rep.append("  %-44s %5d lines" % (dest, len(out)))
    text = "\n".join(rep) + "\n"
    sys.stdout.write(text)
    if a.report:
        with open(a.report, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)

    if a.dry_run:
        return
    for dest, out in outputs.items():
        p = os.path.join(a.src, dest)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        if os.path.exists(p) and dest != h_rel:
            die("refusing to overwrite " + dest)
        with open(p, "w", encoding="utf-8", newline="\n") as f:
            f.writelines(out)
    if os.path.exists(old_cpp_in_src):
        os.remove(old_cpp_in_src)


if __name__ == "__main__":
    main()
