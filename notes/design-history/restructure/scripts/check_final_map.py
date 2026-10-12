# Read-only check of a move map against the worktree.
#
#   py -3 check_final_map.py <move_map.tsv> [<report.txt>]
#
# Checks: coverage (every src file once), destinations unique, folder rules,
# plug-in folder names vs XML elements, every quoted include in src/tests/tools
# resolves after the move (noop split and step 3 files included), include
# cycles, and cross-folder edges worth knowing (layering).

import os
import re
import sys
from collections import defaultdict

WT = r"C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/wt-navi"
NAV = os.path.join(WT, "pi", "naviGATR")

P = "src/pipeline/"
STAGES = ["resources", "sensors", "commands", "localization", "world_estimation", "targets",
          "publishing"]

# plug-in folder -> XML element that selects what is inside
PLUGIN_ELEMENT = {
    P + "resources/resources/": "Resource",
    P + "sensors/sensors/": "Sensor",
    P + "commands/command_collections/": "CommandCollection",
    P + "localization/observations/": "Observation",
    P + "localization/estimators/": "Estimator",
    P + "world_estimation/estimators/": "Estimator",
    P + "targets/target_resolutions/": "TargetResolution",
    P + "publishing/publishings/": "Publishing",
}

OLD_TOP = ["app", "contracts", "payloads", "state", "runtime", "impl", "resources", "transport",
           "capture", "diagnostics", "inspection"]


def snake_plural(element):
    s = re.sub(r"(?<!^)([A-Z])", r"_\1", element).lower()
    return s + "s"


def main():
    tsv = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else None
    lines = []

    def say(s=""):
        lines.append(s)

    errors = []

    rows = []
    with open(tsv, encoding="utf-8") as f:
        for n, raw in enumerate(f, 1):
            raw = raw.rstrip("\r\n")
            if not raw:
                continue
            parts = raw.split("\t")
            if len(parts) != 2:
                errors.append("tsv line %d: not two columns" % n)
                continue
            rows.append((parts[0], parts[1]))

    inventory = []
    for root, _, files in os.walk(os.path.join(NAV, "src")):
        for fn in files:
            rel = os.path.relpath(os.path.join(root, fn), NAV).replace("\\", "/")
            inventory.append(rel)
    inventory.sort()

    olds = [o for o, _ in rows]
    dup_old = sorted({o for o in olds if olds.count(o) > 1})
    missing = sorted(set(inventory) - set(olds))
    extra = sorted(set(olds) - set(inventory))
    for x in dup_old:
        errors.append("listed twice: " + x)
    for x in missing:
        errors.append("missing from map: " + x)
    for x in extra:
        errors.append("mapped but does not exist: " + x)

    dest = {}
    for o, n in rows:
        d = o if n == "same" else n
        if d in dest.values():
            errors.append("destination used twice: " + d)
        dest[o] = d

    # files that exist after step 1 (noop split) and after step 3
    noop_pieces = [
        P + "commands/command_collections/noop.h", P + "commands/command_collections/noop.cpp",
        P + "localization/estimators/noop.h", P + "localization/estimators/noop.cpp",
        P + "targets/target_resolutions/noop.h", P + "targets/target_resolutions/noop.cpp",
        P + "publishing/publishings/noop.h", P + "publishing/publishings/noop.cpp",
    ]
    for x in noop_pieces:
        if x in dest.values():
            errors.append("noop piece collides with a moved file: " + x)
    after = set(dest.values()) | set(noop_pieces)

    say("rows %d, inventory %d, same %d, moving %d" % (
        len(rows), len(inventory), sum(1 for _, n in rows if n == "same"),
        sum(1 for _, n in rows if n != "same")))

    # folder rules
    for d in sorted(set(dest.values())):
        parts = d.split("/")
        if "impl" in parts:
            errors.append("impl folder: " + d)
        if d.startswith("src/") and len(parts) > 2 and parts[1] in OLD_TOP:
            errors.append("old top-level folder survives: " + d)
        if d.startswith("src/core/") and os.path.basename(d) not in (
                "system.h", "system.cpp", "handoff.h", "build.cpp", "run.cpp"):
            errors.append("core/ holds more than the engine: " + d)
    for folder, element in PLUGIN_ELEMENT.items():
        name = folder.rstrip("/").split("/")[-1]
        if name != snake_plural(element):
            errors.append("plug-in folder %s does not match <%s>" % (folder, element))
    plugin_dirs = sorted({os.path.dirname(d) + "/" for d in dest.values()
                          if d.startswith(P) and d.count("/") >= 4})
    say()
    say("plug-in folders in the map:")
    for pd in plugin_dirs:
        el = None
        for k, v in PLUGIN_ELEMENT.items():
            if pd.startswith(k):
                el = v
        say("  %-52s %s" % (pd, "<%s>" % el if el else "NOT A PLUG-IN FOLDER"))
        if el is None:
            errors.append("folder below a stage that is no plug-in folder: " + pd)

    # includes
    old_inc = {}   # old src-relative include path -> new src-relative
    for o, d in dest.items():
        if o.startswith("src/") and d.startswith("src/"):
            old_inc[o[4:]] = d[4:]
    after_inc = {d[4:] for d in after if d.startswith("src/")}
    after_inc |= {"pipeline/world_estimation/estimators/apriltag/tag_association_trace.h",
                  "math/units.h"}

    inc_re = re.compile(r'^\s*#\s*include\s+"([^"]+)"')
    scan = []
    for top in ("src", "tests", "tools"):
        for root, _, files in os.walk(os.path.join(NAV, top)):
            for fn in files:
                if fn.endswith((".h", ".cpp", ".c", ".in")):
                    scan.append(os.path.relpath(os.path.join(root, fn), NAV).replace("\\", "/"))
    graph = defaultdict(set)
    unresolved = []
    total = 0
    rewritten = 0
    for f in sorted(scan):
        new_f = dest.get(f, f)
        if f.startswith("src/impl/noop/"):
            # split into the five pieces, each on its own stage contract
            for piece in noop_pieces:
                if piece.endswith(".h"):
                    stage = piece.split("/")[2]
                    graph[piece[4:]].add("pipeline/%s/contract.h" % stage)
                else:
                    graph[piece[4:]].add(piece[4:-4] + ".h")
            graph["pipeline/world_estimation/estimators/noop/noop.h"].add(
                "pipeline/world_estimation/contract.h")
            graph["pipeline/world_estimation/estimators/noop/noop.cpp"].add(
                "pipeline/world_estimation/estimators/noop/noop.h")
            continue
        with open(os.path.join(NAV, f), encoding="utf-8", errors="replace") as fh:
            for ln, text in enumerate(fh, 1):
                m = inc_re.match(text)
                if not m:
                    continue
                inc = m.group(1)
                total += 1
                if inc == "impl/noop/noops.h":
                    rewritten += 1
                    continue   # noop split: each includer takes its stage piece
                if inc in old_inc:
                    new_inc = old_inc[inc]
                    if new_inc != inc:
                        rewritten += 1
                    if new_f.startswith("src/"):
                        graph[new_f[4:]].add(new_inc)
                    if new_inc not in after_inc:
                        unresolved.append("%s:%d %s" % (f, ln, inc))
                    continue
                # not a src path: generated, vendored, translaGATR, or local to tests/tools
                local = os.path.join(NAV, os.path.dirname(f), inc)
                brain = ("actugatr/", "communigatr/", "investigatr/", "sim/")
                ok = (os.path.exists(local) or inc.startswith(("translaGATR/", "tinyxml2/")) or
                      (f == "tests/brain_link_e2e_gtest.cpp" and inc.startswith(brain)) or
                      inc in ("navigatr_build_config.h", "apriltag.h") or
                      os.path.exists(os.path.join(NAV, "third_party", inc)) or
                      os.path.exists(os.path.join(NAV, "third_party", "stb", inc)) or
                      os.path.exists(os.path.join(NAV, "third_party", "apriltag", inc)))
                if not ok:
                    unresolved.append("%s:%d %s (not a src path)" % (f, ln, inc))
    say()
    say("quoted includes scanned: %d, rewritten by the map: %d" % (total, rewritten))
    for u in unresolved:
        errors.append("include does not resolve after the move: " + u)

    # cycles (Tarjan) on the post-move src graph
    index, low, stack, on, sccs = {}, {}, [], set(), []
    counter = [0]

    def strong(v):
        index[v] = low[v] = counter[0]
        counter[0] += 1
        stack.append(v)
        on.add(v)
        for w in graph.get(v, ()):
            if w not in index:
                strong(w)
                low[v] = min(low[v], low[w])
            elif w in on:
                low[v] = min(low[v], index[w])
        if low[v] == index[v]:
            comp = []
            while True:
                w = stack.pop()
                on.discard(w)
                comp.append(w)
                if w == v:
                    break
            if len(comp) > 1:
                sccs.append(sorted(comp))

    sys.setrecursionlimit(10000)
    for v in list(graph):
        if v not in index:
            strong(v)
    say("file-level include cycles: %d" % len(sccs))
    for c in sccs:
        errors.append("include cycle: " + " ".join(c))

    # layering edges
    def stage_of(p):
        if p.startswith("pipeline/"):
            parts = p.split("/")
            if parts[1] in STAGES:
                return parts[1]
        return None

    def area(p):
        s = stage_of(p)
        if s:
            return "pipeline/" + s
        parts = p.split("/")
        if parts[0] == "pipeline" and len(parts) > 2:
            return "pipeline/" + parts[1]
        return parts[0] if len(parts) > 1 else "(root)"

    def is_plugin(p):
        return any(("src/" + p).startswith(k) for k in PLUGIN_ELEMENT)

    say()
    say("layering edges after the move (includer -> included):")
    groups = defaultdict(list)
    for a, bs in sorted(graph.items()):
        for b in sorted(bs):
            sa, sb = stage_of(a), stage_of(b)
            if a.startswith("pipeline/framework/") and sb:
                groups["framework -> stage"].append((a, b))
            if sa and sb and STAGES.index(sb) > STAGES.index(sa):
                groups["earlier stage -> later stage"].append((a, b))
            if sa and sb == sa and not is_plugin(a) and is_plugin(b):
                groups["stage level -> own plug-in folder"].append((a, b))
            if sa and sb and sb != sa and is_plugin(b):
                groups["other stage -> plug-in folder"].append((a, b))
            if a.startswith(("math/", "config/", "time/", "io/")) and b.startswith("pipeline/"):
                groups["math/config/time/io -> pipeline"].append((a, b))
            if a.startswith("pipeline/") and b.startswith(("core/", "cli/")):
                groups["pipeline -> core/cli"].append((a, b))
            if a.startswith("pipeline/") and b.startswith("brain_profile/"):
                groups["pipeline -> brain_profile"].append((a, b))
            if a.startswith("pipeline/") and b.startswith("debug/"):
                groups["pipeline -> debug"].append((a, b))
            if a.startswith("debug/") and b.startswith("core/"):
                groups["debug -> core"].append((a, b))
            if a.startswith("debug/") and is_plugin(b):
                groups["debug -> plug-in folder"].append((a, b))
            if a.startswith("brain_profile/") and b.startswith("core/"):
                groups["brain_profile -> core"].append((a, b))
    for k in sorted(groups):
        say("  %s: %d" % (k, len(groups[k])))
        for a, b in groups[k]:
            say("    %s -> %s" % (a, b))

    say()
    say("errors: %d" % len(errors))
    for e in errors:
        say("  " + e)
    text = "\n".join(lines) + "\n"
    if out_path:
        with open(out_path, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(text)
    sys.stdout.write(text)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
