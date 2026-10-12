# Trial of step 1 (move) and step 2a (noop split) on a scratch copy of the
# naviGATR tree. Never run it on the worktree.
#
#   py -3 apply_move_trial.py <scratch pi/naviGATR root> <move_map.tsv> [--no-noop-split]
#
# Moves files (os.rename), rewrites quoted includes in place (no re-sort),
# rewrites CMake/embed_assets paths, then splits impl/noop/noops.* into the
# five stage pieces.

import os
import re
import sys

NOOP_PIECES = {
    # stage folder: (plug-in folder, factory, class, header label)
    "commands": ("pipeline/commands/command_collections", "makeNoopCommands", "NoopCommands",
                 "CommandCollection noop: carries the previous command state forward"),
    "localization": ("pipeline/localization/estimators", "makeNoopStateEstimator",
                     "NoopStateEstimator", "Estimator noop: preserves the previous robot state"),
    "world_estimation": ("pipeline/world_estimation/estimators/noop", "makeNoopFieldEstimation",
                         "NoopFieldEstimation",
                         "Estimator noop: preserves the previous field, publishes no evidence"),
    "targets": ("pipeline/targets/target_resolutions", "makeNoopTargetResolution",
                "NoopTargetResolution",
                "TargetResolution noop: preserves the previous target state"),
    "publishing": ("pipeline/publishing/publishings", "makeNoopPublishing", "NoopPublishing",
                   "Publishing noop: publishes nothing, successfully"),
}
NOOP_INCLUDERS = {
    "register_commands.cpp": "commands",
    "register_localization.cpp": "localization",
    "register_world_estimation.cpp": "world_estimation",
    "register_target_resolution.cpp": "targets",
    "register_publishers.cpp": "publishing",
}


def read(p):
    with open(p, encoding="utf-8", newline="") as f:
        return f.read()


def write(p, s):
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, "w", encoding="utf-8", newline="") as f:
        f.write(s)


def main():
    nav = sys.argv[1]
    tsv = sys.argv[2]
    noop_split = "--no-noop-split" not in sys.argv
    rows = []
    for raw in read(tsv).splitlines():
        if raw.strip():
            o, n = raw.split("\t")
            rows.append((o, o if n == "same" else n))

    # 1. move
    for o, n in rows:
        if o == n:
            continue
        src = os.path.join(nav, o)
        dst = os.path.join(nav, n)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        os.rename(src, dst)
    for o, _ in rows:
        d = os.path.dirname(os.path.join(nav, o))
        while os.path.isdir(d) and not os.listdir(d):
            os.rmdir(d)
            d = os.path.dirname(d)

    inc_map = {o[4:]: n[4:] for o, n in rows if o.startswith("src/") and n.startswith("src/")}
    inc_re = re.compile(r'^(\s*#\s*include\s+")([^"]+)(")', re.M)

    # 2. includes, in place
    changed = 0
    for top in ("src", "tests", "tools"):
        for root, _, files in os.walk(os.path.join(nav, top)):
            for fn in files:
                if not fn.endswith((".h", ".cpp", ".c", ".in")):
                    continue
                p = os.path.join(root, fn)
                s = read(p)

                def sub(m):
                    nonlocal changed
                    new = inc_map.get(m.group(2), m.group(2))
                    if new != m.group(2):
                        changed += 1
                    return m.group(1) + new + m.group(3)

                t = inc_re.sub(sub, s)
                if t != s:
                    write(p, t)
    print("include lines rewritten:", changed)

    # 3. CMake and embed_assets
    cm = os.path.join(nav, "CMakeLists.txt")
    s = read(cm)
    for o, n in rows:
        if o != n:
            s = s.replace(o + "\n", n + "\n").replace(o + ")", n + ")").replace(o + " ", n + " ")
    write(cm, s)
    tc = os.path.join(nav, "tests", "CMakeLists.txt")
    write(tc, read(tc).replace("../src/app/command_line.cpp", "../src/cli/command_line.cpp"))
    ea = os.path.join(nav, "cmake", "embed_assets.cmake")
    write(ea, read(ea).replace('\\"inspection/viewer_assets.h\\"',
                               '\\"debug/inspection/viewer_assets.h\\"'))

    if not noop_split:
        return

    # 4. noop split (step 2a)
    we = NOOP_PIECES["world_estimation"][0]
    h_all = read(os.path.join(nav, "src", we, "noop.h"))
    c_all = read(os.path.join(nav, "src", we, "noop.cpp"))
    flags = re.S | re.M
    classes = {name: body for body, name in re.findall(
        r"^(class (Noop\w+) : public \w+\n\{.*?\n\};\n)", c_all, flags)}
    factories = {name: body for body, name in re.findall(
        r"^(std::unique_ptr<\w+> (makeNoop\w+)\(.*?\n\}\n)", c_all, flags)}
    decls = {name: body for body, name in re.findall(
        r"^(std::unique_ptr<\w+>\s+(makeNoop\w+)\(.*?\);\n)", h_all, flags)}
    assert len(classes) == 5 and len(factories) == 5 and len(decls) == 5, (
        len(classes), len(factories), len(decls))
    cmake_lines = []
    for stage, (folder, fac, cls, label) in NOOP_PIECES.items():
        h = ("// %s/noop.h\n// %s\n\n#pragma once\n#include <memory>\n#include <string>\n\n"
             "#include \"pipeline/%s/contract.h\"\n\nnamespace navigatr\n{\n\n%s\n"
             "} // namespace navigatr\n" % (folder, label, stage, decls[fac]))
        c = ("// %s/noop.cpp\n\n#include \"%s/noop.h\"\n\nnamespace navigatr\n{\n\n"
             "namespace\n{\n\n%s\n} // namespace\n\n%s\n} // namespace navigatr\n" % (
                 folder, folder, classes[cls], factories[fac]))
        write(os.path.join(nav, "src", folder, "noop.h"), h)
        write(os.path.join(nav, "src", folder, "noop.cpp"), c)
        cmake_lines.append("    src/%s/noop.cpp\n" % folder)
    old_noop_inc = "pipeline/world_estimation/estimators/noop/noop.h"
    for root, _, files in os.walk(os.path.join(nav, "src")):
        for fn in files:
            if fn in NOOP_INCLUDERS:
                p = os.path.join(root, fn)
                folder = NOOP_PIECES[NOOP_INCLUDERS[fn]][0]
                s = read(p)
                assert old_noop_inc in s, p
                write(p, s.replace(old_noop_inc, folder + "/noop.h"))
    sb = os.path.join(nav, "tests", "system_build_gtest.cpp")
    s = read(sb)
    assert '#include "%s"\n' % old_noop_inc in s
    write(sb, s.replace('#include "%s"\n' % old_noop_inc, ""))
    s = read(cm)
    old = "    src/%s/noop.cpp\n" % we
    assert old in s
    write(cm, s.replace(old, "".join(cmake_lines)))
    print("noop split: 5 pieces")


if __name__ == "__main__":
    main()
