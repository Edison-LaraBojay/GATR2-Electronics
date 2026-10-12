# naviGATR restructure step 1: file move per plan/move_map_final.tsv, then the noop split.
#
#   py -3 move.py [--root <repo root>] [--report <file>] [--trial]
#
# - git mv every mapped file
# - include paths in pi/naviGATR/{src,tests,tools}
# - CMake lists, embed_assets.cmake, field_refs target
# - line-1 labels of renamed files, path comments, doc links
# - noop split: one noop per stage, bodies moved verbatim
#
# Refuses to run on a tree that is not clean or no longer has the old layout.
# --trial: scratch repo (index only, no HEAD check).

import argparse
import glob
import os
import re
import subprocess
import sys

R = "C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/restructure"
WT = ("C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/"
      "827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/wt-navi")
TSV = R + "/plan/move_map_final.tsv"
BASE = "ae457eee4d038dc9aa28d036571316f4466e1e8a"
NAV = "pi/naviGATR"

CODE_EXT = (".h", ".hpp", ".cpp", ".c", ".in")
CODE_DIRS = ("src", "tests", "tools")
CMAKE_FILES = ["pi/naviGATR/CMakeLists.txt", "pi/naviGATR/tests/CMakeLists.txt",
               "pi/naviGATR/tools/perf/CMakeLists.txt", "pi/naviGATR/cmake/*.cmake"]
DOC_FILES = ["docs/*.md", "README.md", "pi/naviGATR/README.md", "pi/naviGATR/docs/*.md",
             "pi/naviGATR/third_party/README.md"]
OLD_TOP = ["app", "contracts", "payloads", "state", "runtime", "impl", "resources", "transport",
           "capture", "diagnostics", "inspection"]
CORE_AFTER = {"system.h", "system.cpp", "handoff.h"}
SKIP_SCAN = ("pi/naviGATR/third_party/apriltag/", "pi/naviGATR/third_party/three/",
             "pi/naviGATR/third_party/tinyxml2/", "pi/naviGATR/third_party/stb/")

# noop split: stage, piece folder (src-relative), factory, class, label
NOOP_WE = "pipeline/world_estimation/estimators/noop"
NOOP_PIECES = [
    ("commands", "pipeline/commands/collectors", "makeNoopCommands", "NoopCommands",
     "CommandCollection noop: carries the previous command state forward"),
    ("localization", "pipeline/localization/estimators", "makeNoopStateEstimator",
     "NoopStateEstimator", "Estimator noop: preserves the previous robot state"),
    ("world_estimation", NOOP_WE, "makeNoopFieldEstimation", "NoopFieldEstimation",
     "Estimator noop: preserves the previous field, publishes no evidence"),
    ("targets", "pipeline/targets/resolvers", "makeNoopTargetResolution",
     "NoopTargetResolution", "TargetResolution noop: preserves the previous target state"),
    ("publishing", "pipeline/publishing/publishers", "makeNoopPublishing", "NoopPublishing",
     "Publishing noop: publishes nothing, successfully"),
]
NOOP_USERS = {
    "pipeline/commands/collectors/register_commands.cpp": "commands",
    "pipeline/localization/register_localization.cpp": "localization",
    "pipeline/world_estimation/estimators/register_world_estimation.cpp": "world_estimation",
    "pipeline/targets/resolvers/register_target_resolution.cpp": "targets",
    "pipeline/publishing/publishers/register_publishers.cpp": "publishing",
}
NOOP_DROP = "tests/system_build_gtest.cpp"

INC_RE = re.compile(r'^(\s*#\s*include\s+")([^"]+)(")', re.M)
LINK_RE = re.compile(r"\[([^\]]+)\]\(([^)\s]+)\)")


def die(msg):
    print("move.py: " + msg, file=sys.stderr)
    sys.exit(2)


def read(p):
    with open(p, encoding="utf-8", newline="") as f:
        return f.read()


def write(p, s):
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, "w", encoding="utf-8", newline="") as f:
        f.write(s)


def git(root, *args):
    r = subprocess.run(["git", "-C", root] + list(args), capture_output=True, text=True)
    if r.returncode:
        die("git %s failed: %s" % (" ".join(args), r.stderr.strip()))
    return r.stdout


def load_rows():
    rows = []
    for raw in read(TSV).splitlines():
        if raw.strip():
            o, n = raw.split("\t")
            rows.append((o, o if n == "same" else n))
    return rows


def expand(root, patterns):
    out = []
    for p in patterns:
        out += sorted(os.path.relpath(f, root).replace("\\", "/")
                      for f in glob.glob(os.path.join(root, p)))
    return out


def code_files(root):
    out = []
    for top in CODE_DIRS:
        for d, _, files in os.walk(os.path.join(root, NAV, top)):
            for fn in files:
                if fn.endswith(CODE_EXT):
                    out.append(os.path.relpath(os.path.join(d, fn), root).replace("\\", "/"))
    return sorted(out)


class PathRewriter:
    # old src-relative paths (and .h/.cpp/.c stems) in free text -> new paths
    def __init__(self, moves):
        self.full = {o[4:]: n for o, n in moves}
        stems = {}
        for o, n in moves:
            so, eo = os.path.splitext(o[4:])
            sn, en = os.path.splitext(n)
            if eo in (".h", ".cpp", ".c") and eo == en:
                stems.setdefault(so, set()).add(sn)
        self.stem = {k: v.pop() for k, v in stems.items() if len(v) == 1 and k not in self.full}
        keys = sorted(list(self.full) + list(self.stem), key=len, reverse=True)
        self.re = re.compile(r"(?<![\w./-])((?:\.\./)*(?:pi/naviGATR/)?src/)?(" +
                             "|".join(re.escape(k) for k in keys) + r")(?!\w)")

    def _repl(self, m):
        prefix, key = m.group(1) or "", m.group(2)
        if key in self.stem:
            if re.match(r"\.\w", m.string[m.end():]):
                return m.group(0)
            new = self.stem[key]
        else:
            new = self.full[key]
        if prefix:
            return prefix[:-len("src/")] + new
        return new[len("src/"):] if new.startswith("src/") else new

    def sub(self, line):
        return self.re.sub(self._repl, line)

    def find(self, line):
        out = []
        for m in self.re.finditer(line):
            if m.group(2) in self.stem and re.match(r"\.\w", line[m.end():]):
                continue
            out.append(m.group(0))
        return out


def include_candidates(root, f, inc):
    nav = os.path.join(root, NAV)
    return [os.path.join(root, os.path.dirname(f), inc), os.path.join(nav, "src", inc),
            os.path.join(root, inc), os.path.join(nav, "third_party", inc),
            os.path.join(nav, "third_party", "stb", inc),
            os.path.join(nav, "third_party", "apriltag", inc)]


def unresolved_includes(root):
    out = set()
    for f in code_files(root):
        for m in INC_RE.finditer(read(os.path.join(root, f))):
            if not any(os.path.isfile(c) for c in include_candidates(root, f, m.group(2))):
                out.add((f, m.group(2)))
    return out


def scan_old_paths(root, rw):
    files = git(root, "ls-files").split("\n") + git(
        root, "ls-files", "--others", "--exclude-standard").split("\n")
    hits = []
    for f in files:
        if not f or f.startswith(SKIP_SCAN):
            continue
        p = os.path.join(root, f)
        if not os.path.isfile(p):
            continue
        try:
            s = read(p)
        except UnicodeDecodeError:
            continue
        for i, line in enumerate(s.split("\n"), 1):
            for h in rw.find(line):
                hits.append("%s:%d: %s" % (f, i, h))
    return hits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=WT)
    ap.add_argument("--report", default=R + "/logs/e1.move.report.txt")
    ap.add_argument("--trial", action="store_true")
    a = ap.parse_args()
    root = a.root
    nav = os.path.join(root, NAV)
    rep = []

    def log(s=""):
        rep.append(s)

    rows = load_rows()
    moves = [(o, n) for o, n in rows if o != n]

    # preconditions: clean tree, old layout present, no destination yet
    if not a.trial:
        head = git(root, "rev-parse", "HEAD").strip()
        if head != BASE:
            die("refusing: HEAD is %s, expected %s" % (head, BASE))
        if git(root, "status", "--porcelain").strip():
            die("refusing: tree not clean (already run?)")
    if not os.path.isfile(os.path.join(nav, "src/impl/noop/noops.h")):
        die("refusing: src/impl/noop/noops.h missing (already run?)")
    for o, n in rows:
        if not os.path.isfile(os.path.join(nav, o)):
            die("refusing: missing source " + o)
    for o, n in moves:
        if os.path.exists(os.path.join(nav, n)):
            die("refusing: destination exists " + n)
    dests = [n for _, n in moves]
    if len(dests) != len(set(dests)):
        die("duplicate destination in TSV")
    new_src = {n[4:] for n in dests if n.startswith("src/")}
    if new_src & {o[4:] for o, _ in moves}:
        die("an old path equals a new path")
    for f in (code_files(root) + expand(root, CMAKE_FILES) + expand(root, DOC_FILES)):
        read(os.path.join(root, f))
    unresolved_before = unresolved_includes(root)
    rw = PathRewriter(moves)
    rename_nav = {NAV + "/" + o: NAV + "/" + n for o, n in moves}

    # 1. git mv
    for o, n in moves:
        os.makedirs(os.path.dirname(os.path.join(nav, n)), exist_ok=True)
        git(root, "mv", "--", NAV + "/" + o, NAV + "/" + n)
    removed = []
    for o, _ in moves:
        d = os.path.dirname(os.path.join(nav, o))
        while os.path.isdir(d) and not os.listdir(d):
            os.rmdir(d)
            removed.append(os.path.relpath(d, nav).replace("\\", "/"))
            d = os.path.dirname(d)
    log("git mv: %d files (%d stay)" % (len(moves), len(rows) - len(moves)))
    log("empty folders removed: " + " ".join(sorted(set(removed))))

    # 2. include paths, in place, no re-sort
    inc_map = {o[4:]: n[4:] for o, n in moves if n.startswith("src/")}
    inc_changed = {}
    for f in code_files(root):
        p = os.path.join(root, f)
        s = read(p)
        count = [0]

        def sub(m):
            new = inc_map.get(m.group(2))
            if new is None:
                return m.group(0)
            count[0] += 1
            return m.group(1) + new + m.group(3)

        t = INC_RE.sub(sub, s)
        if t != s:
            write(p, t)
            inc_changed[f] = count[0]
    by_top = {}
    for f, c in inc_changed.items():
        top = f[len(NAV) + 1:].split("/")[0]
        by_top.setdefault(top, [0, 0])
        by_top[top][0] += 1
        by_top[top][1] += c
    log()
    log("include lines rewritten: %d in %d files" % (sum(inc_changed.values()), len(inc_changed)))
    for top in sorted(by_top):
        log("  %s: %d files, %d lines" % (top, by_top[top][0], by_top[top][1]))

    # 3. line-1 labels: renamed basenames and the payload headers use the src-relative path
    log()
    log("line-1 labels:")
    labels = 0
    for o, n in moves:
        orel, nrel = o[4:], n[4:]
        if orel.startswith("impl/noop/"):
            continue
        if not (orel.startswith("payloads/") or os.path.basename(orel) != os.path.basename(nrel)):
            continue
        p = os.path.join(nav, n)
        s = read(p)
        first, rest = s.split("\n", 1)
        want = ("// " + orel) if orel.startswith("payloads/") else ("// " + os.path.basename(orel))
        if first != want:
            die("unexpected line 1 in %s: %r" % (n, first))
        write(p, "// " + nrel + "\n" + rest)
        labels += 1
        log("  %s: %s -> // %s" % (n, first, nrel))
    log("  total: %d" % labels)

    # 4. old paths in comments, CMake and docs
    def rewrite_lines(f, pick):
        p = os.path.join(root, f)
        s = read(p)
        lines = s.split("\n")
        changes = []
        for i, line in enumerate(lines):
            if not pick(line):
                continue
            t = rw.sub(line)
            if f.endswith(".md") and t != line:
                t = fix_link_text(t)
            if t != line:
                lines[i] = t
                changes.append((i + 1, line, t))
        if changes:
            write(p, "\n".join(lines))
        return changes

    def fix_link_text(line):
        def repl(m):
            text, target = m.group(1), m.group(2)
            for o, n in moves:
                ob, nb = os.path.basename(o), os.path.basename(n)
                if ob != nb and text in (ob, "`" + ob + "`") and target.endswith("/" + n):
                    parent = os.path.basename(os.path.dirname(n))
                    return "[%s](%s)" % (text.replace(ob, parent + "/" + nb), target)
            return m.group(0)
        return LINK_RE.sub(repl, line)

    def comment_line(line):
        return line.lstrip().startswith("//")

    for title, files, pick in (
            ("code comments", code_files(root), comment_line),
            ("CMake", expand(root, CMAKE_FILES), lambda line: True),
            ("docs", expand(root, DOC_FILES), lambda line: True)):
        total = 0
        log()
        log(title + ":")
        for f in files:
            for ln, old, new in rewrite_lines(f, pick):
                total += 1
                log("  %s:%d" % (f, ln))
                log("    - " + old.strip())
                log("    + " + new.strip())
        log("  lines changed: %d" % total)

    # 5. noop split
    src = os.path.join(nav, "src")
    h_all = read(os.path.join(src, NOOP_WE, "noop.h"))
    c_all = read(os.path.join(src, NOOP_WE, "noop.cpp"))
    flags = re.S | re.M
    classes = {name: body for body, name in re.findall(
        r"^(class (Noop\w+) : public \w+\n\{.*?\n\};\n)", c_all, flags)}
    factories = {name: body for body, name in re.findall(
        r"^(std::unique_ptr<\w+> (makeNoop\w+)\(.*?\n\}\n)", c_all, flags)}
    decls = {name: body for body, name in re.findall(
        r"^(std::unique_ptr<\w+>\s+(makeNoop\w+)\(.*?\);\n)", h_all, flags)}
    if not (len(classes) == len(factories) == len(decls) == 5):
        die("noop split: found %d classes, %d factories, %d declarations" % (
            len(classes), len(factories), len(decls)))

    # everything outside the moved pieces is boilerplate the pieces re-emit
    def leftover(text, pieces):
        for piece in pieces:
            text = text.replace(piece, "", 1)
        return [ln for ln in text.split("\n") if ln.strip()]

    c_left = leftover(c_all, list(classes.values()) + list(factories.values()))
    c_expect = ["// noops.cpp", '#include "%s/noop.h"' % NOOP_WE, "namespace navigatr", "{",
                "namespace", "{", "} // namespace", "} // namespace navigatr"]
    if c_left != c_expect:
        die("noop split: unexpected text in noop.cpp: %r" % c_left)
    h_left = leftover(h_all, list(decls.values()))
    h_code = [ln for ln in h_left if not ln.startswith("//")]
    h_expect = ["#pragma once", "#include <memory>", "#include <string>",
                '#include "pipeline/commands/contract.h"',
                '#include "pipeline/world_estimation/contract.h"',
                '#include "pipeline/localization/contract.h"',
                '#include "pipeline/publishing/contract.h"',
                '#include "pipeline/targets/contract.h"', "namespace navigatr", "{",
                "} // namespace navigatr"]
    if h_code != h_expect:
        die("noop split: unexpected code in noop.h: %r" % h_code)

    log()
    log("noop split (%s/noop.{h,cpp} -> 5 pieces):" % NOOP_WE)
    header_comment = [ln for ln in h_left if ln.startswith("//")]
    log("  noops.h header comment, not carried over (each piece gets its own line):")
    for ln in header_comment:
        log("    " + ln)
    cmake_lines = []
    for stage, folder, fac, cls, label in NOOP_PIECES:
        h = ("// %s/noop.h\n// %s\n\n#pragma once\n#include <memory>\n#include <string>\n\n"
             '#include "pipeline/%s/contract.h"\n\nnamespace navigatr\n{\n\n%s\n'
             "} // namespace navigatr\n" % (folder, label, stage, decls[fac]))
        c = ('// %s/noop.cpp\n\n#include "%s/noop.h"\n\nnamespace navigatr\n{\n\n'
             "namespace\n{\n\n%s\n} // namespace\n\n%s\n} // namespace navigatr\n" % (
                 folder, folder, classes[cls], factories[fac]))
        write(os.path.join(src, folder, "noop.h"), h)
        write(os.path.join(src, folder, "noop.cpp"), c)
        cmake_lines.append("    src/%s/noop.cpp\n" % folder)
        log("  %s: src/%s/noop.{h,cpp} (%s, %s)" % (stage, folder, fac, cls))
    old_inc = '#include "%s/noop.h"\n' % NOOP_WE
    for user, stage in NOOP_USERS.items():
        p = os.path.join(src, user)
        s = read(p)
        if s.count(old_inc) != 1:
            die("noop split: %s does not include %s/noop.h once" % (user, NOOP_WE))
        folder = [pc[1] for pc in NOOP_PIECES if pc[0] == stage][0]
        new_inc = '#include "%s/noop.h"\n' % folder
        write(p, s.replace(old_inc, new_inc))
        log("  %s: %s" % (user, new_inc.strip()))
    p = os.path.join(nav, NOOP_DROP)
    s = read(p)
    if s.count(old_inc) != 1:
        die("noop split: %s does not include %s/noop.h once" % (NOOP_DROP, NOOP_WE))
    write(p, s.replace(old_inc, ""))
    log("  %s: include dropped (uses no noop symbol)" % NOOP_DROP)
    p = os.path.join(nav, "CMakeLists.txt")
    s = read(p)
    old_cm = "    src/%s/noop.cpp\n" % NOOP_WE
    if s.count(old_cm) != 1:
        die("noop split: CMakeLists.txt has no single noop.cpp line")
    write(p, s.replace(old_cm, "".join(cmake_lines)))
    log("  CMakeLists.txt: 1 noop.cpp line -> %d" % len(cmake_lines))

    # 6. checks
    log()
    log("checks:")
    errors = []
    for d in OLD_TOP:
        if os.path.exists(os.path.join(src, d)):
            errors.append("old folder still present: src/" + d)
    core = set(os.listdir(os.path.join(src, "core")))
    if core != CORE_AFTER:
        errors.append("src/core holds %s" % sorted(core))
    for d, subdirs, _ in os.walk(nav):
        if "impl" in subdirs:
            errors.append("folder named impl: " + os.path.join(d, "impl"))
    log("  old top-level folders gone: %s" % ", ".join(OLD_TOP))
    log("  src/core: " + ", ".join(sorted(core)))
    unresolved_after = unresolved_includes(root)
    before_mapped = {(rename_nav.get(f, f), inc) for f, inc in unresolved_before}
    new_unresolved = sorted(unresolved_after - before_mapped)
    log("  unresolved quoted includes before: %d, after: %d, new: %d" % (
        len(unresolved_before), len(unresolved_after), len(new_unresolved)))
    for f, inc in sorted(unresolved_after):
        log("    %s: %s" % (f, inc))
    errors += ["new unresolved include %s: %s" % x for x in new_unresolved]
    hits = scan_old_paths(root, rw)
    log("  old path mentions left in the tree: %d" % len(hits))
    for h in hits:
        log("    " + h)
    errors += ["old path left: " + h for h in hits]
    log("  errors: %d" % len(errors))
    for e in errors:
        log("    " + e)

    write(a.report, "\n".join(rep) + "\n")
    print("\n".join(rep))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
