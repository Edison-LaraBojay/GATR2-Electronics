#!/usr/bin/env python3
"""clang-format the lines the System split authored, never the moved text
(FINAL.md C7). Moved = any unit whose text matches a unit of the old
system.cpp/system.h (verify_split.parse). New files: every other line.
core/system.h (edited in place): only the added include and label lines.
A range whose formatting would change a line outside it is skipped.

Usage:
  py -3 fmt_split.py --orig <dir with old system.cpp, system.h> --src <src>
      --cf <clang-format> [--dry-run]
"""

import argparse
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import verify_split as V  # noqa: E402


def runs(nums):
    out = []
    for n in sorted(nums):
        if out and n == out[-1][1] + 1:
            out[-1][1] = n
        else:
            out.append([n, n])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--orig", required=True)
    ap.add_argument("--src", required=True)
    ap.add_argument("--cf", required=True)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    keys = set()
    for u in V.parse(os.path.join(a.orig, "system.cpp"), "old")[0]:
        keys.add(V.squash(u.text))
    old_h = os.path.join(a.orig, "system.h")
    for u in V.parse(old_h, "old")[0] + V.parse(old_h, "old", into_class="System")[0]:
        keys.add(V.squash(u.text))
    with open(old_h, encoding="utf-8") as f:
        old_h_lines = set(f.read().split("\n"))

    for rel in V.NEW_CPP + V.NEW_H + [V.SYSTEM_H]:
        path = os.path.join(a.src, rel)
        units, _, lines = V.parse(path, rel)
        if rel == V.SYSTEM_H:
            units += V.parse(path, rel, into_class="System")[0]
            authored = set()
            for u in units:
                if u.kind == "comment" and V.squash(u.text) not in keys and u.scope:
                    authored.update(range(u.first, u.last + 1))
            for i, l in enumerate(lines, 1):
                if l.startswith("#include ") and l not in old_h_lines:
                    authored.add(i)
        else:
            moved = set()
            for u in units:
                if V.squash(u.text) in keys:
                    moved.update(range(u.first, u.last + 1))
            authored = set(range(1, len(lines) + 1)) - moved
        rs = runs(authored)
        # a range whose formatting spills onto moved lines is skipped
        keep, skip = [], []
        for r in rs:
            got = fmt(a.cf, path, [r])
            outside = [i for i, (x, y) in enumerate(zip(lines, got), 1)
                       if x != y and not r[0] <= i <= r[1]]
            if len(got) != len(lines) and not outside:
                outside = ["line count"]
            (skip if outside else keep).append(r)
        print("%-40s %4d lines, %3d authored, ranges %s" % (
            rel, len(lines), len(authored), " ".join("%d:%d" % tuple(r) for r in keep)))
        for r in skip:
            print("    skipped %d:%d (formatting it rewrites moved lines)" % tuple(r))
        if a.dry_run or not keep:
            continue
        cmd = [a.cf, "-i", "--style=file"] + ["--lines=%d:%d" % tuple(r) for r in keep] + [path]
        subprocess.run(cmd, check=True)


def fmt(cf, path, ranges):
    cmd = [cf, "--style=file"] + ["--lines=%d:%d" % tuple(r) for r in ranges] + [path]
    out = subprocess.run(cmd, check=True, capture_output=True).stdout.decode("utf-8")
    lines = out.split("\n")
    if lines and lines[-1] == "":
        lines.pop()
    return lines


if __name__ == "__main__":
    main()
