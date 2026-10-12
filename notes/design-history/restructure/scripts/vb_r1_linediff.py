#!/usr/bin/env python3
# vb_r1_linediff.py
# Per renamed/modified code file: changed lines classified as include, comment, blank or code.
# Also checks that the include block order change never crosses a non-include line.
#
#   py -3 vb_r1_linediff.py <worktree> <old repo root> <out report>

import difflib
import re
import subprocess
import sys

CODE_EXT = (".h", ".hpp", ".cpp", ".c", ".in", ".txt", ".cmake")


def classify(line):
    s = line.strip()
    if s == "":
        return "blank"
    if re.match(r"#\s*include\b", s):
        return "include"
    if s.startswith("//") or s.startswith("/*") or s.startswith("*"):
        return "comment"
    return "code"


def main():
    wt, old_root, out = sys.argv[1:4]
    ns = subprocess.run(
        ["git", "-C", wt, "diff", "--cached", "-M", "--name-status", "ae457ee", "--", "pi/naviGATR"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.splitlines()
    rep = []
    totals = {"include": 0, "comment": 0, "blank": 0, "code": 0}
    for row in ns:
        parts = row.split("\t")
        st = parts[0]
        if st.startswith("R"):
            old, new = parts[1], parts[2]
        elif st == "M":
            old = new = parts[1]
        else:
            continue
        if not new.endswith(CODE_EXT):
            continue
        with open(old_root + "/" + old, encoding="utf-8") as fh:
            a = fh.read().split("\n")
        with open(wt + "/" + new, encoding="utf-8") as fh:
            b = fh.read().split("\n")
        if a == b:
            continue
        sm = difflib.SequenceMatcher(None, a, b, autojunk=False)
        code_lines = []
        counts = {"include": 0, "comment": 0, "blank": 0, "code": 0}
        for op, a1, a2, b1, b2 in sm.get_opcodes():
            if op == "equal":
                continue
            for l in a[a1:a2]:
                c = classify(l)
                counts[c] += 1
                if c == "code":
                    code_lines.append("  - %d: %s" % (a1 + 1, l))
            for l in b[b1:b2]:
                c = classify(l)
                counts[c] += 1
                if c == "code":
                    code_lines.append("  + %d: %s" % (b1 + 1, l))
        # include order: sequence of include lines, and whether a non-include line sits
        # between includes in a different position
        inc_a = [l.strip() for l in a if classify(l) == "include"]
        inc_b = [l.strip() for l in b if classify(l) == "include"]
        for k in totals:
            totals[k] += counts[k]
        rep.append(
            "%s %s -> %s  inc %d com %d blank %d CODE %d  includes %d->%d"
            % (st, old, new, counts["include"], counts["comment"], counts["blank"], counts["code"], len(inc_a), len(inc_b))
        )
        rep.extend(code_lines)
    rep.append("")
    rep.append("TOTAL changed lines: %s" % totals)
    with open(out, "w", encoding="utf-8") as fh:
        fh.write("\n".join(rep) + "\n")
    print("TOTAL changed lines: %s" % totals)


if __name__ == "__main__":
    main()
