#!/usr/bin/env python3
# v1_md_links.py: check relative markdown links and backticked src paths resolve.
# Usage: py -3 v1_md_links.py <worktree>
import os
import re
import subprocess
import sys

wt = sys.argv[1]
files = subprocess.run(["git", "-C", wt, "ls-files", "--cached", "--others", "--exclude-standard", "*.md"],
                       capture_output=True, text=True, check=True).stdout.splitlines()
skip = ("pi/naviGATR/third_party/", "pcb/")
link_re = re.compile(r"\]\(([^)#\s]+)(#[^)]*)?\)")
code_re = re.compile(r"`((?:pi/naviGATR/)?src/[A-Za-z0-9_./-]+)`")
bad = 0
checked = 0
for f in files:
    if f.startswith(skip):
        continue
    base = os.path.dirname(os.path.join(wt, f))
    for i, line in enumerate(open(os.path.join(wt, f), encoding="utf-8", errors="replace"), 1):
        for m in link_re.finditer(line):
            target = m.group(1)
            if re.match(r"^[a-z]+:", target):
                continue
            checked += 1
            if not os.path.exists(os.path.normpath(os.path.join(base, target))):
                bad += 1
                print(f"LINK {f}:{i}: {target}")
        for m in code_re.finditer(line):
            target = m.group(1)
            cands = [os.path.join(wt, target), os.path.join(wt, "pi/naviGATR", target), os.path.join(base, target)]
            checked += 1
            if not any(os.path.exists(c.rstrip("/")) for c in cands):
                bad += 1
                print(f"CODE {f}:{i}: {target}")
print(f"md files: {len(files)}; refs checked: {checked}; unresolved: {bad}")
