#!/usr/bin/env python3
# v1_stale_paths.py: grep the worktree for old src paths (HEAD) that no longer exist.
# Usage: py -3 v1_stale_paths.py <worktree>
import os
import re
import subprocess
import sys

wt = sys.argv[1]
src_prefix = "pi/naviGATR/src/"

old = subprocess.run(["git", "-C", wt, "ls-tree", "-r", "--name-only", "HEAD", src_prefix],
                     capture_output=True, text=True, check=True).stdout.split()
old_rel = [p[len(src_prefix):] for p in old]
gone = [r for r in old_rel if not os.path.exists(os.path.join(wt, src_prefix, r))]

# Old basenames that no longer exist anywhere under src.
new_names = set()
for root, _, files in os.walk(os.path.join(wt, src_prefix)):
    new_names.update(files)
gone_names = sorted({os.path.basename(r) for r in gone} - new_names)

files = subprocess.run(["git", "-C", wt, "ls-files", "--cached", "--others", "--exclude-standard"],
                       capture_output=True, text=True, check=True).stdout.splitlines()
skip = ("pi/naviGATR/third_party/", "pcb/", "brain/", "pico/", "translaGATR/")
exts = (".h", ".hpp", ".c", ".cpp", ".txt", ".cmake", ".md", ".yaml", ".yml", ".py", ".sh",
        ".in", ".xml", ".js", ".html", ".json")

alts = "|".join(re.escape(r) for r in gone)
path_re = re.compile(r"(?:(?<![A-Za-z0-9_/])|(?<=src/))(" + alts + r")(?![A-Za-z0-9_])")
name_re = re.compile(r"(?<![A-Za-z0-9_/.])(" + "|".join(re.escape(n) for n in gone_names) + r")(?![A-Za-z0-9_])")

print(f"old src files: {len(old_rel)}; old rel paths gone: {len(gone)}; old basenames gone: {len(gone_names)}")
print("gone basenames:", " ".join(gone_names))
hits_path = 0
hits_name = 0
for f in files:
    if f.startswith(skip) or not f.endswith(exts) and os.path.basename(f) != "CMakeLists.txt":
        continue
    full = os.path.join(wt, f)
    if not os.path.isfile(full):
        continue
    try:
        text = open(full, encoding="utf-8", errors="replace").read()
    except OSError:
        continue
    for i, line in enumerate(text.splitlines(), 1):
        m = path_re.search(line)
        if m:
            hits_path += 1
            print(f"PATH {f}:{i}: {m.group(1)} :: {line.strip()[:160]}")
            continue
        m = name_re.search(line)
        if m:
            hits_name += 1
            print(f"NAME {f}:{i}: {m.group(1)} :: {line.strip()[:160]}")
print(f"path hits: {hits_path}; basename hits: {hits_name}")
