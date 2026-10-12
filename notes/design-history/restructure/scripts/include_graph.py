# Include graph for pi/naviGATR/src: who includes each src file.
import os, re, sys, json
WT = r"C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/wt-navi"
NAV = os.path.join(WT, "pi", "naviGATR")
SRC = os.path.join(NAV, "src")
inc_re = re.compile(r'^\s*#\s*include\s*"([^"]+)"')
src_files = []
for root, _, files in os.walk(SRC):
    for f in files:
        src_files.append(os.path.relpath(os.path.join(root, f), SRC).replace("\\", "/"))
src_set = set(src_files)
# scan every C/C++ file in the worktree
code_ext = (".h", ".hpp", ".c", ".cc", ".cpp", ".in")
includers = {s: [] for s in src_files}
unresolved = []
for root, dirs, files in os.walk(WT):
    if ".git" in dirs:
        dirs.remove(".git")
    for f in files:
        if not f.endswith(code_ext):
            continue
        p = os.path.join(root, f)
        rel = os.path.relpath(p, WT).replace("\\", "/")
        try:
            lines = open(p, encoding="utf-8", errors="replace").read().splitlines()
        except Exception:
            continue
        for ln in lines:
            m = inc_re.match(ln)
            if not m:
                continue
            inc = m.group(1)
            # resolve: relative to src/ first, then relative to the file dir
            target = None
            if inc in src_set:
                target = inc
            else:
                cand = os.path.normpath(os.path.join(os.path.dirname(p), inc))
                if cand.startswith(os.path.normpath(SRC)):
                    r = os.path.relpath(cand, SRC).replace("\\", "/")
                    if r in src_set:
                        target = r
            if target:
                includers[target].append(rel)
            elif rel.startswith("pi/naviGATR/src/"):
                unresolved.append((rel, inc))
json.dump({"includers": includers, "unresolved": unresolved}, open(sys.argv[1], "w"), indent=1)
for s in sorted(src_files):
    who = includers[s]
    print(f"{s}\t{len(who)}\t" + " ".join(sorted(set(w.replace('pi/naviGATR/', '') for w in who))))
