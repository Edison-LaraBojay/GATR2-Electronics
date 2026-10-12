# Step 1 proof: every translation unit preprocesses to the same token text
# before and after the move (paths inside __FILE__ strings mapped).
#
#   py -3 pp_compare.py <base build dir> <moved build dir> <move_map.tsv> <out dir>
#
# Both build dirs configured with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON.
# Generated sources (viewer_assets.cpp) are skipped.

import json
import os
import re
import shlex
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor


def norm(p):
    return os.path.normpath(p).replace("\\", "/")


def load(build):
    with open(os.path.join(build, "compile_commands.json"), encoding="utf-8") as f:
        return json.load(f)


def nav_root(entries):
    for e in entries:
        f = norm(e["file"])
        i = f.find("/pi/naviGATR/")
        if i >= 0:
            return f[:i + len("/pi/naviGATR/")], f[:i + 1]
    raise SystemExit("no naviGATR sources")


def pp(entry, out):
    args = shlex.split(entry["command"].replace("\\", "/"), posix=True)
    cleaned = []
    skip = False
    for a in args:
        if skip:
            skip = False
            continue
        if a == "-o":
            skip = True
            continue
        if a == "-c":
            continue
        cleaned.append(a)
    cleaned += ["-E", "-P", "-o", out]
    r = subprocess.run(cleaned, cwd=entry["directory"], capture_output=True, text=True)
    return r.returncode, r.stderr


def main():
    base_b, moved_b, tsv, out = sys.argv[1:5]
    os.makedirs(out, exist_ok=True)
    rows = []
    for raw in open(tsv, encoding="utf-8").read().splitlines():
        if raw.strip():
            o, n = raw.split("\t")
            rows.append((o, o if n == "same" else n))
    ren = dict(rows)
    base = load(base_b)
    moved = load(moved_b)
    base_nav, base_top = nav_root(base)
    moved_nav, moved_top = nav_root(moved)
    moved_by_rel = {}
    for e in moved:
        f = norm(e["file"])
        if f.startswith(moved_nav):
            moved_by_rel[f[len(moved_nav):]] = e
    pairs = []
    missing = []
    for e in base:
        f = norm(e["file"])
        if not f.startswith(base_nav):
            continue   # translaGATR, brain, vendored outside naviGATR
        rel = f[len(base_nav):]
        if rel.startswith("third_party/"):
            continue
        new_rel = ren.get(rel, rel)
        if new_rel not in moved_by_rel:
            missing.append(rel)
            continue
        pairs.append((rel, e, moved_by_rel[new_rel]))

    def job(i_pair):
        i, (rel, eb, em) = i_pair
        ob = os.path.join(out, "%03d.base.i" % i)
        om = os.path.join(out, "%03d.moved.i" % i)
        rb, sb = pp(eb, ob)
        rm, sm = pp(em, om)
        if rb or rm:
            return rel, "preprocess failed: " + (sb or sm)[:300]
        tb = open(ob, encoding="utf-8", errors="replace").read()
        tm = open(om, encoding="utf-8", errors="replace").read()
        tb = tb.replace(base_top, "<TOP>/")
        tm = tm.replace(moved_top, "<TOP>/")
        stamp = re.compile(r'"[A-Z][a-z]{2} [ 0-9]\d \d{4}"|"\d\d:\d\d:\d\d"')   # __DATE__, __TIME__
        tb = stamp.sub("<STAMP>", tb)
        tm = stamp.sub("<STAMP>", tm)
        for o, n in rows:
            if o != n:
                tb = tb.replace("<TOP>/pi/naviGATR/" + o + '"', "<TOP>/pi/naviGATR/" + n + '"')
        os.remove(ob)
        os.remove(om)
        return rel, "same" if tb == tm else "DIFFERENT"

    with ThreadPoolExecutor(max_workers=16) as ex:
        results = list(ex.map(job, enumerate(pairs)))
    diff = [r for r in results if r[1] != "same"]
    print("translation units compared: %d, identical: %d, different: %d, unmatched: %d" % (
        len(results), len(results) - len(diff), len(diff), len(missing)))
    for rel, why in diff:
        print("  %s: %s" % (rel, why))
    for rel in missing:
        print("  unmatched: " + rel)
    return 1 if diff or missing else 0


if __name__ == "__main__":
    sys.exit(main())
