# Preprocessed diff of chosen translation units, base vs moved (normalized as in pp_compare.py).
#
#   py -3 pp_diff.py <base build> <moved build> <move_map.tsv> <out dir> <base rel path>...

import difflib
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pp_compare as pc  # noqa: E402


def main():
    base_b, moved_b, tsv, out = sys.argv[1:5]
    wanted = sys.argv[5:]
    os.makedirs(out, exist_ok=True)
    rows = []
    for raw in open(tsv, encoding="utf-8").read().splitlines():
        if raw.strip():
            o, n = raw.split("\t")
            rows.append((o, o if n == "same" else n))
    ren = dict(rows)
    base, moved = pc.load(base_b), pc.load(moved_b)
    base_nav, base_top = pc.nav_root(base)
    moved_nav, moved_top = pc.nav_root(moved)
    by_rel = {pc.norm(e["file"])[len(moved_nav):]: e for e in moved
              if pc.norm(e["file"]).startswith(moved_nav)}
    stamp = re.compile(r'"[A-Z][a-z]{2} [ 0-9]\d \d{4}"|"\d\d:\d\d:\d\d"')
    for e in base:
        rel = pc.norm(e["file"])[len(base_nav):]
        if rel not in wanted:
            continue
        ob, om = os.path.join(out, "b.i"), os.path.join(out, "m.i")
        pc.pp(e, ob)
        pc.pp(by_rel[ren.get(rel, rel)], om)
        tb = stamp.sub("<STAMP>", open(ob, encoding="utf-8").read().replace(base_top, "<TOP>/"))
        tm = stamp.sub("<STAMP>", open(om, encoding="utf-8").read().replace(moved_top, "<TOP>/"))
        for o, n in rows:
            if o != n:
                tb = tb.replace("<TOP>/pi/naviGATR/" + o + '"', "<TOP>/pi/naviGATR/" + n + '"')
        d = [ln for ln in difflib.unified_diff(tb.split("\n"), tm.split("\n"), lineterm="", n=0)
             if not ln.startswith(("---", "+++"))]
        added = [ln for ln in d if ln.startswith("+")]
        removed = [ln for ln in d if ln.startswith("-")]
        print("=== %s -> %s: +%d -%d" % (rel, ren.get(rel, rel), len(added), len(removed)))
        for ln in d:
            print(ln)
        os.remove(ob)
        os.remove(om)


if __name__ == "__main__":
    main()
