#!/usr/bin/env python3
# e6_apply.py
# Step 6 rewrite of pi/naviGATR/src: nested namespaces per folder, using-declarations
# for cross-namespace names, forward declarations moved to their home namespace,
# payload_names qualified where the nearest payload_names is not the home one.
# Skips main.cpp and pipeline/register_all.* (edited by hand) and files without
# a navigatr block.
#
#   py -3 e6_apply.py <naviGATR dir> <report.txt>

import collections
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import e6_ns  # noqa: E402


def qual(target, own):
    """Qualifier for target namespace written inside own."""
    tp = target.split("::")
    op = own.split("::") if own else []
    k = 0
    while k < len(tp) - 1 and k < len(op) and tp[k] == op[k]:
        k += 1
    return "::".join(tp[k:])


def open_lines(ns):
    out = []
    for part in ns.split("::"):
        out += ["namespace " + part, "{"]
    return out


def close_lines(ns):
    return ["} // namespace " + part for part in reversed(ns.split("::"))]


def payload_home(index, name):
    homes = {e[0] for e in index.get(name, []) if "payload_names" in e[0].split("::")}
    if len(homes) != 1:
        return None
    return next(iter(homes))[: -len("::payload_names")]


def main():
    navi, report = sys.argv[1:3]
    index_from = navi
    only = None
    for a in sys.argv[3:]:
        if a.startswith("--index-from="):
            index_from = a.split("=", 1)[1]
        elif a.startswith("--only="):
            only = set(a.split("=", 1)[1].split(","))
    src_files, info, index, nested_ns, needs = e6_ns.compute(index_from)
    fwd_moves = e6_ns.compute.fwd_moves
    pn_owners = {h[: -len("::payload_names")] for h in nested_ns.get("payload_names", set())}
    rep = []
    total_using = 0
    for rel in src_files:
        if only is not None and rel not in only:
            continue
        if rel in e6_ns.ROOT_FILES:
            rep.append("SKIP (hand) %s" % rel)
            continue
        path = os.path.join(navi, "src", rel)
        with open(path, encoding="utf-8", newline="") as fh:
            text = fh.read()
        if "\r" in text:
            raise SystemExit("CR in " + rel)
        lines = text.split("\n")
        if not any(l == "namespace navigatr" for l in lines):
            rep.append("SKIP (no navigatr) %s" % rel)
            continue
        own = e6_ns.ns_of(rel)
        # 1. drop moved forward declarations
        moves = fwd_moves.get(rel, [])
        drop = set()
        for home, name, line in moves:
            idx = line - 1
            if lines[idx].strip() not in ("class %s;" % name, "struct %s;" % name):
                raise SystemExit("fwd mismatch %s:%d %r" % (rel, line, lines[idx]))
            drop.add(idx)
        fwd_text = {}
        for home, name, line in moves:
            fwd_text.setdefault(home, []).append(lines[line - 1].strip())
        new = []
        for i, l in enumerate(lines):
            if i in drop:
                # collapse a blank left behind
                continue
            new.append(l)
        # collapse runs of blank lines created by the drop
        lines = []
        for l in new:
            if l == "" and lines and lines[-1] == "" :
                continue
            lines.append(l)
        # 2. payload_names qualification
        pn_changes = 0
        for i, l in enumerate(lines):
            def repl(m):
                nonlocal pn_changes
                pre, name = m.group(1), m.group(2)
                if pre:   # already qualified
                    return m.group(0)
                home = payload_home(index, name)
                if home is None or home == own:
                    return m.group(0)
                # nearest payload_names seen from own
                parts = own.split("::")
                nearest = None
                for k in range(len(parts), 0, -1):
                    cand = "::".join(parts[:k])
                    if cand in pn_owners:
                        nearest = cand
                        break
                if nearest == home:
                    return m.group(0)
                pn_changes += 1
                return "%s::payload_names::%s" % (qual(home, own), name)
            lines[i] = re.sub(r"(\w+::)?\bpayload_names::(k\w+)", repl, l)
        # 3. namespace blocks
        opens = [i for i, l in enumerate(lines) if l == "namespace navigatr" and lines[i + 1] == "{"]
        closes = [i for i, l in enumerate(lines) if l == "} // namespace navigatr"]
        if len(opens) != len(closes) or not opens:
            raise SystemExit("unbalanced navigatr blocks in " + rel)
        usings = []
        for ns in sorted(needs.get(rel, {})):
            q = qual(ns, own)
            for name in needs[rel][ns]:
                usings.append("using %s::%s;" % (q, name))
        usings.sort(key=lambda u: [c.lower() for c in u[6:-1].split("::")])
        total_using += len(usings)
        out = []
        for i, l in enumerate(lines):
            if i in closes:
                out += close_lines(own)
                out.append(l)
                continue
            out.append(l)
            if i - 1 in opens and l == "{":
                first = (i - 1) == opens[0]
                if first and fwd_text:
                    out.append("")
                    for home in sorted(fwd_text):
                        out += open_lines(home)
                        out += sorted(set(fwd_text[home]), key=lambda s: s.split()[1])
                        out += close_lines(home)
                        out.append("")
                out += open_lines(own)
                if first and usings:
                    out.append("")
                    out += usings
                    if i + 1 < len(lines) and lines[i + 1] != "":
                        out.append("")
        with open(path, "w", encoding="utf-8", newline="") as fh:
            fh.write("\n".join(out))
        rep.append("%-75s %-28s usings %3d fwd %d payload_names %d blocks %d" % (
            rel, own, len(usings), len(moves), pn_changes, len(opens)))
    rep.append("total using-declarations: %d" % total_using)
    with open(report, "w", encoding="utf-8") as fh:
        fh.write("\n".join(rep) + "\n")
    print(rep[-1])


if __name__ == "__main__":
    main()
