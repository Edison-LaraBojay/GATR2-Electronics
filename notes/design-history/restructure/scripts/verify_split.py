#!/usr/bin/env python3
"""Independent check of the System split (step 2): every top-level unit of the
old system.cpp and every member of the old class System lands, text intact,
in exactly one place after the split. Shares no tables with split_system.py.

Unit = one namespace-scope (cpp) or class-scope (h) declaration or definition,
from its directly attached comment lines to its closing brace or semicolon.
A comment block followed by a blank line is a standalone comment unit.
Key = (scope path, access, whitespace-free text).

Usage:
  py -3 verify_split.py --orig <dir with old system.cpp, system.h> --src <src>
      [--out <log>]
"""

import argparse
import collections
import os
import re
import sys

NEW_CPP = [
    "core/build.cpp",
    "core/run.cpp",
    "brain_profile/profile_host.cpp",
    "brain_profile/control.cpp",
    "brain_profile/pico_operation.cpp",
    "brain_profile/recovery.cpp",
    "debug/diagnostics/events.cpp",
    "debug/inspection/inspection_state.cpp",
]
NEW_H = [
    "brain_profile/profile_host.h",
    "brain_profile/pico_operation.h",
    "debug/diagnostics/events.h",
]
SYSTEM_H = "core/system.h"

Unit = collections.namedtuple("Unit", "file first last scope access kind text code")


def code_parts(lines):
    """Per line: the code with comments and literals blanked."""
    out = []
    in_block = False
    for line in lines:
        buf = []
        i, n = 0, len(line)
        while i < n:
            c = line[i]
            if in_block:
                if line.startswith("*/", i):
                    in_block = False
                    i += 2
                else:
                    i += 1
                continue
            if line.startswith("//", i):
                break
            if line.startswith("/*", i):
                in_block = True
                i += 2
                continue
            if c == '"' or (c == "'" and not (i and (line[i - 1].isalnum() or line[i - 1] == "_"))):
                q = c
                i += 1
                while i < n and line[i] != q:
                    i += 2 if line[i] == "\\" else 1
                buf.append(q + q)
                i += 1
                continue
            buf.append(c)
            i += 1
        out.append("".join(buf))
    return out


def squash(s):
    return re.sub(r"\s+", "", s)


def parse(path, rel, into_class=None):
    """Units at namespace scope; with into_class, descend into that class body
    and return its members instead (access tracked)."""
    with open(path, encoding="utf-8", newline="") as f:
        lines = f.read().split("\n")
    if lines and lines[-1] == "":
        lines.pop()
    codes = code_parts(lines)
    units, structural = [], []
    scope = []           # namespace / class names we descended into
    access = ""
    pend_comments = []   # (lineno) of comment lines waiting for code
    cur = None           # dict: start, head, depth, lines
    for ln, (line, code) in enumerate(zip(lines, codes), 1):
        s = line.strip()
        cs = code.strip()
        if cur is None:
            if not s:
                if pend_comments:
                    units.append(mk(rel, lines, pend_comments[0], pend_comments[-1], scope,
                                    access, "comment"))
                    pend_comments = []
                continue
            if not cs:          # comment-only line
                pend_comments.append(ln)
                continue
            if cs.startswith("#"):
                structural.append((ln, line))
                if pend_comments:
                    units.append(mk(rel, lines, pend_comments[0], pend_comments[-1], scope,
                                    access, "comment"))
                    pend_comments = []
                continue
            if into_class and scope and scope[-1] == into_class and re.fullmatch(
                    r"(public|private|protected)\s*:", cs):
                access = cs[:-1].strip()
                structural.append((ln, line))
                continue
            if re.fullmatch(r"\}\s*;?", cs) and scope:     # scope close
                structural.append((ln, line))
                scope.pop()
                if pend_comments:
                    die("%s:%d comment before scope close" % (rel, ln))
                continue
            cur = dict(start=pend_comments[0] if pend_comments else ln, head="", depth=0)
            pend_comments = []
        # inside a unit (or starting one): walk the code
        k = 0
        ended = False
        while k < len(code):
            ch = code[k]
            if ch == "{":
                if cur["depth"] == 0:
                    h = squash(cur["head"])
                    m = re.fullmatch(r"namespace([A-Za-z_]\w*)?", h)
                    mc = into_class and re.fullmatch(r"class" + re.escape(into_class), h)
                    if m or mc:
                        if code[k + 1:].strip():
                            die("%s:%d code after scope open" % (rel, ln))
                        scope.append(m.group(1) or "<anon>" if m else into_class)
                        access = "private" if mc else access
                        for x in range(cur["start"], ln + 1):
                            structural.append((x, lines[x - 1]))
                        cur = None
                        ended = True
                        break
                cur["depth"] += 1
            elif ch == "}":
                cur["depth"] -= 1
                if cur["depth"] == 0:
                    rest = code[k + 1:].strip()
                    if rest in ("", ";"):
                        units.append(mk(rel, lines, cur["start"], ln, scope, access, "code"))
                        cur = None
                        ended = True
                        break
            elif ch == ";" and cur["depth"] == 0:
                if code[k + 1:].strip():
                    die("%s:%d code after ';' at unit level" % (rel, ln))
                units.append(mk(rel, lines, cur["start"], ln, scope, access, "code"))
                cur = None
                ended = True
                break
            if cur["depth"] == 0:
                cur["head"] += ch
            k += 1
        if not ended and cur is not None and cur["depth"] == 0:
            cur["head"] += " "
    if cur is not None:
        die("%s: unterminated unit at line %d" % (rel, cur["start"]))
    if pend_comments:
        units.append(mk(rel, lines, pend_comments[0], pend_comments[-1], scope, access, "comment"))
    if scope:
        die("%s: unclosed scope %r" % (rel, scope))
    return units, structural, lines


def mk(rel, lines, a, b, scope, access, kind):
    text = "\n".join(lines[a - 1:b])
    code = "\n".join(l for l in lines[a - 1:b] if not l.strip().startswith("//"))
    return Unit(rel, a, b, "::".join(scope), access, kind, text, code)


def name_of(u):
    if u.kind == "comment":
        first = u.text.strip().splitlines()[0]
        return first[:60]
    head = re.split(r"[{;=[]", squash_keep(u.code), maxsplit=1)[0]
    if head.startswith(("class ", "struct ")):
        return " ".join(head.split()[:2])
    m = re.search(r"([~\w:]+)\s*\(", head)
    if m:
        return m.group(1)
    ids = re.findall(r"[~\w:]+", head)
    return ids[-1] if ids else head[:60]


def squash_keep(s):
    return " ".join(s.split())


def die(msg):
    sys.stderr.write("verify_split: " + msg + "\n")
    sys.exit(2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--orig", required=True)
    ap.add_argument("--src", required=True)
    ap.add_argument("--out")
    a = ap.parse_args()
    out = []
    p = out.append
    bad = 0

    # ---- system.cpp -> 8 .cpp + profile_host.h (namespace scope) ----
    old_cpp, old_cpp_struct, _ = parse(os.path.join(a.orig, "system.cpp"), "old:system.cpp")
    new_units = []
    for rel in NEW_CPP + ["brain_profile/profile_host.h"]:
        u, st, _ = parse(os.path.join(a.src, rel), rel)
        new_units += u
    pool = collections.defaultdict(list)
    for u in new_units:
        pool[(u.scope, squash(u.text))].append(u)
    used = set()
    p("== system.cpp: %d units (%d code, %d standalone comments)" % (
        len(old_cpp), sum(u.kind == "code" for u in old_cpp),
        sum(u.kind == "comment" for u in old_cpp)))
    p("%-28s %-11s %-9s %-40s %s" % ("unit", "old lines", "scope", "new place", "match"))
    for u in old_cpp:
        hits = pool.get((u.scope, squash(u.text)), [])
        if len(hits) == 1:
            h = hits[0]
            used.add(id(h))
            exact = "exact" if h.text == u.text else "ws-only"
            p("%-28s %5d-%-5d %-9s %-40s %s" % (name_of(u)[:28], u.first, u.last, u.scope or "-",
                                            "%s:%d-%d" % (h.file, h.first, h.last), exact))
        elif not hits:
            tag = "DROPPED" if u.kind == "comment" else "MISSING"
            if tag == "MISSING":
                bad += 1
            p("%-28s %5d-%-5d %-9s %-40s %s" % (name_of(u)[:28], u.first, u.last, u.scope or "-",
                                            "-", tag))
        else:
            bad += 1
            p("%-28s %5d-%-5d %s  DUPLICATED %d times" % (name_of(u)[:28], u.first, u.last,
                                                      u.scope, len(hits)))
    extra = [u for u in new_units if id(u) not in used]
    p("new units with no original (must be file labels only):")
    for u in extra:
        ok = u.kind == "comment" and u.first == 1 and u.last == 1
        if not ok:
            bad += 1
        p("   %s:%d-%d %s %r" % (u.file, u.first, u.last, "label" if ok else "UNEXPECTED",
                               u.text[:70]))
    p("old structural lines (includes, namespace lines): %d" % len(old_cpp_struct))

    # ---- system.h: namespace-scope structs and class members ----
    old_h_ns, old_h_struct, old_h_lines = parse(os.path.join(a.orig, "system.h"), "old:system.h")
    new_h_ns, new_h_struct, new_h_lines = parse(os.path.join(a.src, SYSTEM_H), SYSTEM_H)
    moved_ns = []
    for rel in NEW_H[1:]:
        u, st, _ = parse(os.path.join(a.src, rel), rel)
        moved_ns += u
    old_cls = parse(os.path.join(a.orig, "system.h"), "old:system.h", into_class="System")[0]
    new_cls = parse(os.path.join(a.src, SYSTEM_H), SYSTEM_H, into_class="System")[0]
    old_cls = [u for u in old_cls if u.scope == "navigatr::System"]
    new_cls = [u for u in new_cls if u.scope == "navigatr::System"]

    # preamble: everything before "namespace navigatr"
    def pre(lines):
        return lines[:lines.index("namespace navigatr")]
    op, np_ = pre(old_h_lines), pre(new_h_lines)
    added = collections.Counter(np_) - collections.Counter(op)
    removed = collections.Counter(op) - collections.Counter(np_)
    np_minus = [l for l in np_ if l not in added]
    p("")
    p("== system.h preamble (%d -> %d lines)" % (len(op), len(np_)))
    p("   added: %s" % sorted(added))
    p("   removed: %s" % sorted(removed))
    if removed or np_minus != op or any(not l.startswith("#include ") for l in added):
        bad += 1
        p("   PREAMBLE CHANGED beyond added includes")
    else:
        p("   identical apart from the added include lines")

    # namespace-scope units (structs, class System as one unit is skipped)
    def ns_units(units):
        return [u for u in units if u.scope == "navigatr" and not squash(u.code).startswith("classSystem")]
    old_ns = ns_units(old_h_ns)
    new_ns = ns_units(new_h_ns) + [u for u in moved_ns if u.scope == "navigatr"]
    pool = collections.defaultdict(list)
    for u in new_ns:
        pool[squash(u.text)].append(u)
    p("")
    p("== system.h namespace-scope units: %d old, %d new" % (len(old_ns), len(new_ns)))
    usedn = set()
    for u in old_ns:
        hits = pool.get(squash(u.text), [])
        if len(hits) != 1:
            bad += 1
            p("   %-24s %d-%d  %s" % (name_of(u), u.first, u.last, "MISSING" if not hits else "DUP"))
            continue
        h = hits[0]
        usedn.add(id(h))
        p("   %-24s %4d-%-4d -> %s:%d-%d %s" % (name_of(u), u.first, u.last, h.file, h.first, h.last,
                                           "exact" if h.text == u.text else "ws-only"))
    for u in new_ns:
        if id(u) not in usedn:
            bad += 1
            p("   UNEXPECTED new unit %s:%d-%d %r" % (u.file, u.first, u.last, u.text[:60]))

    # class members
    p("")
    p("== class System members: %d old units, %d new units" % (len(old_cls), len(new_cls)))
    pool = collections.defaultdict(list)
    for u in new_cls:
        pool[(u.access, squash(u.text))].append(u)
    usedc = set()
    for u in old_cls:
        hits = [h for h in pool.get((u.access, squash(u.text)), []) if id(h) not in usedc]
        if not hits:
            bad += 1
            p("   MISSING %-8s %d-%d %r" % (u.access, u.first, u.last, u.text[:70]))
            continue
        h = hits[0]
        usedc.add(id(h))
        p("   %-8s %-26s %4d-%-4d -> %4d-%-4d %s" % (u.access, name_of(u)[:26], u.first, u.last,
                                                h.first, h.last,
                                                "exact" if h.text == u.text else "ws-only"))
    p("new class units with no original (must be standalone labels):")
    for u in new_cls:
        if id(u) not in usedc:
            ok = u.kind == "comment" and u.first == u.last
            if not ok:
                bad += 1
            p("   %4d %-8s %s %s" % (u.first, u.access, "label" if ok else "UNEXPECTED", u.text.strip()))

    # data members: same sequence, same place (the tail of the class)
    def tail(units):
        idx = [i for i, u in enumerate(units) if squash(u.code).startswith("doubleloop_rate_hz_")]
        if len(idx) != 1:
            die("data member anchor")
        return units[idx[0]:]
    ot, nt = tail(old_cls), tail(new_cls)
    same = [squash(u.text) for u in ot] == [squash(u.text) for u in nt]
    exact = [u.text for u in ot] == [u.text for u in nt]
    p("")
    p("== data members: %d old, %d new, same order: %s, exact text: %s" % (len(ot), len(nt), same, exact))
    if not same:
        bad += 1

    p("")
    p("RESULT: %s (%d problems)" % ("OK" if not bad else "FAIL", bad))
    text = "\n".join(out) + "\n"
    sys.stdout.write(text)
    if a.out:
        with open(a.out, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
