#!/usr/bin/env python3
# vb_r1_units.py
# Token-level unit comparison of naviGATR code, baseline tree vs restructured tree.
# Units: namespace-scope declarations/definitions (class System members as their own units).
# Comments, whitespace, #include and #pragma once are dropped.
#
#   py -3 vb_r1_units.py <old naviGATR dir> <new naviGATR dir> <out report>

import collections
import difflib
import os
import re
import sys

EXTS = (".h", ".hpp", ".cpp", ".c", ".in")
SUBDIRS = ("src", "tests", "tools")
TRANSPARENT_CLASSES = {"System"}

TOKEN_RE = re.compile(
    r"""
    (?P<ws>\s+)
  | (?P<lcom>//[^\n]*)
  | (?P<bcom>/\*.*?\*/)
  | (?P<raw>R"(?P<delim>[^(\s]*)\(.*?\)(?P=delim)")
  | (?P<str>(?:u8|u|U|L)?"(?:\\.|[^"\\\n])*")
  | (?P<chr>(?:u8|u|U|L)?'(?:\\.|[^'\\\n])+')
  | (?P<num>\.?\d(?:[eEpP][+-]|[\w.]|'(?=\w))*)
  | (?P<id>[A-Za-z_]\w*)
  | (?P<punct>::|->\*?|\.\.\.|<<=|>>=|<=>|[<>=!+\-*/%&|^]=|&&|\|\||\+\+|--|<<|\#\#|.)
    """,
    re.S | re.X,
)


def strip_pp(text):
    """Split into (preprocessor directives, code with directives blanked)."""
    out_lines = []
    directives = []
    lines = text.split("\n")
    i = 0
    in_block_comment = False
    while i < len(lines):
        line = lines[i]
        s = line.lstrip()
        if not in_block_comment and s.startswith("#"):
            full = line
            j = i
            while full.endswith("\\") and j + 1 < len(lines):
                j += 1
                full = full[:-1] + " " + lines[j]
            directives.append((i + 1, full.strip()))
            for _ in range(i, j + 1):
                out_lines.append("")
            i = j + 1
            continue
        # track block comments crudely so a '#' inside one is not a directive
        k = 0
        while True:
            if in_block_comment:
                e = line.find("*/", k)
                if e < 0:
                    break
                in_block_comment = False
                k = e + 2
            else:
                b = line.find("/*", k)
                lc = line.find("//", k)
                if b < 0 or (0 <= lc < b):
                    break
                in_block_comment = True
                k = b + 2
        out_lines.append(line)
        i += 1
    return directives, "\n".join(out_lines)


def tokenize(text):
    toks = []
    pos = 0
    line = 1
    while pos < len(text):
        m = TOKEN_RE.match(text, pos)
        if not m:
            raise ValueError("tokenize failed at %r" % text[pos : pos + 40])
        kind = m.lastgroup
        val = m.group(0)
        if kind not in ("ws", "lcom", "bcom"):
            toks.append((val, line))
        line += val.count("\n")
        pos = m.end()
    return toks


def pp_units(directives):
    units = []
    for ln, d in directives:
        norm = re.sub(r"\s+", " ", re.sub(r"//.*$", "", d)).strip()
        if re.match(r"#\s*include\b", norm) or re.match(r"#\s*pragma\s+once\b", norm):
            continue
        units.append((ln, norm))
    return units


def units_of(text):
    """Return list of (scope, access, token-string, line)."""
    directives, code = strip_pp(text)
    toks = tokenize(code)
    out = []
    for ln, d in pp_units(directives):
        out.append(("<pp>", "", d, ln))
    # scope stack entries: (kind, name, access)
    stack = []
    i = 0
    n = len(toks)

    def scope_str():
        return "::".join(s[1] for s in stack)

    def access():
        for s in reversed(stack):
            if s[0] == "class":
                return s[2]
        return ""

    while i < n:
        t = toks[i][0]
        # close of a transparent scope
        if t == "}":
            if stack:
                kind = stack.pop()[0]
                i += 1
                if kind == "class" and i < n and toks[i][0] == ";":
                    i += 1
                continue
            raise ValueError("unbalanced } at line %d" % toks[i][1])
        # namespace
        if t == "namespace":
            j = i + 1
            name = []
            while j < n and toks[j][0] not in ("{", "=", ";"):
                name.append(toks[j][0])
                j += 1
            if j < n and toks[j][0] == "{":
                stack.append(("ns", "".join(name) or "<anon>", ""))
                i = j + 1
                continue
        if t == "extern" and i + 2 < n and toks[i + 1][0].startswith('"') and toks[i + 2][0] == "{":
            stack.append(("ns", "extern" + toks[i + 1][0], ""))
            i += 3
            continue
        # transparent class
        if t in ("class", "struct") and i + 1 < n and toks[i + 1][0] in TRANSPARENT_CLASSES:
            j = i + 2
            while j < n and toks[j][0] not in ("{", ";"):
                j += 1
            if j < n and toks[j][0] == "{":
                stack.append(("class", toks[i + 1][0], "private" if t == "class" else "public"))
                i = j + 1
                continue
        # access specifier inside transparent class
        if (
            t in ("public", "private", "protected")
            and i + 1 < n
            and toks[i + 1][0] == ":"
            and stack
            and stack[-1][0] == "class"
        ):
            k, nm, _ = stack.pop()
            stack.append((k, nm, t))
            i += 2
            continue
        # a unit
        start_line = toks[i][1]
        depth = 0
        unit = []
        while i < n:
            tv = toks[i][0]
            unit.append(tv)
            i += 1
            if tv in ("(", "[", "{"):
                depth += 1
            elif tv in (")", "]", "}"):
                depth -= 1
                if tv == "}" and depth == 0:
                    nxt = toks[i][0] if i < n else ""
                    if nxt == ";":
                        unit.append(";")
                        i += 1
                        break
                    if nxt in ("{", ",", "catch"):
                        continue
                    break
            elif tv == ";" and depth == 0:
                break
        out.append((scope_str(), access(), " ".join(unit), start_line))
    if stack:
        raise ValueError("unclosed scope %s" % stack)
    return out


def collect(root):
    files = {}
    for sub in SUBDIRS:
        base = os.path.join(root, sub)
        for dp, dn, fn in os.walk(base):
            if "third_party" in dp.replace("\\", "/").split("/"):
                continue
            for f in fn:
                if f.endswith(EXTS):
                    p = os.path.join(dp, f)
                    rel = os.path.relpath(p, root).replace("\\", "/")
                    with open(p, encoding="utf-8") as fh:
                        files[rel] = fh.read()
    return files


def main():
    old_root, new_root, out_path = sys.argv[1:4]
    old_files = collect(old_root)
    new_files = collect(new_root)
    old_units = collections.Counter()
    new_units = collections.Counter()
    old_where = collections.defaultdict(list)
    new_where = collections.defaultdict(list)
    for tree, files, cnt, where in (
        ("old", old_files, old_units, old_where),
        ("new", new_files, new_units, new_where),
    ):
        for rel, text in sorted(files.items()):
            try:
                us = units_of(text)
            except Exception as e:  # noqa
                print("PARSE FAIL %s %s: %s" % (tree, rel, e))
                continue
            for scope, acc, body, ln in us:
                key = (scope, acc, body)
                cnt[key] += 1
                where[key].append("%s:%d" % (rel, ln))
    missing = old_units - new_units
    extra = new_units - old_units
    lines = []
    lines.append("old files %d, new files %d" % (len(old_files), len(new_files)))
    lines.append("old units %d, new units %d" % (sum(old_units.values()), sum(new_units.values())))
    lines.append("units only in old: %d, only in new: %d" % (sum(missing.values()), sum(extra.values())))
    lines.append("")

    # pair missing/extra by best similarity for readable token diffs
    extra_keys = list(extra.elements())
    used = set()
    lines.append("==== CHANGED OR REMOVED (old units without an identical new unit) ====")
    for key in sorted(missing.elements(), key=lambda k: old_where[k][0]):
        scope, acc, body = key
        best = None
        best_r = 0.0
        otoks = body.split(" ")
        for idx, ek in enumerate(extra_keys):
            if idx in used:
                continue
            etoks = ek[2].split(" ")
            if abs(len(etoks) - len(otoks)) > max(len(otoks), 40):
                continue
            sm = difflib.SequenceMatcher(None, otoks, etoks, autojunk=False)
            if sm.real_quick_ratio() < best_r:
                continue
            r = sm.ratio()
            if r > best_r:
                best_r, best = r, idx
        lines.append("")
        lines.append("OLD %s [%s|%s] %s" % (old_where[key][0], scope, acc, body[:160]))
        if best is not None and best_r > 0.5:
            used.add(best)
            ek = extra_keys[best]
            lines.append("  ~NEW %s [%s|%s] ratio %.3f" % (new_where[ek][0], ek[0], ek[1], best_r))
            etoks = ek[2].split(" ")
            sm = difflib.SequenceMatcher(None, otoks, etoks, autojunk=False)
            if ek[0] != scope or ek[1] != acc:
                lines.append("  SCOPE/ACCESS CHANGED: %s|%s -> %s|%s" % (scope, acc, ek[0], ek[1]))
            for op, a1, a2, b1, b2 in sm.get_opcodes():
                if op == "equal":
                    continue
                ctx0 = " ".join(otoks[max(0, a1 - 6) : a1])
                lines.append(
                    "  %s: ...%s | -[%s] +[%s]"
                    % (op, ctx0, " ".join(otoks[a1:a2])[:600], " ".join(etoks[b1:b2])[:600])
                )
        else:
            lines.append("  (no similar new unit)")
    lines.append("")
    lines.append("==== NEW ONLY (new units without an identical old unit, unpaired) ====")
    for idx, ek in enumerate(extra_keys):
        if idx in used:
            continue
        lines.append("NEW %s [%s|%s] %s" % (new_where[ek][0], ek[0], ek[1], ek[2][:400]))

    # anonymous-namespace and static units: file placement
    lines.append("")
    lines.append("==== INTERNAL-LINKAGE UNITS: old file -> new file(s) ====")
    for key in sorted(old_units):
        scope, acc, body = key
        internal = "<anon>" in scope.split("::") or body.startswith("static ")
        if not internal:
            continue
        if not old_where[key][0].startswith("src/"):
            continue
        ow = sorted(set(w.split(":")[0] for w in old_where[key]))
        nw = sorted(set(w.split(":")[0] for w in new_where.get(key, [])))
        if len(ow) != len(nw):
            lines.append("COUNT %s -> %s : %s" % (ow, nw, body[:120]))
    with open(out_path, "w", encoding="ascii", errors="replace") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\n".join(lines[:4]))


if __name__ == "__main__":
    main()
