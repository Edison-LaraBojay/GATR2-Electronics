#!/usr/bin/env python3
# e6_ns.py
# Step 6 analysis: folder -> namespace map, namespace-scope declaration index,
# per-file unqualified uses, and the using-declarations each file needs.
#
#   py -3 e6_ns.py index <naviGATR dir> <out.json> <report.txt>

import collections
import json
import os
import re
import sys

EXTS = (".h", ".cpp")

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

KEYWORDS = set(
    """alignas alignof and asm auto bool break case catch char char16_t char32_t class
    const constexpr const_cast continue decltype default delete do double dynamic_cast
    else enum explicit export extern false float for friend goto if inline int long
    mutable namespace new noexcept not nullptr operator or private protected public
    register reinterpret_cast return short signed sizeof static static_assert static_cast
    struct switch template this thread_local throw true try typedef typeid typename union
    unsigned using virtual void volatile wchar_t while override final uint8_t uint16_t
    uint32_t uint64_t int8_t int16_t int32_t int64_t size_t std""".split()
)

FAMILIES = {
    "pipeline/resources/cameras": "resources::cameras",
    "pipeline/resources/pico": "resources::pico",
    "pipeline/resources/links": "resources::links",
    "pipeline/resources/geometry": "resources::geometry",
    "pipeline/resources/tag_detectors": "resources::tag_detectors",
    "pipeline/resources/synthetic": "resources::synthetic",
    "pipeline/world_estimation/estimators/apriltag": "world_estimation::apriltag",
    "pipeline/world_estimation/estimators/noop": "world_estimation::noop",
}
AREAS = {
    "cli": "cli",
    "core": "core",
    "config": "config",
    "io": "io",
    "time": "time",
    "math": "math",
    "brain_profile": "brain_profile",
    "debug/inspection": "inspection",
    "debug/capture": "capture",
    "debug/diagnostics": "diagnostics",
    "pipeline/framework": "framework",
}
STAGES = [
    "resources",
    "sensors",
    "commands",
    "localization",
    "world_estimation",
    "targets",
    "publishing",
]
# Files holding only core::System member definitions: a member definition
# must sit in its class's namespace.
SYSTEM_MEMBER_FILES = {
    "brain_profile/control.cpp",
    "brain_profile/pico_operation.cpp",
    "brain_profile/profile_host.cpp",
    "brain_profile/profile_host.h",
    "brain_profile/recovery.cpp",
    "debug/diagnostics/events.cpp",
    "debug/inspection/inspection_state.cpp",
}
ROOT_FILES = {"main.cpp", "pipeline/register_all.h", "pipeline/register_all.cpp"}


def ns_of(rel):
    """Target namespace below navigatr for a path relative to src/ ('' = navigatr)."""
    if rel in ROOT_FILES:
        return ""
    if rel in SYSTEM_MEMBER_FILES:
        return "core"
    d = os.path.dirname(rel)
    for pre, ns in FAMILIES.items():
        if d == pre or d.startswith(pre + "/"):
            return ns
    for pre, ns in AREAS.items():
        if d == pre or d.startswith(pre + "/"):
            return ns
    for st in STAGES:
        pre = "pipeline/" + st
        if d == pre or d.startswith(pre + "/"):
            return st
    raise ValueError("no namespace for " + rel)


def strip_pp(text):
    """Return (directives [(line, text)], code with directive lines blanked)."""
    out_lines = []
    directives = []
    lines = text.split("\n")
    i = 0
    in_block = False
    while i < len(lines):
        line = lines[i]
        s = line.lstrip()
        if not in_block and s.startswith("#"):
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
        k = 0
        while True:
            if in_block:
                e = line.find("*/", k)
                if e < 0:
                    break
                in_block = False
                k = e + 2
            else:
                b = line.find("/*", k)
                lc = line.find("//", k)
                q = line.find('"', k)
                if b < 0 or (0 <= lc < b) or (0 <= q < b):
                    break
                in_block = True
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
            toks.append((kind, val, line))
        line += val.count("\n")
        pos = m.end()
    return toks


def skip_angles(stmt, i):
    """stmt[i] == '<'; return index after the matching '>'."""
    depth = 0
    while i < len(stmt):
        t = stmt[i]
        if t == "<":
            depth += 1
        elif t == ">":
            depth -= 1
        elif t == ">>":
            depth -= 2
        elif t in ("(",):
            j = i
            pd = 0
            while j < len(stmt):
                if stmt[j] == "(":
                    pd += 1
                elif stmt[j] == ")":
                    pd -= 1
                    if pd == 0:
                        break
                j += 1
            i = j
        i += 1
        if depth <= 0:
            return i
    return i


SPEC = {"inline", "static", "constexpr", "extern", "const", "volatile", "mutable",
        "explicit", "virtual", "friend", "thread_local", "typename"}


def analyze_stmt(stmt):
    """Namespace-scope statement tokens (braced bodies replaced by '{}') -> list of (name, kind)."""
    s = list(stmt)
    while s and s[0] == "template" and len(s) > 1 and s[1] == "<":
        s = s[skip_angles(s, 1):]
    # attributes
    while s and s[0] == "[" and len(s) > 1 and s[1] == "[":
        j = 2
        while j + 1 < len(s) and not (s[j] == "]" and s[j + 1] == "]"):
            j += 1
        s = s[j + 2:]
    if not s:
        return []
    if s[0] == "using":
        if len(s) > 2 and s[2] == "=":
            return [(s[1], "alias")]
        if len(s) > 1 and s[1] == "namespace":
            return [("<using-directive>", "using-directive")]
        return [("<using-decl>", "using-decl")]
    if s[0] == "typedef":
        ids = [t for t in s if re.match(r"^[A-Za-z_]\w*$", t) and t not in KEYWORDS]
        return [(ids[-1], "alias")] if ids else []
    if s[0] == "static_assert":
        return []
    k = 0
    while k < len(s) and s[k] in SPEC:
        k += 1
    if k < len(s) and s[k] in ("struct", "class", "union"):
        if k + 1 < len(s):
            name = s[k + 1]
            nxt = s[k + 2] if k + 2 < len(s) else ""
            if nxt == ";" and k == 0:
                return [(name, "fwd")]
            if nxt in ("{}", ":", "final"):
                return [(name, "type")]
            if nxt == "::" and k + 3 < len(s):
                return [(name + "::" + s[k + 3], "member-def")]
        # elaborated type in a declaration; fall through to declarator logic
    if k < len(s) and s[k] == "enum":
        j = k + 1
        if j < len(s) and s[j] in ("class", "struct"):
            j += 1
        if j < len(s):
            return [(s[j], "enum")]
        return []
    # function or variable
    depth_a = 0
    first_paren = None
    first_term = None
    i = k
    while i < len(s):
        t = s[i]
        if t == "<" and i > 0 and re.match(r"^[A-Za-z_]\w*$", s[i - 1]) and s[i - 1] != "operator":
            i = skip_angles(s, i)
            continue
        if t == "(" and first_paren is None:
            first_paren = i
            break
        if t in ("=", "{}", "[", ";") and first_term is None:
            first_term = i
            break
        i += 1
    if first_paren is not None:
        j = first_paren - 1
        # operator names
        if j >= 0 and s[j] == "operator":
            # operator()( ... )
            return [("operator()", "operator")]
        m = j
        while m >= 0 and s[m] != "operator" and not re.match(r"^[A-Za-z_]\w*$", s[m]):
            m -= 1
        if m >= 0 and s[m] == "operator":
            return [("operator" + "".join(s[m + 1:first_paren]), "operator")]
        if j >= 0 and re.match(r"^[A-Za-z_~]\w*$", s[j]):
            if j >= 1 and s[j - 1] == "::":
                q = []
                x = j - 1
                while x >= 1 and s[x] == "::":
                    q.insert(0, s[x - 1])
                    x -= 2
                return [("::".join(q + [s[j]]), "member-def")]
            if s[j] in KEYWORDS:
                return []
            return [(s[j], "func")]
        return []
    if first_term is not None:
        j = first_term - 1
        if j >= 0 and re.match(r"^[A-Za-z_]\w*$", s[j]) and s[j] not in KEYWORDS:
            if j >= 1 and s[j - 1] == "::":
                return [(s[j], "member-var-def")]
            return [(s[j], "var")]
    return []


def analyze_text(text):
    directives, code = strip_pp(text)
    includes = []
    for ln, d in directives:
        m = re.match(r'#\s*include\s+"([^"]+)"', d)
        if m:
            includes.append(m.group(1))
    toks = tokenize(code)
    # scope stack: (kind, name)
    stack = []
    decls = []        # (nspath tuple, name, kind, line)
    anon = set()      # file-local names
    uses = collections.Counter()
    use_lines = collections.defaultdict(list)
    qual_uses = collections.Counter()   # X:: prefixes
    stmt = []         # tokens of current statement at current namespace scope
    stmt_line = None
    enum_stack = []   # enumerator collection for unscoped enums at ns scope

    def ns_path():
        return tuple(n for k, n in stack if k == "ns")

    def at_ns_scope():
        return all(k in ("ns", "anon", "extern") for k, _ in stack)

    def in_anon():
        return any(k == "anon" for k, _ in stack)

    i = 0
    n = len(toks)
    while i < n:
        kind, t, line = toks[i]
        prev = toks[i - 1][1] if i > 0 else ""
        nxt = toks[i + 1][1] if i + 1 < n else ""
        if kind == "id" and t not in KEYWORDS:
            if prev not in (".", "->", "::"):
                uses[t] += 1
                if len(use_lines[t]) < 5:
                    use_lines[t].append(line)
            if nxt == "::" and prev not in (".", "->", "::"):
                qual_uses[t] += 1
        if at_ns_scope():
            if t == "{":
                # classify scope from stmt
                s = stmt
                if s and s[0] == "namespace":
                    if len(s) > 1:
                        stack.append(("ns", "".join(s[1:])))
                    else:
                        stack.append(("anon", ""))
                    stmt = []
                    i += 1
                    continue
                if s and s[0] == "inline" and len(s) > 1 and s[1] == "namespace":
                    stack.append(("ns", "".join(s[2:])))
                    stmt = []
                    i += 1
                    continue
                if len(s) == 2 and s[0] == "extern" and s[1].startswith('"'):
                    stack.append(("extern", ""))
                    stmt = []
                    i += 1
                    continue
                # find matching brace, skip body
                depth = 0
                j = i
                body = []
                while j < n:
                    tv = toks[j][1]
                    if tv == "{":
                        depth += 1
                    elif tv == "}":
                        depth -= 1
                        if depth == 0:
                            break
                    body.append(toks[j])
                    j += 1
                # uses inside body
                for bi in range(1, len(body)):
                    bk, bt, bl = body[bi]
                    bp = body[bi - 1][1]
                    bn = body[bi + 1][1] if bi + 1 < len(body) else "}"
                    if bk == "id" and bt not in KEYWORDS:
                        if bp not in (".", "->", "::"):
                            uses[bt] += 1
                            if len(use_lines[bt]) < 5:
                                use_lines[bt].append(bl)
                        if bn == "::" and bp not in (".", "->", "::"):
                            qual_uses[bt] += 1
                stmt2 = stmt + ["{}"]
                res = analyze_stmt(stmt2)
                # unscoped enum enumerators
                ss = [x for x in stmt if x not in SPEC]
                if ss and ss[0] == "enum" and (len(ss) < 2 or ss[1] not in ("class", "struct")):
                    expect = True
                    d2 = 0
                    for bk, bt, bl in body[1:]:
                        if bt in ("(", "{", "["):
                            d2 += 1
                        elif bt in (")", "}", "]"):
                            d2 -= 1
                        if d2 == 0 and bt == ",":
                            expect = True
                            continue
                        if expect and d2 == 0 and bk == "id":
                            res.append((bt, "enumerator"))
                        expect = False
                for name, k2 in res:
                    if in_anon():
                        anon.add(name)
                    else:
                        decls.append((ns_path(), name, k2, stmt_line or line))
                # does the statement continue (class/enum/initializer) or end (function body)?
                i = j + 1
                if i < n and toks[i][1] in (";", ","):
                    # class, enum, or braced initializer
                    stmt = stmt2 if toks[i][1] == "," else []
                    if toks[i][1] == ";":
                        stmt = []
                        stmt_line = None
                        i += 1
                    continue
                if res and res[0][1] in ("type", "enum"):
                    # struct X {...} var; (declarator follows)
                    stmt = stmt2
                    continue
                stmt = []
                stmt_line = None
                continue
            if t == "}":
                if stack:
                    stack.pop()
                stmt = []
                stmt_line = None
                i += 1
                continue
            if t == ";":
                if stmt:
                    res = analyze_stmt(stmt + [";"])
                    for name, k2 in res:
                        if in_anon():
                            anon.add(name)
                        else:
                            decls.append((ns_path(), name, k2, stmt_line or line))
                stmt = []
                stmt_line = None
                i += 1
                continue
            if not stmt:
                stmt_line = line
            stmt.append(t)
            i += 1
            continue
        i += 1
    return {
        "includes": includes,
        "decls": decls,
        "anon": sorted(anon),
        "uses": dict(uses),
        "use_lines": {k: v for k, v in use_lines.items()},
        "qual_uses": dict(qual_uses),
    }


def list_files(root, sub):
    out = []
    base = os.path.join(root, sub)
    for dp, dn, fn in os.walk(base):
        for f in fn:
            if f.endswith(EXTS):
                out.append(os.path.relpath(os.path.join(dp, f), base).replace("\\", "/"))
    return sorted(out)


def build_index(navi):
    src_files = list_files(navi, "src")
    info = {}
    for rel in src_files:
        with open(os.path.join(navi, "src", rel), encoding="utf-8") as fh:
            info[rel] = analyze_text(fh.read())
    return src_files, info


def closure(rel, info, cache):
    if rel in cache:
        return cache[rel]
    cache[rel] = set()
    out = set()
    for inc in info[rel]["includes"]:
        if inc in info:
            out.add(inc)
            out |= closure(inc, info, cache)
    cache[rel] = out
    return out


def ancestors(ns):
    """Namespaces visible by enclosing lookup from ns ('' = navigatr root)."""
    parts = ns.split("::") if ns else []
    return {"::".join(parts[:k]) for k in range(len(parts) + 1)}


def full_ns(file_ns, nspath):
    """Declaration nspath (e.g. ('navigatr','payload_names')) -> target namespace string."""
    rest = [p for p in nspath[1:]] if nspath and nspath[0] == "navigatr" else None
    if rest is None:
        return None
    parts = ([file_ns] if file_ns else []) + rest
    return "::".join(parts)


def relative_to(target, own):
    """Shortest qualifier for target seen from own (both below navigatr)."""
    tp = target.split("::") if target else []
    # lookup of the first component from own walks outward; the first
    # component of tp must not be shadowed by a same-named namespace nearer in
    return "::".join(tp)


DEF_KINDS = ("type", "enum", "alias", "func", "var", "enumerator", "operator")


def compute(navi):
    src_files, info = build_index(navi)
    # index: name -> list of (ns, file, kind)
    index = collections.defaultdict(list)
    nested_ns = collections.defaultdict(set)   # nested namespace names -> set(full ns)
    for rel in src_files:
        fns = ns_of(rel)
        for nspath, name, kind, line in info[rel]["decls"]:
            full = full_ns(fns, nspath)
            if full is None:
                continue
            index[name].append((full, rel, kind, line))
        for d in info[rel]["decls"]:
            pass
    # nested namespaces (payload_names)
    for rel in src_files:
        fns = ns_of(rel)
        for nspath, name, kind, line in info[rel]["decls"]:
            if len(nspath) > 1 and nspath[0] == "navigatr":
                for k in range(1, len(nspath)):
                    nested_ns[nspath[k]].add(full_ns(fns, nspath[: k + 1]))
    cache = {}
    raw = {}
    fwd_moves = {}
    for rel in src_files:
        own = ns_of(rel)
        vis = ancestors(own)
        cl = closure(rel, info, cache) | {rel}
        anon = set(info[rel]["anon"])
        need = collections.defaultdict(set)   # ns -> names
        for name in info[rel]["uses"]:
            ents = [e for e in index.get(name, []) if e[1] in cl and e[2] in DEF_KINDS
                    and "payload_names" not in e[0].split("::")]
            if not ents:
                continue
            kinds = {e[2] for e in ents}
            funcs_only = kinds <= {"func", "operator"}
            own_has = any(e[0] in vis for e in ents)
            if (name in anon or own_has) and not funcs_only:
                continue
            for e in ents:
                if e[0] in vis:
                    continue
                need[e[0]].add(name)
        # forward declarations of types defined in another namespace
        moves = []
        for nspath, name, kind, line in info[rel]["decls"]:
            if kind != "fwd" or full_ns(own, nspath) is None:
                continue
            homes = {e[0] for e in index.get(name, []) if e[2] == "type"}
            if len(homes) != 1:
                continue
            home = next(iter(homes))
            if home in vis:
                continue
            moves.append((home, name, line))
            need[home].add(name)
        fwd_moves[rel] = moves
        raw[rel] = need
    needs = {}
    for rel in src_files:
        own = ns_of(rel)
        provided = set()
        for inc in info[rel]["includes"]:
            if inc in raw and ns_of(inc) == own and inc.endswith(".h"):
                for ns, names in raw[inc].items():
                    provided |= {(ns, n) for n in names}
        needs[rel] = {}
        for ns, names in raw[rel].items():
            keep = sorted(n for n in names if (ns, n) not in provided)
            if keep:
                needs[rel][ns] = keep
    compute.fwd_moves = fwd_moves
    return src_files, info, index, nested_ns, needs


def main():
    cmd = sys.argv[1]
    if cmd == "index":
        navi, out_json, report = sys.argv[2:5]
        src_files, info, index, nested_ns, needs = compute(navi)
        rep = []
        rep.append("files: %d" % len(src_files))
        rep.append("")
        rep.append("== namespace per file ==")
        for rel in src_files:
            rep.append("%-70s %s" % (rel, ns_of(rel) or "(navigatr)"))
        rep.append("")
        rep.append("== names declared in more than one namespace ==")
        for name in sorted(index):
            nss = sorted({e[0] for e in index[name] if e[2] in DEF_KINDS})
            if len(nss) > 1:
                rep.append("%s: %s" % (name, "; ".join(
                    "%s[%s %s]" % (e[0], e[1], e[2]) for e in index[name] if e[2] in DEF_KINDS)))
        rep.append("")
        rep.append("== forward declarations whose definition is elsewhere ==")
        for name in sorted(index):
            fw = [e for e in index[name] if e[2] == "fwd"]
            dfn = [e for e in index[name] if e[2] in ("type",)]
            for f in fw:
                dns = {d[0] for d in dfn}
                if dns and f[0] not in dns:
                    rep.append("%s fwd in %s (%s:%d), defined in %s" % (
                        name, f[0], f[1], f[3], ",".join(sorted(dns))))
                if not dns:
                    rep.append("%s fwd in %s (%s:%d), no definition found" % (name, f[0], f[1], f[3]))
        rep.append("")
        rep.append("== member definitions whose class lives elsewhere ==")
        for name in sorted(index):
            for e in index[name]:
                if e[2] != "member-def" and e[2] != "member-var-def":
                    continue
                cls = name.split("::")[0]
                cns = {d[0] for d in index.get(cls, []) if d[2] in ("type", "alias")}
                if cns and e[0] not in cns:
                    rep.append("%s in %s (%s:%d), class in %s" % (name, e[0], e[1], e[3], ",".join(sorted(cns))))
        rep.append("")
        rep.append("== nested namespaces ==")
        for k, v in sorted(nested_ns.items()):
            rep.append("%s: %s" % (k, ", ".join(sorted(v))))
        rep.append("")
        rep.append("== needed using-declarations per file ==")
        total = 0
        for rel in src_files:
            nd = needs[rel]
            cnt = sum(len(v) for v in nd.values())
            total += cnt
            rep.append("%s (%s): %d" % (rel, ns_of(rel) or "(navigatr)", cnt))
            for ns in sorted(nd):
                rep.append("    %s: %s" % (ns or "(navigatr)", " ".join(nd[ns])))
        rep.append("total using-declarations: %d" % total)
        with open(report, "w", encoding="utf-8") as fh:
            fh.write("\n".join(rep) + "\n")
        with open(out_json, "w", encoding="utf-8") as fh:
            json.dump({"index": {k: v for k, v in index.items()}, "needs": needs,
                       "info": {k: {"includes": v["includes"], "anon": v["anon"]} for k, v in info.items()}},
                      fh, indent=1)
        print("\n".join(rep[-1:]))


if __name__ == "__main__":
    main()
