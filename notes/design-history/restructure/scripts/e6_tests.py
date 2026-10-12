#!/usr/bin/env python3
# e6_tests.py
# Step 6 test and tool updates: after `using namespace navigatr;` add a using-directive
# for each sub-namespace whose names the file uses; qualify payload_names where two
# nominated namespaces both have one; rewrite `nv::X` (namespace nv = navigatr)
# to `nv::<ns>::X`.
#
#   py -3 e6_tests.py <naviGATR dir> <pre-src naviGATR dir> <report.txt> <files...>

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import e6_ns  # noqa: E402


def main():
    navi, pre, report = sys.argv[1:4]
    files = sys.argv[4:]
    src_files, info, index, nested_ns, needs = e6_ns.compute(pre)
    cache = {}
    pn_owners = {h[: -len("::payload_names")] for h in nested_ns.get("payload_names", set())}
    rep = []
    for rel in files:
        path = os.path.join(navi, rel)
        with open(path, encoding="utf-8", newline="") as fh:
            text = fh.read()
        a = e6_ns.analyze_text(text)
        cl = set()
        for inc in a["includes"]:
            if inc in info:
                cl.add(inc)
                cl |= e6_ns.closure(inc, info, cache)
        # namespaces needed by unqualified uses
        nss = set()
        why = {}
        for name in a["uses"]:
            ents = [e for e in index.get(name, [])
                    if e[1] in cl and e[2] in e6_ns.DEF_KINDS
                    and "payload_names" not in e[0].split("::")]
            for e in ents:
                if e[0]:
                    nss.add(e[0])
                    why.setdefault(e[0], set()).add(name)
        # nv:: qualified uses
        nv_changes = 0
        if re.search(r"namespace nv = navigatr;", text):
            def nv_repl(m):
                nonlocal nv_changes
                name = m.group(1)
                homes = {e[0] for e in index.get(name, []) if e[1] in cl and e[2] in e6_ns.DEF_KINDS}
                if len(homes) != 1:
                    raise SystemExit("nv::%s homes %s in %s" % (name, homes, rel))
                home = next(iter(homes))
                if not home:
                    return m.group(0)
                nv_changes += 1
                return "nv::%s::%s" % (home, name)
            text = re.sub(r"\bnv::(\w+)\b(?!::)", nv_repl, text)
            # nv::X::Y (member access on a type)
            text = re.sub(r"\bnv::(\w+)(?=::)", lambda m: nv_repl(m) if m.group(1) not in (
                "core", "cli", "config", "io", "time", "math", "brain_profile", "inspection",
                "capture", "diagnostics", "framework", "resources", "sensors", "commands",
                "localization", "world_estimation", "targets", "publishing") else m.group(0), text)
        # payload_names
        pn_changes = 0
        uses_pn = re.findall(r"(?<![\w:])payload_names::(k\w+)", text)
        nominated = {n for n in nss}
        pn_nom = {n for n in nominated if n in pn_owners}
        if uses_pn and len(pn_nom) > 1:
            def pn_repl(m):
                nonlocal pn_changes
                name = m.group(1)
                homes = {e[0] for e in index.get(name, []) if "payload_names" in e[0].split("::")}
                assert len(homes) == 1, (name, homes)
                home = next(iter(homes))[: -len("::payload_names")]
                pn_changes += 1
                return "%s::payload_names::%s" % (home, name)
            text = re.sub(r"(?<![\w:])payload_names::(k\w+)", pn_repl, text)
        # using-directives
        added = 0
        if "using namespace navigatr;" in text and nss:
            lines = text.split("\n")
            out = []
            for l in lines:
                out.append(l)
                if l == "using namespace navigatr;":
                    for ns in sorted(nss):
                        out.append("using namespace navigatr::%s;" % ns)
                        added += 1
            text = "\n".join(out)
        elif nss and not re.search(r"namespace nv = navigatr;", text):
            rep.append("WARN %s uses %s but has no 'using namespace navigatr;'" % (rel, sorted(nss)))
        with open(path, "w", encoding="utf-8", newline="") as fh:
            fh.write(text)
        rep.append("%-40s directives %2d payload_names %2d nv %2d  [%s]" % (
            rel, added, pn_changes, nv_changes, " ".join(sorted(nss))))
    with open(report, "w", encoding="utf-8") as fh:
        fh.write("\n".join(rep) + "\n")
    print("\n".join(rep))


if __name__ == "__main__":
    main()
