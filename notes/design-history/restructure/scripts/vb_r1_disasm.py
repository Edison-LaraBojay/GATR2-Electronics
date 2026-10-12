#!/usr/bin/env python3
# vb_r1_disasm.py
# Per-function disassembly comparison of two navigatr_core builds (Debug, -O0).
# Functions keyed by demangled name; addresses and section offsets normalized,
# relocation symbol names kept.
#
#   py -3 vb_r1_disasm.py <objdump> <old obj dir> <new obj dir> <out report>

import collections
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

FUNC_RE = re.compile(r"^[0-9a-f]+ <(.*)>:$")
INSN_RE = re.compile(r"^\s+[0-9a-f]+:\t(.*)$")
RELOC_RE = re.compile(r"^\s+[0-9a-f]+: (R_\S+)\t(.*)$")
ADDR_TGT_RE = re.compile(r"\b[0-9a-f]+ <")
RIP_RE = re.compile(r"-?0x[0-9a-f]+\(%rip\)")
SECOFF_RE = re.compile(r"^(\.(?:rdata|data|bss|text|xdata|pdata)[^+]*)\+0x[0-9a-f]+$")


def objs(root):
    out = []
    for dp, dn, fn in os.walk(root):
        for f in fn:
            if f.endswith(".obj"):
                out.append(os.path.join(dp, f))
    return sorted(out)


def disasm(objdump, path):
    txt = subprocess.run(
        [objdump, "-d", "-r", "-C", "--no-show-raw-insn", path],
        capture_output=True,
        text=True,
        errors="replace",
    ).stdout
    funcs = collections.defaultdict(list)
    cur = None
    body = []
    for line in txt.splitlines():
        m = FUNC_RE.match(line)
        if m:
            if cur is not None:
                funcs[cur].append("\n".join(body))
            cur = m.group(1)
            body = []
            continue
        if line.startswith("Disassembly of section"):
            if cur is not None:
                funcs[cur].append("\n".join(body))
            cur = None
            body = []
            continue
        if cur is None:
            continue
        r = RELOC_RE.match(line)
        if r:
            sym = r.group(2).strip()
            sm = SECOFF_RE.match(sym)
            if sm:
                sym = sm.group(1) + "+OFF"
            body.append("  R " + r.group(1) + " " + sym)
            continue
        i = INSN_RE.match(line)
        if i:
            ins = re.sub(r"\s+#.*$", "", i.group(1))
            ins = RIP_RE.sub("X(%rip)", ins)
            ins = ADDR_TGT_RE.sub("<", ins).rstrip()
            body.append(ins)
    if cur is not None:
        funcs[cur].append("\n".join(body))
    return funcs


def collect(objdump, root):
    all_funcs = collections.defaultdict(collections.Counter)
    where = collections.defaultdict(set)
    paths = objs(root)
    with ThreadPoolExecutor(max_workers=16) as ex:
        results = list(ex.map(lambda p: (p, disasm(objdump, p)), paths))
    for p, funcs in results:
        rel = os.path.relpath(p, root).replace("\\", "/")
        for name, bodies in funcs.items():
            for b in bodies:
                all_funcs[name][b] += 1
            where[name].add(rel)
    return all_funcs, where, len(paths)


def main():
    objdump, old_root, new_root, out = sys.argv[1:5]
    old, old_where, n_old = collect(objdump, old_root)
    new, new_where, n_new = collect(objdump, new_root)
    rep = []
    rep.append("objects: old %d, new %d" % (n_old, n_new))
    rep.append("functions: old %d, new %d" % (len(old), len(new)))
    only_old = sorted(set(old) - set(new))
    only_new = sorted(set(new) - set(old))
    differ = []
    for name in sorted(set(old) & set(new)):
        if set(old[name]) != set(new[name]):
            differ.append(name)
    rep.append("only in old: %d, only in new: %d, same name different code: %d"
               % (len(only_old), len(only_new), len(differ)))
    rep.append("")
    rep.append("==== ONLY IN OLD ====")
    rep.extend("  %s  [%s]" % (n, ",".join(sorted(old_where[n]))) for n in only_old)
    rep.append("==== ONLY IN NEW ====")
    rep.extend("  %s  [%s]" % (n, ",".join(sorted(new_where[n]))) for n in only_new)
    rep.append("==== DIFFERENT CODE ====")
    import difflib
    for name in differ:
        rep.append("")
        rep.append("## %s" % name)
        rep.append("   old in %s" % ",".join(sorted(old_where[name])))
        rep.append("   new in %s" % ",".join(sorted(new_where[name])))
        ob = sorted(set(old[name]))
        nb = sorted(set(new[name]))
        rep.append("   variants old %d new %d" % (len(ob), len(nb)))
        d = difflib.unified_diff(ob[0].split("\n"), nb[0].split("\n"), lineterm="", n=2)
        lines = list(d)
        rep.extend("   " + l for l in lines[:80])
        if len(lines) > 80:
            rep.append("   ... %d more diff lines" % (len(lines) - 80))
    with open(out, "w", encoding="utf-8") as fh:
        fh.write("\n".join(rep) + "\n")
    print("\n".join(rep[:3]))


if __name__ == "__main__":
    main()
