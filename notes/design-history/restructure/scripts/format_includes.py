# clang-format the changed lines of the files whose include lines changed (worktree vs index).
#
#   py -3 format_includes.py prepare <repo root> <out dir>   diff + file list, touches nothing
#   py -3 format_includes.py apply <repo root> <out dir>     clang-format-diff on the saved diff
#
# prepare before staging the move, apply after, so `git diff` then shows only the format delta.

import os
import re
import subprocess
import sys

CF = "C:/Users/mbes1/AppData/Roaming/Python/Python314/site-packages/clang_format/data/bin/clang-format.exe"
CFD = "C:/Users/mbes1/AppData/Roaming/Python/Python314/Scripts/clang-format-diff.exe"
EXT = (".h", ".hpp", ".cpp", ".c")


def git(root, *args):
    r = subprocess.run(["git", "-C", root] + list(args), capture_output=True, text=True,
                       encoding="utf-8")
    if r.returncode:
        sys.exit("git %s failed: %s" % (" ".join(args), r.stderr))
    return r.stdout


def save(path, text):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def prepare(root, out):
    os.makedirs(out, exist_ok=True)
    full = git(root, "diff", "-U0", "--no-color", "--", "pi/naviGATR")
    files, cur = [], None
    for line in full.split("\n"):
        if line.startswith("+++ "):
            cur = line[6:] if line.startswith("+++ b/") else None
        elif cur and re.match(r"[+-]\s*#\s*include\b", line) and cur.endswith(EXT):
            if cur not in files:
                files.append(cur)
    files.sort()
    fmt = git(root, "diff", "-U0", "--no-color", "--", *files)
    save(os.path.join(out, "fmt_files.txt"), "\n".join(files) + "\n")
    save(os.path.join(out, "fmt_input.diff"), fmt)
    print("files with changed include lines: %d" % len(files))
    print("diff lines fed to clang-format-diff: %d" % len(fmt.split("\n")))


def apply(root, out):
    with open(os.path.join(out, "fmt_input.diff"), encoding="utf-8") as f:
        fmt = f.read()
    r = subprocess.run([CFD, "-p1", "-i", "-binary", CF], input=fmt,
                       text=True, cwd=root, capture_output=True)
    sys.stdout.write(r.stdout)
    sys.stderr.write(r.stderr)
    if r.returncode:
        sys.exit("clang-format-diff failed: %d" % r.returncode)
    print("clang-format-diff: ok")


if __name__ == "__main__":
    {"prepare": prepare, "apply": apply}[sys.argv[1]](sys.argv[2], sys.argv[3])
