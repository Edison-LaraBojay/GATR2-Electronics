# check_md_links.py
# Relative markdown links in a tree that do not resolve, plus non-ASCII
# characters in the given files.
#
#   py -3 check_md_links.py <root> [--ascii <file> ...]

import os
import re
import sys

LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)\)")
SKIP_DIRS = {".git", "third_party", "node_modules", "build"}


def md_files(root):
    for d, dirs, files in os.walk(root):
        dirs[:] = [x for x in dirs if x not in SKIP_DIRS and not x.startswith("build")]
        for f in files:
            if f.endswith(".md"):
                yield os.path.join(d, f)


def broken_links(root):
    out = []
    for path in md_files(root):
        with open(path, encoding="utf-8") as fh:
            text = fh.read()
        # fenced code is not markdown
        text = re.sub(r"```.*?```", "", text, flags=re.S)
        for n, line in enumerate(text.splitlines(), 1):
            for target in LINK.findall(line):
                if re.match(r"^[a-z]+:", target) or target.startswith("#"):
                    continue
                file_part = target.split("#", 1)[0]
                if not file_part:
                    continue
                full = os.path.normpath(os.path.join(os.path.dirname(path), file_part))
                if not os.path.exists(full):
                    out.append(f"{os.path.relpath(path, root)}: {target}")
    return out


def non_ascii(files):
    out = []
    for path in files:
        with open(path, encoding="utf-8") as fh:
            for n, line in enumerate(fh, 1):
                bad = [c for c in line if ord(c) > 127]
                if bad:
                    out.append(f"{path}:{n}: {''.join(bad)!r}")
    return out


def main():
    args = sys.argv[1:]
    root = args[0]
    ascii_files = args[args.index("--ascii") + 1:] if "--ascii" in args else []
    links = broken_links(root)
    chars = non_ascii(ascii_files)
    for line in links:
        print("broken link:", line)
    for line in chars:
        print("non-ascii:", line)
    print(f"broken links {len(links)}, non-ascii lines {len(chars)}, "
          f"ascii-checked files {len(ascii_files)}")
    return 1 if links or chars else 0


if __name__ == "__main__":
    sys.exit(main())
