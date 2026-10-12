# Trial only: after we_apply_proto.py, fold state/landmark_error.h into
# state/field_state.h (FINAL.md correction W1). ae457ee layout.
#
#   py -3 we_fold_trial.py <scratch pi/naviGATR root>

import os
import re
import sys


def rd(p):
    with open(p, encoding="utf-8", newline="") as f:
        return f.read()


def wr(p, s):
    with open(p, "w", encoding="utf-8", newline="") as f:
        f.write(s)


def once(text, old, new, what):
    if text.count(old) != 1:
        sys.exit("%s: expected 1 match of %r" % (what, old))
    return text.replace(old, new)


def main():
    nav = sys.argv[1]
    le = os.path.join(nav, "src", "state", "landmark_error.h")
    fs = os.path.join(nav, "src", "state", "field_state.h")
    body = rd(le)
    block = body.split("namespace navigatr\n{\n\n", 1)[1].rsplit("} // namespace navigatr", 1)[0]
    s = rd(fs)
    s = once(s, '#include "state/landmark_error.h"\n', "", "field_state.h")
    s = once(s, "#pragma once\n#include <cstdint>\n", "#pragma once\n#include <cmath>\n#include <cstdint>\n",
             "field_state.h")
    s = once(s, '#include "math/transforms.h"\n',
             '#include "math/angles.h"\n#include "math/transforms.h"\n#include "math/units.h"\n',
             "field_state.h")
    s = once(s, "struct FieldObjectState {", block + "struct FieldObjectState {", "field_state.h")
    wr(fs, s)
    os.remove(le)
    n = 0
    for top in ("src", "tests"):
        for root, _, files in os.walk(os.path.join(nav, top)):
            for fn in files:
                p = os.path.join(root, fn)
                if not fn.endswith((".h", ".cpp")):
                    continue
                t = rd(p)
                if '#include "state/landmark_error.h"' in t:
                    if '#include "state/field_state.h"' in t:
                        t = t.replace('#include "state/landmark_error.h"\n', "")
                    else:
                        t = t.replace('#include "state/landmark_error.h"',
                                      '#include "state/field_state.h"')
                    wr(p, t)
                    n += 1
    print("folded; includers updated:", n)


if __name__ == "__main__":
    main()
