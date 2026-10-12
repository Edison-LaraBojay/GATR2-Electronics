---
name: feedback-comment-style
description: "User's explicit rules for how code comments must read on GATR2 — terse, self-describing, no fancy symbols, not AI-ish"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 0693582d-d417-490c-8cb4-1a0c07d0b810
  modified: 2026-07-19T18:34:57.920Z
---

Explicit rules from the user (stated directly, 2026-07-19, NOT derived from the context doc) for how comments are written in this project.

**Rules:**
- No emojis in comments.
- No em dashes in comments.
- No symbol in comments that is not directly on a standard keyboard (e.g. no en/em dash, no smart quotes, no arrows). Plain ASCII only.
- Avoid symbols in general unless they carry real meaning. `*_gtest.cpp` is fine because it means something. Ordinary punctuation like `;` `:` `-` is allowed but use it only when it helps; do not reach for it.
- Do NOT be hyper formal about grammar unless explicitly asked. Casual, plain phrasing is fine.
- Comments should NOT sound AI-ish. They should read as self-describing (what the thing is / what is needed), not as an explanation delivered to the reader. Do not over-explain. Do not narrate rationale the code already makes obvious.
- Formats vs requirements: state a format only where it is not already self-evident. Example: a comment saying "every file here is named *_gtest.cpp" is unnecessary once more files exist and make the pattern obvious; that kind of standard belongs in docs, not repeated in comments.

**Why:** User wants comments that a maintainer reads as terse, human, self-describing signposts, not verbose AI narration or rationale dumps. Overexplaining and formal grammar both read as AI-generated and as noise.

**How to apply:** Write short comments that say what a thing is or what to do, not why-in-detail. Prefer fewer words. Skip any comment that just restates obvious code or a pattern that is self-evident from surrounding files. Plain ASCII, casual grammar ok. When in doubt, cut it.

Calibration on terseness (user pushed twice to cut further): a bare label is the target. `# Test files.` and `# Sources under test.` are right. Even "add each one here" or "added as they land" was still too much (reads as AI narrating an instruction). Aim for a noun-phrase label, not a sentence, not an instruction.

See [[feedback-conventions]], [[feedback-design-principles]].
