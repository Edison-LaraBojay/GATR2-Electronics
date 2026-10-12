---
name: project-coding-standard
description: 2026-09-28 drafting a comprehensive GATR2 coding standard (.md) to follow and then apply system-wide; CI gets rebuilt around it; review deferred in favor of this
metadata:
  type: project
---
2026-09-28: the user replaced the "preliminary whole-system code review" with drafting a big coding-rules Markdown the team will follow, then apply to the entire codebase.

It targets bugs, overcomplicated code and systems, readability (`using` aliases liberally, named structs), and industry-grade docs.

Draft and inputs live at `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/coding-standard/`: requirements.md, survey/, critique/, CODING_STANDARD.md.

The current CI workflows are not a constraint; CI will be rebuilt around the standard.

The spectuGATR design work continues in parallel ([[project-spectugatr]]).

**Why:** the user thinks rules first, then applying them, is a better use of time than an ad hoc review.

**How to apply:** the standard must respect [[feedback-comment-style]] and [[feedback-conventions]], or list any conflict (e.g. -Werror) as a user decision. Applying it to the code happens only after the user reviews it.
