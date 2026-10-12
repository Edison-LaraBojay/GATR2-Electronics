---
name: feedback-codex-authorized
description: "Codex works in this repo with the user's authorization; its restructures (local-only deprecated/, retired communiGATR/translaGATR) are intended and must not be undone"
metadata:
  node_type: memory
  type: feedback
  originSessionId: 827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b
  modified: 2026-10-09T23:42:49.629Z
---

Codex, another agent, edits this repo with the user's authorization. On 2026-10-09 it:
- made `deprecated/` local-only and gitignored;
- moved the PCB designs back under `pcb/`, where they are tracked;
- retired communiGATR and translaGATR;
- rewrote the READMEs to match.

The user confirmed all of it: "Codex made those changes with my authorization ... Don't undo the cleanup. The pinned historical commit is sufficient for the old fallback."

**Why:** I had treated those changes as a conflict with an earlier instruction ("keep the fallback under deprecated"). The user's actual intent was retirement into history plus a local archive.

**How to apply:**
- Changes from Codex or another non-Claude actor in the tree are presumed authorized. Do not revert them.
- If they seem to contradict an instruction, ask the user once and keep working on your own files meanwhile.
- In tracked docs, reference retired code by commit (for example `10c568e`), never by `deprecated/` paths.

Related: [[project-spectagatr-standalone]]
