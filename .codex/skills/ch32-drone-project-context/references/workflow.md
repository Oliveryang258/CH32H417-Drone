# Codex And Claude Code Workflow

Use one owner per task.

Recommended task header:

```text
Owner: Codex
Reviewer: Claude Code
Task: Add V3F 0xDD overcurrent disarm.
Scope: V3F/User/main.c only unless inspection proves otherwise.
Hardware action: none.
```

Owner responsibilities:

- Inspect local code before editing.
- Keep changes narrow.
- Run available static checks or builds.
- Summarize changed files, risk, and hardware validation steps.

Reviewer responsibilities:

- Review the diff and safety behavior.
- Do not edit the same files unless the user reassigns ownership.
- Focus on missed edge cases, protocol mismatches, and test gaps.

Hardware rule:

- AI may prepare checklists and expected observations.
- Human performs flashing, motor spin, prop-on tests, and power-system actions.
