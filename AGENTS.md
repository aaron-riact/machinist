# Project Guidelines

## Change Scope

- Make one logical change at a time.
- Keep commits very small and tightly focused.
- Do not mix unrelated fixes, refactors, docs, or UI changes into the same commit.
- If a second issue is discovered while working, finish the current change first and handle the new issue in a separate commit unless it is a direct blocker.
- When the user asks for sequential work, apply edits serially rather than batching multiple independent changes together.

## Commit Discipline

- Before committing, review the staged diff and confirm every staged file belongs to the same logical change.
- Prefer a follow-up `--fixup` commit for mistakes in an earlier commit rather than folding unrelated cleanup into the next change.
- Leave unrelated working tree changes unstaged.

## Validation

- Reproduce the specific issue before claiming it is fixed.
- After each focused change, run the smallest relevant validation for that change before moving to the next one.
- Only run broader validation once the current focused slice is complete.

## Workflow Expectations

- For behavior changes, update tests in the same focused commit as the code they verify.
- For documentation changes, keep them in their own commit unless the user explicitly wants docs bundled with code.
- If a request includes process constraints, follow those constraints over the default urge to batch work for speed.
