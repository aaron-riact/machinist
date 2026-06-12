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


## Core Principles ##

### 1. Parse, Don't Validate
You should not just check if data is valid and keep it as a loose type (like a dict or a string). Instead, parse that loose data into a strict, strongly-typed data structure as soon as it enters your system. Once it is an object, the type checker guarantees it is valid, and you never have to check it again.

### 2. Boundary vs. Core (Hexagonal Architecture)

Code should be split into two zones:

* The Boundary: Where the messy, untrusted outside world lives (JSON payloads, raw dictionaries, CLI inputs, environment variables).
* The Core: Where your pure business logic lives.
All data normalization, fallback defaults, and type casting must happen at the Boundary so the Core stays clean.

### 3. Fail-Fast (The Robustness Principle)

The traditional robustness principle says, "Be conservative in what you do, be liberal in what you accept." Late-binding takes this too far by letting bad data slip deep into the application before it blows up. By parsing up front, your system fails fast. If a user passes an integer instead of a path, it crashes immediately at the gateway, rather than hours later during a deep background operation.

### 4. Separation of Concerns (SoC)

A class constructor should be responsible for initializing the object, not for deciding what the fallback directory pattern should be if an option is missing. Moving the dictionary logic out separates "how we configure the machine" from "how the machine operates."


## Good Rules-of-Thumb for this Codebase

* The "One-Hop" Rule: A dictionary payload from the outside world should never travel more than one function hop before being turned into a type-safe object (a Dataclass, Pydantic model, or NamedTuple).
* Dicts are for Transport, Objects are for Logic: Use dict for JSON payloads, API requests, and loading YAML configs. Use objects for everything else. If you are typing my_dict["key"] inside a core class, it is a code smell.
* Enforce Keyword-Only Arguments (*): When you do have multiple configuration options, use the * syntax in your Python signatures. This forces developers to write Config(doors=X) instead of Config(X), making call sites self-documenting.
* No Magic Fallbacks in Core Logic: If a default value is needed (like ["main"]), it belongs in the data schema definition, not buried inside a for loop or an if/else block deep in the code.

* An object should either do work or wire things together, but never both.If a class does work (like managing machine states, managing doors, writing to files), its constructor should only accept ready-to-use dependencies. It should never call complex factory methods or use the new keyword/instantiation for heavy components.
