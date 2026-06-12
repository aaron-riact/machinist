from pathlib import Path
from typing import Any
from dataclasses import dataclass, field

@dataclass(frozen=True)  # frozen=True makes it immutable and safer to pass around
class MachinistOptions:
    """
    Systemic Refactor: Replace raw configuration dict parsing inside class __init__ methods with typed dataclasses.

    Context: 
    Our codebase frequently passes a raw `dict[str, Any]` named `options` or `config` into constructors, then handles type fallbacks and key parsing inline. This is brittle and introduces edge-case type bugs.

    Task:
    Scan the workspace for class constructors (`__init__`) accepting a configuration dictionary. For each instance:

    1. Create an immutable, type-hinted `@dataclass(frozen=True)` to represent the configuration schema.
    2. Implement a `from_dict(cls, data: dict | None)` factory method on the dataclass to handle type normalization (e.g., converting fallback values, parsing strings to Paths, wrapping single values into lists).
    3. Update the class constructor signature to accept this new dataclass instead of a raw dictionary.
    4. Locate all call sites where this class is instantiated, and wrap the passed dictionary input with `YourDataClass.from_dict(raw_dict)`.

    Rules:
    - Keep the core business logic inside __init__ exactly the same.
    - Do not introduce external dependencies like Pydantic unless requested.
    - Ensure all new dataclass fields have proper type hints.
    """
    doors: list[str] = field(default_factory=lambda: ["main"])
    program_folder: Path | None = None

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "MachinistOptions":
        """Safely parses raw payloads, handling missing or malformed keys."""
        if not data:
            return cls()
            
        doors = data.get("doors", ["main"])
        if isinstance(doors, str):
            doors = [doors]
            
        folder = data.get("program_folder")
        return cls(
            doors=list(doors), 
            program_folder=Path(folder) if folder else None
        )

# --- Your Class Definition ---

def __init__(
    self, name: str, endpoint: Endpoint, bus: EventBus, options: MachinistOptions
) -> None:
    super().__init__(name, endpoint, bus)
    
    # The constructor is now completely free of dictionary-parsing logic
    self.state = MachineState()
    for d in options.doors:
        self.state.doors[d] = Toggle(name=d)

    # Clean, safe path fallback using the pre-parsed option
    self.program_path = (
        options.program_folder.expanduser() if options.program_folder
        else Path.cwd() / ".machinist_programs" / name
    )



