# Refactoring Plan & Progress

## Goal
Architectural cleanup of the Machinist emulation framework: move service instantiation out of constructors into factories (dependency injection), enforce the "Parse, Don't Validate" pattern, and confine dict access to the boundary layer.

---

## Phase 1 — Typed Options Dataclasses

Define a typed `*Options` dataclass for every device, replacing raw dict access in constructors.

- Each device gets a `@dataclass(frozen=True, slots=True)` options class (e.g. `HaasNGCOptions`, `MazakSmoothXOptions`)
- Factories parse `dict[str, Any] → *Options` at the boundary
- Constructors receive the typed object instead of calling `.get()` / `[]` on a dict

**Files affected:** All 12 device files + 5 `_factory` functions.

---

## Phase 2 — Move Fallback Logic Out of Constructors

Normalise options (defaults, fallbacks) in the factory, not the constructor.

- Any `options.get("key", default_value)` or `if key not in options` pattern moved to the factory's `*Options(...)` construction
- Constructors assume pre-normalised typed data

---

## Phase 3 — Service Injection via Factory

Move heavy-service instantiation out of device constructors into factory functions.

Two injection strategies:

- **Pre-construction injection** (services without device callbacks): `SignalBank`, `S7Store`, `S7Server`, `BroadcastServer`, `MessageServer` are created first and injected via keyword-only args
- **Post-construction assignment** (services needing `self` callbacks): `HoldingRegisterServer`, `MTConnectAgent`, `EtherNetIPAdapter`, `LineServer`, `OpcUaServer` are created after the device and assigned to `device._attr`

**Bonus:** Fixed a copy-paste bug in `FanucKarelServer` (duplicate `SignalBank` creation with bare undefined variable names — would have raised `NameError` at runtime).

**Devices refactored:** `OnRobot3FG25`, `WeidmullerUR20`, `PneumaticGripper`, `ZimmerGED6000IL`, `MazakSinumerik840D`, `MazakSmoothXEmulator`, `HaasNGC`, `RobotDevice`, `FanucKarelServer` (9 devices).

---

## Phase 4 — Enforce Keyword-Only Arguments

Add `*` before `options` in every device `__init__` signature.

```python
# Before
def __init__(self, name, endpoint, bus, options, ...)
# After
def __init__(self, name, endpoint, bus, *, options, ...)
```

Callers must write `Device(..., options=opts, ...)` instead of `Device(..., opts, ...)`, making call sites self-documenting.

**Files affected:** 12 device files + 10 test files.

---

## Phase 5 — Collapse Remaining Dict Paths

Eliminate remaining `dict[str, Any]` access in core functions:

- `build_kinematics()` now accepts `KinematicsOptions` (typed dataclass) instead of `dict[str, Any]`
- `RobotModel.extras: dict[str, Any]` (untyped escape hatch) replaced by `robot_type: str | None`
- Old `_model_from_options`, `_infer_backend(dict)`, `_parse_dh(dict)` removed from `kinematics/api.py`
- Dict-to-`DHParams` parsing moved to `arm.py` boundary (`_parse_dh`)
- `IKGeoKinematics` uses `model.robot_type` instead of `model.extras.get("robot_type")`

**Files affected:** `kinematics/api.py`, `kinematics/ikgeo_backend.py`, `arm.py`, `test_kinematics.py`.

---

## Key Architectural Decisions

1. **Boundary vs Core:** Dict access is permitted only in factory functions (`_factory`) and options-dataclass constructors. Once parsed into a typed object, it never goes back to a dict.

2. **Two injection strategies:** Services needing device callbacks are assigned post-construction (`device._service = ...`); services without callbacks are injected via keyword args.

3. **Post-construction is safe because** services are not started until `device._run()` is called after the factory returns.

4. **The `*` before `options`** in `__init__` signatures forces keyword arguments at call sites, preventing accidental positional mix-ups.

---

## Known Remaining Dict Paths (Out of Scope)

These are lower-priority items identified during the audit but not addressed:

- `gcode.py`: `_apply_tooling(words: dict[str, str])` and `_apply_position(words: dict[str, str])` — core interpreter methods receiving raw dict instead of typed G-code word object
- `iolink_http_master.py`: `IOLinkPort` protocol uses `dict[str, Any]` for process data
- `mazak_smoothx.py`: `_build_adapter_config` / `_build_scanner_config` accept `dict[str, Any] | None`
- `ethernetip.py`: `_enum_value(mapping: dict[str, object], ...)` — untyped mapping parameter
- `ArmOptions.kinematics: dict[str, Any]` is still an untyped dict field in a boundary dataclass
