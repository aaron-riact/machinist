# ◇ Machinist

![machinist](./machinist.png "Machinist")

A composable, modern Python framework for **emulating fleets of
industrial machines** — robot arms, CNCs, grippers, IO controllers — so
your shop-floor software can be developed and tested without bringing
expensive iron to a stand-still.

```
+-------------------------------------------------------------+
|  ◇ Machinist  ─ a fleet of N device(s)                      |
+---------------+---------------------------------------------+
| devices table |     selected device detail                  |
+---------------+---------------------------------------------+
| event log (all devices) — scrolling                         |
+-------------------------------------------------------------+
| ◇ command bar (estop ur1 / set io1.o5 1 / …)                |
+-------------------------------------------------------------+
```

## Highlights

* **Pure-Python, single binary** (`uv run machinist`) — no Docker, no
  PLC simulators, no vendor SDKs required for the core flow.
* **Declarative YAML scenes** describing whole cells (robots, grippers,
  IO controllers, machines) including IO wiring between them.
* **Tiny native protocol stacks** (Modbus/TCP, S7 stub, IO-Link HTTP,
  generic line protocols) — heavy vendor libraries (`pymodbus`,
  `python-snap7`, `aiohttp`, `smbprotocol`) remain *optional* extras.
* **One worker thread per device, agnostic to threads vs. asyncio vs.
  simpy**. Each device decides its own concurrency model.
* **Pluggable kinematics** (`noop` ships; `pinocchio`, `pykdl`, `ik-geo`
  plug in via `kinematics.api.register_backend`).
* **Live Textual TUI in a "Claude-code" style** — devices grid, signal
  panel, event log, and a modal command bar.
* **Composable, not extendable**: the abstract base classes
  (`Device`, `LineServerDevice`, `RobotArm`, `MachineState`) own the
  cross-cutting behaviour; per-vendor modules are 50–100 LoC each.

## Install / develop

```bash
uv sync             # install runtime + dev deps
uv run pytest -q    # run tests (sub-second)
uv run machinist kinds   # list registered device kinds
```

Optional protocol back-ends:

```bash
uv pip install -e ".[modbus,s7,http,smb,kinematics]"
```

## Running a scene

```bash
uv run machinist run examples/scene.yaml
# or, for headless / CI:
uv run machinist run examples/scene.yaml --no-tui
```

Override individual devices on the command line (handy for one-off
testing without editing YAML):

```bash
uv run machinist run examples/scene.yaml \
    -d ur_dashboard:ur_extra:127.0.0.5:30099
```

## YAML schema

```yaml
devices:
  - name: ur1                  # unique
    kind: ur_dashboard         # one of `machinist kinds`
    host: 127.0.0.1            # optional; defaults to 127.0.0.1
    port: 29999                # optional; defaults to the device's standard port
    options: {joint_count: 6}

io_links:
  - {source: io1.o5, target: gripper1.cmd_open}
```

When two devices want the same `host:port`, Machinist:

* **bumps the host along the loopback range** (`127.0.0.1` →
  `127.0.0.2`) if neither device pinned its host explicitly,
* **fails fast** if a host *was* pinned (silent reassignment is *worse*
  than a noisy error in industrial settings).

## Bundled devices

| Kind                | Protocol         | Default port | Notes                              |
| ------------------- | ---------------- | ------------ | ---------------------------------- |
| `ur_dashboard`      | TCP text         | 29999        | UR Dashboard (ur-rtde compatible)  |
| `motoman_nx100`     | TCP text (HSE)   | 80           | Yaskawa NX100/DX100 telnet         |
| `dobot_dashboard`   | TCP text         | 29999        | Dobot Verb(args); reply pattern    |
| `fanuc_r30ib`       | TCP text         | 18735        | fanucpy / FaRoC compatible verbs   |
| `haas_ngc`          | TCP text (MDC)   | 5051         | Q-queries; DPRINT log              |
| `mazak_840d`        | S7 (stub)        | 102          | Configurable DB/byte/bit mappings  |
| `pneumatic_gripper` | IO only          | n/a          | open/close + limit switches        |
| `onrobot_3fg25`     | Modbus/TCP       | 502          | Diameter, force, grip command      |
| `zimmer_ged6000il`  | IO-Link HTTP     | 80           | Emulates IFM AL1350 master         |
| `weidmuller_ur20`   | Modbus/TCP       | 502          | Configurable I/O width             |

## Architecture (one screenful)

```
machinist/
├── core/              ← framework spine, no vendor knowledge
│   ├── device.py            ← Device ABC, lifecycle, ready signal
│   ├── line_device.py       ← LineServerDevice convenience base
│   ├── events.py            ← thread-safe pub/sub
│   ├── io.py                ← Signal / SignalBank / IOMap
│   ├── addressing.py        ← AddressAllocator (loopback bump)
│   ├── registry.py          ← @register decorator, default_registry
│   ├── config.py            ← pydantic SystemConfig + YAML loader
│   └── world.py             ← WorldBuilder, World context manager
├── transport/
│   ├── line_server.py        ← reusable threaded line protocol server
│   ├── modbus_server.py      ← native MBAP/TCP slave (FC 03/06/16)
│   ├── s7_server.py          ← S7Store + stub listener
│   └── iolink_http_master.py ← stdlib HTTP gateway emulating AL1350
├── kinematics/
│   └── api.py                ← Kinematics Protocol + NoOpKinematics
├── devices/
│   ├── robots/        ← UR, Motoman, Dobot, Fanuc — all share arm.py
│   ├── machines/      ← HAAS, Mazak — all share state.py + gcode.py
│   ├── grippers/      ← OnRobot, Zimmer, Pneumatic
│   └── io_controllers/← Weidmuller UR20
├── tui/app.py         ← Textual UI, command bar, live signal panel
└── cli.py             ← Typer CLI (run, kinds, version)
```

## Adding a new device kind

1. Subclass `Device`, `LineServerDevice`, or wrap an existing transport.
2. If your device has IO, expose `self.io: SignalBank` in `__init__`.
3. Decorate the factory with `@register("my_kind", default_port=12345)`.
4. Import the module from `devices/<category>/__init__.py`.

That's it — `WorldBuilder` will instantiate, allocate an endpoint,
adopt the IO bank, and the TUI will show it.

## Roadmap

* 3D web visualiser (the architecture is intentionally event-driven so
  adding a websocket bridge is purely additive).
* Full S7 wire protocol (current shipped server is a stub that parks
  TCP connections; the `S7Store` already models bit-level R/W).
* SMB share abstraction with `pysmb`/`smbprotocol`/`impacket`/`aiosmb`
  back-ends.
* MTConnect, DPRINT and HAAS SMB sub-services (HAAS NGC currently ships
  the MDC interface only).

## License

MIT.
