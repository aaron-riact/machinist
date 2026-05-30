"""Minimal MTConnect HTTP agent.

MTConnect's current-document XML is verbose, but the two endpoints
machinist needs are tiny:

* ``GET /probe``    — describes the device (devices + data items)
* ``GET /current``  — returns the current values of those data items

We build both from a :class:`MachineState`, exposing door toggles,
cycle state and DPRINT log as standard MTConnect events/conditions.
Clients that only inspect the root document see a fully-formed
MTConnect Streams or Devices XML response.

This is intentionally small (~100 lines): the point is to satisfy a
monitoring client's initial probe/current poll, not to be a fully-
compliant agent. Where agents like cppagent spend thousands of lines
on buffer/asset/history logic, we just echo current state.
"""

from __future__ import annotations

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable


class MTConnectAgent:
    """HTTP server that renders a :class:`MachineState` as MTConnect XML."""

    def __init__(self, host: str, port: int, render: Callable[[str], str]) -> None:
        self._host = host
        self._port = port
        self._render = render
        self._server: ThreadingHTTPServer | None = None

    def serve_forever(self, ready: threading.Event | None = None) -> None:
        render = self._render

        class _Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                endpoint = self.path.strip("/").lower() or "current"
                if endpoint not in {"probe", "current"}:
                    self.send_error(404, "unknown endpoint")
                    return
                body = render(endpoint).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/xml; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, format: str, *args) -> None:  # noqa: A002
                pass

        self._server = ThreadingHTTPServer((self._host, self._port), _Handler)
        if ready is not None:
            ready.set()
        self._server.serve_forever()

    def shutdown(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()


def render_mtconnect(state, endpoint: str) -> str:  # type: ignore[no-untyped-def]
    """Render a :class:`MachineState` as MTConnect XML (probe or current)."""
    if endpoint == "probe":
        return _render_probe(state)
    return _render_current(state)


def _render_probe(state) -> str:  # type: ignore[no-untyped-def]
    items = [
        '<DataItem id="execution" category="EVENT" type="EXECUTION"/>',
        '<DataItem id="program" category="EVENT" type="PROGRAM"/>',
        '<DataItem id="tool" category="EVENT" type="TOOL_NUMBER"/>',
        '<DataItem id="parts" category="EVENT" type="PART_COUNT"/>',
        '<DataItem id="spindle" category="SAMPLE" type="ROTARY_VELOCITY"'
        ' units="REVOLUTION/MINUTE"/>',
        '<DataItem id="feed" category="SAMPLE" type="PATH_FEEDRATE"'
        ' units="MILLIMETER/MINUTE"/>',
    ]
    for name in state.doors:
        items.append(f'<DataItem id="door_{name}" category="EVENT" type="DOOR_STATE"/>')
    data_items = "".join(items)
    return (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<MTConnectDevices>'
        '<Devices>'
        f'<Device id="machinist" name="machinist">'
        f'<DataItems>{data_items}</DataItems>'
        '</Device>'
        '</Devices>'
        '</MTConnectDevices>'
    )


def _render_current(state) -> str:  # type: ignore[no-untyped-def]
    events = [
        f'<Execution dataItemId="execution">{state.cycle.value.upper()}</Execution>',
        f'<Program dataItemId="program">{state.program or "NONE"}</Program>',
        f'<ToolNumber dataItemId="tool">{state.tool}</ToolNumber>',
        f'<PartCount dataItemId="parts">{state.parts}</PartCount>',
    ]
    for name, door in state.doors.items():
        events.append(
            f'<DoorState dataItemId="door_{name}">'
            f'{"OPEN" if door.open else "CLOSED"}'
            '</DoorState>'
        )
    events_xml = "".join(events)
    samples = (
        f'<RotaryVelocity dataItemId="spindle">{state.spindle_rpm:g}</RotaryVelocity>'
        f'<PathFeedrate dataItemId="feed">{state.feed:g}</PathFeedrate>'
    )
    return (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<MTConnectStreams>'
        '<Streams>'
        '<DeviceStream name="machinist">'
        '<ComponentStream component="Controller">'
        f'<Events>{events_xml}</Events>'
        f'<Samples>{samples}</Samples>'
        '</ComponentStream>'
        '</DeviceStream>'
        '</Streams>'
        '</MTConnectStreams>'
    )
