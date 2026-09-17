from __future__ import annotations

from machinist.core.events import EventBus
from machinist.core.io import Direction, SignalBank
from machinist.core.panel import Panel
from machinist.core.types import Endpoint
from machinist.devices.grippers.pneumatic import PneumaticGripper, PneumaticGripperOptions


def _gripper() -> PneumaticGripper:
    return PneumaticGripper(
        "g1", Endpoint("127.0.0.1", 0), EventBus(),
        PneumaticGripperOptions(settle_seconds=0.05),
        io=SignalBank(owner="g1"),
    )


def test_the_base_panel_is_empty() -> None:
    """Discrete IO reaches the UI as SignalChanged events, not as panel rows."""
    g = _gripper()
    panel = g.build_detail()
    assert panel == Panel()
    assert panel.mode == "io"
    assert panel.input_fields == () and panel.output_fields == () and panel.status_fields == ()


def test_the_signal_bank_is_the_source_of_io() -> None:
    g = _gripper()
    names = {(sig.name, sig.direction) for sig in g.io}
    assert names == {
        ("cmd_open", Direction.INPUT),
        ("cmd_close", Direction.INPUT),
        ("is_open", Direction.OUTPUT),
        ("is_closed", Direction.OUTPUT),
    }
