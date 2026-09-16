from __future__ import annotations

from machinist.core.events import EventBus
from machinist.core.io import SignalBank
from machinist.core.panel import Field, Panel
from machinist.core.types import Endpoint
from machinist.devices.grippers.pneumatic import PneumaticGripper, PneumaticGripperOptions


def _gripper() -> PneumaticGripper:
    return PneumaticGripper(
        "g1", Endpoint("127.0.0.1", 0), EventBus(),
        PneumaticGripperOptions(settle_seconds=0.05),
        io=SignalBank(owner="g1"),
    )


def test_build_detail_lists_every_signal_as_a_field() -> None:
    g = _gripper()
    detail = g.build_detail()
    names = {f.name for f in detail.input_fields + detail.output_fields}
    assert names == {"cmd_open", "cmd_close", "is_open", "is_closed"}


def test_build_detail_includes_input_and_output_fields() -> None:
    g = _gripper()
    detail = g.build_detail()
    in_names = {f.name for f in detail.input_fields}
    out_names = {f.name for f in detail.output_fields}
    assert in_names == {"cmd_open", "cmd_close"}
    assert out_names == {"is_open", "is_closed"}


def test_build_detail_returns_device_detail_shape() -> None:
    g = _gripper()
    detail = g.build_detail()
    assert isinstance(detail, Panel)
    assert detail.mode == "io"
    assert detail.transport_ready is True
    assert detail.peer_connected is True
    assert detail.clients is None
    assert detail.input_block_hex == ""
    assert detail.output_block_hex == ""
    assert detail.derived_fields == ()


def test_build_detail_io_field_has_required_keys() -> None:
    g = _gripper()
    detail = g.build_detail()
    for field in detail.input_fields + detail.output_fields:
        assert isinstance(field, Field)
        assert field.type == "bit"


def test_build_detail_reflects_signal_values() -> None:
    g = _gripper()
    g.io["cmd_open"].set(True)
    g.io["is_open"].set(True)
    detail = g.build_detail()
    for field in detail.input_fields:
        if field.signal == "CMD_OPEN":
            assert field.value == "ON"
        elif field.signal == "CMD_CLOSE":
            assert field.value == "OFF"
    for field in detail.output_fields:
        if field.signal == "IS_OPEN":
            assert field.value == "ON"
        elif field.signal == "IS_CLOSED":
            assert field.value == "OFF"


