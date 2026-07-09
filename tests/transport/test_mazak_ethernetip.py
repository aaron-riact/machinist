from __future__ import annotations

from machinist.transport.ethernetip import (
    EtherNetIPAdapter,
    EtherNetIPAdapterConfig,
    MazakEthernetIPAdapter,
)

from .test_ethernetip_forward_open import (
    _BAD_O2T_PATH,
    _GOOD_PATH,
    _RecordingClient,
    _build_adapter as _build_base_adapter,
    _build_forward_open,
    _status_of_forward_open,
    _status_of_forward_open_ext,
)


def _build_mazak_adapter() -> MazakEthernetIPAdapter:
    # __init__ does not bind sockets, so this is safe without open().
    return MazakEthernetIPAdapter(EtherNetIPAdapterConfig(host="127.0.0.1", port=44818))


def test_mazak_accepts_valid_header_sizes() -> None:
    adapter = _build_mazak_adapter()
    assert _status_of_forward_open(adapter, _build_forward_open(o_t_size=106, t_o_size=102)) == 0x00


def test_mazak_accepts_undersized_o_t() -> None:
    # Lenient: 80 <= 100 + 6 -> accepted (real Mazak returns 0x00).
    adapter = _build_mazak_adapter()
    assert _status_of_forward_open(adapter, _build_forward_open(o_t_size=80, t_o_size=102)) == 0x00


def test_mazak_accepts_undersized_t_o() -> None:
    adapter = _build_mazak_adapter()
    assert _status_of_forward_open(adapter, _build_forward_open(o_t_size=106, t_o_size=80)) == 0x00


def test_mazak_rejects_oversized_o_t() -> None:
    # 120 > 100 + 6 -> rejected.
    adapter = _build_mazak_adapter()
    assert _status_of_forward_open_ext(adapter, _build_forward_open(o_t_size=120, t_o_size=102)) == (0x01, 0x0127)


def test_mazak_rejects_oversized_t_o() -> None:
    adapter = _build_mazak_adapter()
    assert _status_of_forward_open_ext(adapter, _build_forward_open(o_t_size=106, t_o_size=120)) == (0x01, 0x0128)


def test_mazak_rejects_wrong_o_t_connection_point() -> None:
    # Path check is inherited from the base (spec) behaviour.
    adapter = _build_mazak_adapter()
    assert _status_of_forward_open(
        adapter, _build_forward_open(o_t_size=106, t_o_size=102, path=_BAD_O2T_PATH),
    ) == 0x05


def test_mazak_rejects_duplicate_forward_open() -> None:
    adapter = _build_mazak_adapter()
    client = _RecordingClient()
    pkt = _build_forward_open(o_t_size=106, t_o_size=102, serial=1)
    adapter._handle_forward_open(client, pkt)
    assert _status_of_forward_open_ext(adapter, pkt) == (0x01, 0x0100)


def test_mazak_is_a_distinct_class_from_base() -> None:
    assert issubclass(MazakEthernetIPAdapter, type(_build_base_adapter()))
    assert MazakEthernetIPAdapter is not type(_build_base_adapter())


def test_build_adapter_selects_class_by_behaviour() -> None:
    from machinist.devices.machines.mazak_smoothx import _build_adapter

    generic = _build_adapter(EtherNetIPAdapterConfig(host="127.0.0.1", behaviour="generic"))
    assert type(generic) is EtherNetIPAdapter

    mazak = _build_adapter(EtherNetIPAdapterConfig(host="127.0.0.1", behaviour="mazak"))
    assert type(mazak) is MazakEthernetIPAdapter

    # Default behaviour is generic.
    default = _build_adapter(EtherNetIPAdapterConfig(host="127.0.0.1"))
    assert type(default) is EtherNetIPAdapter


def test_mazak_factory_defaults_adapter_behaviour_to_mazak() -> None:
    from machinist.core.events import EventBus
    from machinist.devices.machines.mazak_smoothx import _factory
    from machinist.core.types import Endpoint

    endpoint = Endpoint(host="127.0.0.1", port=44818)
    device = _factory(
        "smooth_eip_adapter", endpoint, EventBus(),
        options={"interfaces": ["ethernetip"], "ethernetip": {"mode": "adapter"}},
    )
    assert device._ethernetip is not None
    # The Mazak device's adapter uses the lenient Mazak behaviour by default.
    assert isinstance(device._ethernetip, MazakEthernetIPAdapter)
