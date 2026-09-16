from __future__ import annotations

import threading

import pytest

from machinist.core.device import Device
from machinist.core.events import EventBus
from machinist.core.options import Options, OptionsError
from machinist.core.registry import DeviceRegistry
from machinist.core.types import Endpoint


class _LampOptions(Options):
    watts: int = 40
    colour: str = "warm"


class _Lamp(Device):
    kind = "lamp"

    def __init__(self, name: str, endpoint: Endpoint, bus: EventBus, options: _LampOptions) -> None:
        super().__init__(name, endpoint, bus)
        self.options = options

    def _run(self, stop: threading.Event) -> None:  # pragma: no cover
        stop.wait()


def _registry() -> DeviceRegistry:
    reg = DeviceRegistry()
    reg.register("lamp", _Lamp, default_port=1, options=_LampOptions)
    return reg


def _create(reg: DeviceRegistry, kind: str, config: dict) -> Device:
    return reg.create(kind, "d1", Endpoint("127.0.0.1", 0), EventBus(), config)


def test_options_are_parsed_once_with_defaults_filled_in() -> None:
    lamp = _create(_registry(), "lamp", {"watts": 60})
    assert isinstance(lamp, _Lamp)
    assert lamp.options == _LampOptions(watts=60, colour="warm")


def test_unknown_keys_are_rejected_and_name_the_kind() -> None:
    with pytest.raises(OptionsError, match=r"lamp: bad options: wats: Extra inputs"):
        _create(_registry(), "lamp", {"wats": 60})


def test_wrong_types_are_rejected_at_the_boundary() -> None:
    with pytest.raises(OptionsError, match="watts"):
        _create(_registry(), "lamp", {"watts": "bright"})


def test_parsed_options_are_frozen() -> None:
    lamp = _create(_registry(), "lamp", {})
    with pytest.raises(Exception):  # noqa: B017 - pydantic's frozen error type
        lamp.options.watts = 1  # type: ignore[misc]


def test_every_kind_names_its_options_model() -> None:
    assert _registry().options_for("lamp") is _LampOptions
    with pytest.raises(TypeError):
        DeviceRegistry().register("nameless", _Lamp)  # type: ignore[call-arg]


def test_every_builtin_kind_registers_an_options_model() -> None:
    import machinist.devices  # noqa: F401  (registration side-effect)
    from machinist.core.registry import default_registry

    for kind in default_registry.kinds():
        assert issubclass(default_registry.options_for(kind), Options), kind
