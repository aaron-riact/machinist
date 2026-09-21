"""A bare RTU-over-TCP gateway, with a line of its own.

A Dobot or a UR carries an RS485 line on its tool flange and lets the
network at it. Sometimes the line is all you want: a gripper driver
under test does not care whether an arm is bolted to the other end of
the cable, only that RTU frames reach the tool and come back.

So this device is the flange without the arm -- one line, a TCP door (or
several) onto it, and nothing else. Tools are wired on with the same
``flange_links`` an arm uses, because to the line they are the same
thing::

    devices:
      - {name: bridge, kind: modbus_rtu_gateway, port: 60000}
      - {name: rg1, kind: onrobot_rg}
    flange_links:
      - {master: bridge, slave: rg1, slave_id: 65}
"""

from __future__ import annotations

from ...core.capabilities import HasFlange
from ...core.device import Device
from ...core.events import EventBus
from ...core.options import Options
from ...core.panel import Field, Panel
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.flange_bus import FlangeBus
from ...transport.modbus_rtu_gateway import ModbusRtuGateway

#: What a Dobot answers on, and as good a default as any for a bare line.
DEFAULT_GATEWAY_PORT = 60000


class ModbusRtuGatewayOptions(Options):
    """The ``modbus_rtu_gateway`` block: more doors onto the one line."""

    #: Further TCP ports onto the same line, beyond the device's own port.
    extra_ports: tuple[int, ...] = ()


class ModbusRtuGatewayDevice(Device, HasFlange):
    """One RS485 line, reachable over TCP and nothing else."""

    kind = "modbus_rtu_gateway"
    DEFAULT_PORT = DEFAULT_GATEWAY_PORT

    def __init__(
        self,
        name: str,
        endpoint: Endpoint,
        bus: EventBus,
        *,
        flange: FlangeBus,
        gateway: ModbusRtuGateway,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.flange = flange
        self._gateway = gateway
        self.add_service(gateway)

    @property
    def flange_gateway_ports(self) -> tuple[int, ...]:
        """TCP ports that open straight onto the line."""
        return self._gateway.ports

    def build_detail(self) -> Panel:
        """The doors onto the line, and what is sitting on it."""
        return Panel(
            mode="rtu gateway",
            clients=self._gateway.client_count,
            status_fields=(
                Field(
                    signal="ports",
                    name="Listening on",
                    type="str",
                    value=", ".join(str(port) for port in self._gateway.ports),
                ),
                Field(
                    signal="flange",
                    name="Line slaves",
                    type="str",
                    value=self._slaves_detail(),
                ),
            ),
        )

    def _slaves_detail(self) -> str:
        ids = self.flange.slave_ids
        return ", ".join(f"0x{slave_id:02X}" for slave_id in ids) if ids else "-"


@register(
    "modbus_rtu_gateway",
    default_port=DEFAULT_GATEWAY_PORT,
    options=ModbusRtuGatewayOptions,
)
def _factory(
    name: str, endpoint: Endpoint, bus: EventBus, options: ModbusRtuGatewayOptions
) -> Device:
    ports = (endpoint.port, *options.extra_ports)
    flange = FlangeBus()
    return ModbusRtuGatewayDevice(
        name,
        endpoint,
        bus,
        flange=flange,
        gateway=ModbusRtuGateway(host=endpoint.host, ports=ports, line=flange),
    )
