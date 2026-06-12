def __init__(
    self, name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any],
) -> None:
    super().__init__(name, endpoint, bus)
    opts = options or {}

    self.state = MachineState()
    for d in opts.get("doors") or ["main"]:
        self.state.doors[d] = Toggle(name=d)

    folder = opts.get("program_folder")
    root = Path(folder).expanduser() if folder else (
        Path.cwd() / ".machinist_programs" / name
    )


def __init__():
    if "ethernetip" in self._interfaces:
        self._ethernetip = _build_ethernetip_transport(endpoint, opts)
        print("ETHERNETIP", self._ethernetip)

def _build_ethernetip_transport(
    endpoint: Endpoint, options: dict[str, Any]
) -> EtherNetIPAdapter | EtherNetIPScanner:
    config = options.get("ethernetip")
    if config is None:
        config = {}
    if not isinstance(config, dict):
        raise ValueError("ethernetip options must be a mapping")
    mode = _ethernetip_mode(options)
    if mode == "scanner":
        return _build_scanner(config, options)
    if mode == "adapter":
        return _build_adapter(endpoint, config)
    raise ValueError("ethernetip.mode must be either 'adapter' or 'scanner'")

