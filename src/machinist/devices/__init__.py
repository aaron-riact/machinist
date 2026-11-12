"""Built-in device implementations.

Importing this package registers all bundled devices in the default
registry. Custom devices can be loaded by importing further entry points.
"""

from . import (  # noqa: F401  (registration side-effect)
    gateways,
    grippers,
    io_controllers,
    machines,
    robots,
    sensors,
)
