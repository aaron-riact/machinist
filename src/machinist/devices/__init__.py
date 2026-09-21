"""Built-in device implementations.

Importing this package registers all bundled devices in the default
registry. Custom devices can be loaded by importing further entry points.
"""

from . import gateways, grippers, io_controllers, machines, robots  # noqa: F401  (registration side-effect)
