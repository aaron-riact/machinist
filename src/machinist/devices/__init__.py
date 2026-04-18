"""Built-in device implementations.

Importing this package registers all bundled devices in the default
registry. Custom devices can be loaded by importing further entry points.
"""

from . import grippers, robots, machines, io_controllers  # noqa: F401  (registration side-effect)
