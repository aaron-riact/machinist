"""Reusable network transport primitives.

These are deliberately tiny: a device that just wants to "listen on a
TCP port and exchange newline-terminated text" can subclass
:class:`LineServer` instead of touching sockets directly. More exotic
protocols (Modbus, S7, HTTP) plug in via their own adapters under the
same package.
"""
