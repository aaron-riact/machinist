"""The fleet as a reduction of events.

Devices own their state and announce every change as a typed event. This
package folds those events into one immutable :class:`FleetState` that the
TUI and the web UI render. Nothing here reaches into a device after the
initial seed; the event stream is the only input.
"""

from .reduce import Projection, reduce, replay, seed
from .views import DeviceView, FleetState, SignalView

__all__ = ["DeviceView", "FleetState", "Projection", "SignalView", "reduce", "replay", "seed"]
