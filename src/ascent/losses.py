"""The velocity budget of the ascent: where the propellant went.

The ideal velocity a vehicle could reach is spent on three things besides the
orbit itself - lifting against gravity, pushing through the air, and pointing
the thrust away from the velocity in order to fly the programme. The third one
is what a pitch programme is judged by, and the first one is what it trades
against: a flatter turn steers less but climbs longer.

All three are integrated over the powered part of the flight only, which ends
at the last instant the engines were producing thrust.
"""

from dataclasses import dataclass

import numpy as np

from .constants import MU
from .telemetry import Telemetry


@dataclass(frozen=True)
class VelocityBudget:
    # velocity spent climbing out of the gravity well, m/s
    gravity: float
    # velocity spent against aerodynamic drag, m/s
    aerodynamic: float
    # velocity spent deflecting the thrust to fly the programme, m/s
    steering: float
    # instant the engines stopped, s
    burnout_time: float

    @property
    def total(self) -> float:
        return self.gravity + self.aerodynamic + self.steering


def velocity_budget(telemetry: Telemetry, omega: float) -> VelocityBudget:
    """What the propellant bought and what it was spent on, m/s.

    `omega` is the Earth rotation projected on to the launch plane, the same
    figure the flight was integrated with. Asked for rather than defaulted: a
    zero would quietly cost a launch to the east some hundreds of metres per
    second of gravity loss.
    """
    powered = np.flatnonzero(telemetry.thrust > 0.0)
    if not len(powered):
        # nothing was ever spent, so nothing was spent on anything. Integrating
        # the whole flight instead would report the gravity and the drag of a
        # pure coast as losses, and its last instant as a burnout that never was
        return VelocityBudget(0.0, 0.0, 0.0, float(telemetry.t[0]))
    end = int(powered[-1]) + 1

    t = telemetry.t[:end]
    radius = telemetry.radius[:end]
    sin_angle = np.sin(np.radians(telemetry.flight_path_angle[:end]))
    # gravity less the centrifugal term of the rotating frame, which is what the
    # along-track equation of motion carries and so what the budget is spent on
    effective_gravity = MU / radius**2 - omega**2 * radius

    return VelocityBudget(
        gravity=float(np.trapezoid(effective_gravity * sin_angle, t)),
        aerodynamic=float(np.trapezoid(telemetry.drag[:end] / telemetry.mass[:end], t)),
        steering=float(telemetry.steering_loss[end - 1]),
        burnout_time=float(t[-1]),
    )
