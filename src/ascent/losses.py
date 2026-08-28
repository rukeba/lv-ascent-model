"""The velocity budget of the ascent: where the propellant went.

Two of these the trajectory pays as it flies: lifting against gravity and
pushing through the air. What the propellant delivered, less those two, is the
speed reached, and the projection here is the one the equations of motion carry
so that it adds up. The third is not in that sum: the steering loss is the
price of holding the programme, recovered from the normal equation after the
fact, and it is what a pitch programme is judged by.

All three are integrated from lift-off to the last instant the engines were
producing thrust, a coast between two burns included.

See docs/velocity-budget.md
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
    if not len(telemetry):
        raise ValueError('nothing was recorded: there is no flight to account for')

    powered = np.flatnonzero(telemetry.thrust > 0.0)
    if not len(powered):
        # nothing was ever spent, so nothing was spent on anything: the whole
        # flight would report a coast as losses and a burnout that never was
        return VelocityBudget(0.0, 0.0, 0.0, float(telemetry.t[0]))
    end = int(powered[-1]) + 1

    t = telemetry.t[:end]
    radius = telemetry.radius[:end]
    sin_angle = np.sin(np.radians(telemetry.flight_path_angle[:end]))
    # gravity less the centrifugal term of the rotating frame, which is what
    # the along-track equation carries and so what the budget is spent on
    effective_gravity = MU / radius**2 - omega**2 * radius

    return VelocityBudget(
        gravity=float(np.trapezoid(effective_gravity * sin_angle, t)),
        aerodynamic=float(np.trapezoid(telemetry.drag[:end] / telemetry.mass[:end], t)),
        steering=float(telemetry.steering_loss[end - 1]),
        burnout_time=float(t[-1]),
    )
