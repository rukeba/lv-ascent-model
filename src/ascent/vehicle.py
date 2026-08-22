"""The launch vehicle: a stack of stages, its propulsion and its drag.

Nothing here changes as the flight proceeds. The propellant burned is carried
by the integrator instead, so any of these quantities can be evaluated at a
trial point without the vehicle remembering that it was asked.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from .atmosphere import Air
from .constants import SEA_LEVEL_PRESSURE, STANDARD_GRAVITY


@dataclass
class Stage:
    name: str
    # instant this stage takes over, s from lift-off
    ignition_time: float
    dry_mass: float
    propellant_mass: float
    thrust_vacuum: float
    thrust_sea_level: float
    isp_vacuum: float
    # equal to the vacuum figure when the stage only ever burns in vacuum
    isp_sea_level: float | None = None
    length: float = 0.0
    diameter: float = 0.0

    def __post_init__(self) -> None:
        # effective nozzle exit area, from the vacuum-to-sea-level thrust rise
        self.nozzle_area = (self.thrust_vacuum - self.thrust_sea_level) / SEA_LEVEL_PRESSURE

    def thrust(self, pressure: float) -> float:
        """Full-throttle thrust against the given ambient pressure, N."""
        return self.thrust_vacuum - pressure * self.nozzle_area

    def specific_impulse(self, pressure: float) -> float:
        if self.isp_sea_level is None:
            return self.isp_vacuum
        return self.isp_vacuum - (self.isp_vacuum - self.isp_sea_level) \
            * (pressure / SEA_LEVEL_PRESSURE)

    def mass_flow(self, pressure: float, throttle: float) -> float:
        """Propellant consumption at the given throttle, kg/s.

        Takes no account of how much propellant is left: the length of the
        step decides that, and only the caller knows it.
        """
        thrust = max(0.0, self.thrust(pressure)) * throttle
        return thrust / (self.specific_impulse(pressure) * STANDARD_GRAVITY)


@dataclass
class LaunchVehicle:
    name: str
    stages: list[Stage]
    # drag coefficient against Mach number, interpolated linearly
    drag_coefficient: dict[float, float] = field(default_factory=dict)
    # dynamic pressure the airframe is designed for, Pa - reported, not enforced
    design_dynamic_pressure: float | None = None

    def __post_init__(self) -> None:
        self.stages = sorted(self.stages, key=lambda s: s.ignition_time)
        mach = sorted(self.drag_coefficient)
        self._mach = np.array(mach)
        self._cd = np.array([self.drag_coefficient[m] for m in mach])
        self.frontal_area = math.pi * max(s.diameter for s in self.stages) ** 2 / 4

    def active_stage(self, t: float) -> tuple[int, Stage]:
        index = 0
        for i, stage in enumerate(self.stages):
            if t >= stage.ignition_time:
                index = i
        return index, self.stages[index]

    def mass(self, t: float, propellant_burned: float) -> float:
        """Mass still on the vehicle, given what the active stage has burned."""
        index, _ = self.active_stage(t)
        return self.mass_on(index, propellant_burned)

    def mass_on(self, index: int, propellant_burned: float) -> float:
        """The same, for a stage named outright rather than found by time.

        The step is cut at every separation, so the last point of the piece
        below one falls exactly on the ignition above it. Asking by time there
        answers for the stage that has not flown the piece, which is a step
        change in mass inside a step that was cut to avoid exactly that.
        """
        stage = self.stages[index]
        stack = sum(s.dry_mass + s.propellant_mass for s in self.stages[index:])
        return stack - min(propellant_burned, stage.propellant_mass)

    def drag(self, air: Air, altitude: float, speed: float) -> float:
        """Aerodynamic drag, N. Taken as zero above 100 km."""
        if altitude > 100_000:
            return 0.0
        mach = speed / air.speed_of_sound
        cd = float(np.interp(mach, self._mach, self._cd))
        return cd * air.density * speed**2 / 2 * self.frontal_area

    def staging_times_within(self, begin: float, end: float) -> list[float]:
        """Separation instants strictly inside the interval."""
        return [s.ignition_time for s in self.stages if begin < s.ignition_time < end]

    @property
    def lift_off_mass(self) -> float:
        return sum(s.dry_mass + s.propellant_mass for s in self.stages)

    @property
    def payload_mass(self) -> float:
        """The last stage carries no propellant: it is the payload."""
        return self.stages[-1].dry_mass
