"""The launch vehicle: a stack of stages, its propulsion and its drag.

Nothing here changes as the flight proceeds. The propellant burned is carried
by the integrator instead, so any of these quantities can be evaluated at a
trial point without the vehicle remembering that it was asked.
"""

import math
from bisect import bisect_right
from dataclasses import dataclass, field

from .atmosphere import Air
from .constants import SEA_LEVEL_PRESSURE, STANDARD_GRAVITY

# altitude above which the model takes the air as gone, m
DRAG_CEILING = 100_000


def _interpolated(mach: float, points, values, slopes) -> float:
    """Linear interpolation on a table, term for term what `np.interp` gives.

    Written out because the drag is asked for one Mach number at a time, four
    times an integration step: numpy interpolates an array faster than this and
    a single number slower, and the slopes are worked out once when the vehicle
    is built rather than at every call.
    """
    if not slopes:
        # a table of one point is that number at every Mach: no interval to
        # interpolate over, and nothing outside it either
        return values[0]
    if mach <= points[0]:
        return values[0]
    if mach >= points[-1]:
        return values[-1]
    interval = bisect_right(points, mach) - 1
    # a Mach that is not a number compares false against both ends above and
    # would walk off the table here; numpy carries the NaN through instead
    if interval >= len(slopes):
        interval = len(slopes) - 1
    return slopes[interval] * (mach - points[interval]) + values[interval]


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

    def propulsion(self, pressure: float, throttle: float) -> tuple[float, float]:
        """Thrust (N) and propellant flow (kg/s) together, as above.

        Together because the flow is the thrust over the exhaust speed, so
        asking for the two of them separately works the thrust out twice - and
        that is four times an integration step for the length of a flight.
        """
        thrust = self.thrust(pressure)
        return (thrust * throttle,
                max(0.0, thrust) * throttle
                / (self.specific_impulse(pressure) * STANDARD_GRAVITY))


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
        # the drag table as two sorted rows, with the slope of every interval
        # worked out once: see `_interpolated` above
        mach = sorted(self.drag_coefficient)
        self._mach_points = tuple(float(m) for m in mach)
        self._cd_values = tuple(float(self.drag_coefficient[m]) for m in mach)
        self._cd_slopes = tuple(
            (self._cd_values[i + 1] - self._cd_values[i])
            / (self._mach_points[i + 1] - self._mach_points[i])
            for i in range(len(mach) - 1))
        # what the flow sees, from each stage upwards: the widest stage still
        # on the vehicle. Ariane 62 and H3 are as wide as their boosters side
        # by side, but only until those boosters go
        areas, widest = [], 0.0
        for stage in reversed(self.stages):
            widest = max(widest, stage.diameter)
            areas.append(math.pi * widest ** 2 / 4)
        self.frontal_areas = tuple(reversed(areas))
        # mass of the stack from each stage up, with full tanks. Fixed for the
        # flight, and `mass_on` below is asked for it some five times a step
        self._stack_masses = tuple(
            sum(s.dry_mass + s.propellant_mass for s in self.stages[index:])
            for index in range(len(self.stages)))
        self._capacities = tuple(s.propellant_mass for s in self.stages)
        self._ignitions = tuple(s.ignition_time for s in self.stages)
        # a vehicle with no drag profile flies without drag, and the check for
        # it is worth making once rather than at every trial point
        self._has_drag = bool(self._mach_points)

    def active_stage(self, t: float) -> tuple[int, Stage]:
        """The stage flying at this instant, and where it sits in the stack.

        Before the first ignition it is the first stage: a vehicle sitting on
        the pad is the whole of itself. Asked twice an integration step, so the
        instants are searched rather than walked.
        """
        index = bisect_right(self._ignitions, t) - 1
        if index < 0:
            index = 0
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
        return self._stack_masses[index] \
            - min(propellant_burned, self._capacities[index])

    def drag(self, air: Air, altitude: float, speed: float, index: int) -> float:
        """Aerodynamic drag on the stack from `index` upwards, N.

        Taken as zero above `DRAG_CEILING`. A vehicle given no drag profile flies
        without drag, rather than through an interpolation with nothing to
        interpolate between.
        """
        if altitude > DRAG_CEILING or not self._has_drag:
            return 0.0
        cd = _interpolated(speed / air.speed_of_sound, self._mach_points,
                           self._cd_values, self._cd_slopes)
        return cd * air.density * speed**2 / 2 * self.frontal_areas[index]

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
