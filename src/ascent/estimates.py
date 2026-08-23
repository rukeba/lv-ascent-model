"""Two estimates that bound a search without flying anything.

Both come from the dissertation this model was written for, and both answer a
question about a trajectory by quadrature instead of by integrating the
equations of motion.

`analytic_altitude` is the integral of the vertical component of the velocity.
Only that component gains altitude, so the altitude at the end of a programme
is its integral over the ascent; with the speed taken from the Tsiolkovsky
equation stage by stage and the flight-path angle read off the programme
itself, that integral is a quadrature over a curve already tabulated. It says
what orbit a set of parameters is aiming at before the set is flown.

`equivalent_time` is the root of a balance. A circular orbit is an amount of
energy - the speed of the orbit and the work of lifting to it together - and
the propellant buys energy at the rate the Tsiolkovsky equation gives. The
instant at which what has been bought covers what the orbit costs, plus
everything lost on the way, is the ascent time. It says where along the time
axis a cut-off can fall.

Neither stands in for a flight. The first overstates the altitude, by half a
per cent to a fifth across the catalogue; the second understates the time, by
up to a ninth, and by how much depends on the vehicle. Both are accurate
enough to say which flights are worth making, which is all a grid search needs
of them.
"""

import math
from dataclasses import dataclass

import numpy as np

from .constants import EARTH_RADIUS, MU, STANDARD_GRAVITY, circular_velocity
from .pitch import PitchProgramme
from .vehicle import LaunchVehicle

# The loss model of the ascent-time estimate replaces the layered atmosphere
# with an exponential one and the Mach-dependent drag coefficient with a mean
# over the trajectory. Both are coarse on purpose: the term they build is a
# small correction for a vehicle with its stages in a stack, and a term of the
# first order for one that flies with boosters strapped alongside.
MEAN_DRAG_COEFFICIENT = 0.25
SEA_LEVEL_DENSITY = 1.225
DENSITY_SCALE_HEIGHT = 7500.0


@dataclass(frozen=True)
class Burn:
    """One stage's stretch of the flight, as the estimates read it."""
    index: int
    # ignition, the instant the tank is empty, and the instant the next stage
    # takes over. The gap between the last two is a coast
    begin: float
    burn_out: float
    end: float
    # effective exhaust velocity, m/s
    exhaust: float
    # propellant consumption at full throttle, kg/s
    flow: float
    # mass of the stack from this stage up, kg
    start_mass: float

    @property
    def burn_time(self) -> float:
        return self.burn_out - self.begin


def burns(vehicle: LaunchVehicle) -> list[Burn]:
    """The burning stages, each with when it lights, empties and is dropped.

    A stage is dropped when the next one lights, whether or not its tank is
    empty by then. Vacuum figures throughout: these estimates carry no ambient
    pressure, and the deficit it costs low down is one of the reasons they are
    estimates.

    A stage with nothing to burn or nothing to burn it through is not a burn:
    the last stage of every vehicle here is the payload, and both of its
    figures are zero.
    """
    rows = []
    for index, stage in enumerate(vehicle.stages):
        if stage.propellant_mass <= 0.0 or stage.thrust_vacuum <= 0.0 \
                or stage.isp_vacuum <= 0.0:
            continue
        exhaust = stage.isp_vacuum * STANDARD_GRAVITY
        flow = stage.thrust_vacuum / exhaust
        end = (vehicle.stages[index + 1].ignition_time
               if index + 1 < len(vehicle.stages) else math.inf)
        burn_out = min(stage.ignition_time + stage.propellant_mass / flow, end)
        rows.append(Burn(index, stage.ignition_time, burn_out, end,
                         exhaust, flow, vehicle.mass_on(index, 0.0)))
    return rows


def required_velocity(altitude: float) -> float:
    """The characteristic velocity a circular orbit is worth in energy, m/s.

    The kinetic energy of the orbit and the work of lifting to it, folded into
    one speed of the same energy. Unlike the circular speed itself, which falls
    with altitude, this rises - which is why it and not the circular speed is
    the thing a burn has to be asked to cover.
    """
    lift = MU * (1.0 / EARTH_RADIUS - 1.0 / (EARTH_RADIUS + altitude))
    return math.sqrt(circular_velocity(altitude) ** 2 + 2.0 * lift)


def vacuum_time(vehicle: LaunchVehicle, altitude: float) -> float | None:
    """How long the engines would burn if nothing were ever lost, s.

    The lower bound on the ascent time, and of no other use: a vehicle that
    loses nothing does not exist. None when the propellant does not reach it.
    """
    required, gained = required_velocity(altitude), 0.0
    for burn in burns(vehicle):
        rise = burn.exhaust * math.log(
            burn.start_mass / (burn.start_mass - burn.flow * burn.burn_time))
        if gained + rise >= required:
            left = required - gained
            return burn.begin + (burn.start_mass / burn.flow) \
                * (1.0 - math.exp(-left / burn.exhaust))
        gained += rise
    return None


def equivalent_time(vehicle: LaunchVehicle, altitude: float,
                    step: float = 0.1) -> float | None:
    """The instant at which the propellant has bought the orbit, s.

    The balance is the characteristic velocity accumulated so far, less what
    the orbit costs, less everything lost up to that instant. It starts
    negative and rises as long as the thrust acceleration beats the rate of
    loss, which on an ascent it does throughout - so the root is unique and can
    be marched to rather than searched for. The dissertation puts Brent's
    method on the same balance over an adaptive quadrature; marching the
    cumulative integral in flight order finds the same root, and only monotony
    makes either of them safe.

    None when the balance never reaches zero: the orbit is out of reach of this
    vehicle, and that is known before a single trajectory is integrated.
    """
    required = required_velocity(altitude)
    gained = lost = 0.0

    for burn in burns(vehicle):
        # the whole stretch the stage owns, coast included: gravity is lost
        # over a gap between two burns as surely as under thrust
        finish = burn.end if math.isfinite(burn.end) else burn.burn_out
        elapsed = _grid(finish - burn.begin, step)
        # the grid lands on the end of the stretch, so its own step is what the
        # quadrature below has to be given rather than the one asked for
        walked = float(elapsed[1] - elapsed[0])
        burning = np.minimum(elapsed, burn.burn_time)
        mass = burn.start_mass - burn.flow * burning
        speed = gained + burn.exhaust * np.log(burn.start_mass / mass)
        # only the first stage carries the air: by separation the vehicle is
        # above 60 km, where the drag is below the error of the model itself
        area = vehicle.frontal_areas[0] if burn.index == 0 else 0.0
        loss = lost + _cumulative_trapezium(
            _loss_rate(burn.begin + elapsed, speed, mass, required, area), walked)

        balance = speed - required - loss
        crossed = np.flatnonzero(balance >= 0.0)
        if len(crossed):
            first = int(crossed[0])
            if first == 0:
                return burn.begin
            below, above = balance[first - 1], balance[first]
            return float(burn.begin + elapsed[first - 1]
                         + walked * below / (below - above))
        gained, lost = float(speed[-1]), float(loss[-1])

    return None


def analytic_altitude(vehicle: LaunchVehicle, programme: PitchProgramme) -> float:
    """The altitude a pitch programme reaches by the end of it, m.

    The vertical share of the velocity is the sine of the flight-path angle,
    which the programme has already tabulated; the speed is the Tsiolkovsky
    equation stage by stage, less the gravity lost as the area under that same
    share. Their product integrated over the programme is the altitude.

    What is left out is what makes it an estimate: the air, the thrust deficit
    at sea level, and the fall of gravity with altitude. All three push the
    same way, so the figure is always high - which is why the band it is
    screened against is not centred on the target.
    """
    time = programme.time
    share = np.sin(programme.angle)
    step = time[1] - time[0]
    speed = _tsiolkovsky_speed(vehicle, time) \
        - STANDARD_GRAVITY * _cumulative_trapezium(share, step)
    return float(np.trapezoid(share * np.maximum(0.0, speed), time))


def _tsiolkovsky_speed(vehicle: LaunchVehicle, time: np.ndarray) -> np.ndarray:
    """Speed from the propellant alone, stage by stage, m/s."""
    speed = np.zeros_like(time)
    gained = 0.0
    for burn in burns(vehicle):
        # clipped at the burn time, so a coast after the tank is empty carries
        # the speed forward flat rather than going on accelerating
        burning = np.clip(time - burn.begin, 0.0, burn.burn_time)
        rise = burn.exhaust * np.log(
            burn.start_mass / (burn.start_mass - burn.flow * burning))
        alight = time > burn.begin
        speed[alight] = gained + rise[alight]
        gained += burn.exhaust * math.log(
            burn.start_mass / (burn.start_mass - burn.flow * burn.burn_time))
    return speed


def _loss_rate(time: np.ndarray, speed: np.ndarray, mass: np.ndarray,
               required: float, area: float) -> np.ndarray:
    """Velocity lost per second at each instant of a stage, m/s^2.

    The turn is parametrised by the share of the energy already bought rather
    than by time, which is what makes this computable before the length of the
    ascent is known: the angle falls from the vertical to the horizon as the
    square of what is left to buy.
    """
    share = np.minimum(1.0, speed / required)
    angle = 0.5 * np.pi * (1.0 - share) ** 2
    # climbed at the mean speed so far along the current direction. The coarsest
    # thing here, and it only ever enters through R + h and the air density
    altitude = 0.5 * speed * time * np.sin(angle)
    # gravity less what the horizontal speed already carries: at the circular
    # speed in the horizon the two cancel and nothing more is lost to gravity
    effective = np.maximum(0.0, STANDARD_GRAVITY - speed**2 * np.cos(angle)**2
                           / (EARTH_RADIUS + altitude))
    drag = MEAN_DRAG_COEFFICIENT * area * SEA_LEVEL_DENSITY \
        * np.exp(-altitude / DENSITY_SCALE_HEIGHT) * speed**2 / (2.0 * mass)
    return effective * np.sin(angle) + drag


def _grid(span: float, step: float) -> np.ndarray:
    """Uniform grid over [0, span], the last point landing on the end."""
    count = max(1, int(math.ceil(span / step)))
    return np.linspace(0.0, span, count + 1)


def _cumulative_trapezium(values: np.ndarray, step: float) -> np.ndarray:
    """Running integral of a series sampled at a uniform step."""
    return np.concatenate(
        ([0.0], np.cumsum(0.5 * (values[1:] + values[:-1])) * step))
