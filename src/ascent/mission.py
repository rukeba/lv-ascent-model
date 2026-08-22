"""Equations of motion of the powered ascent, and the stepping that solves them.

The flight is planar and written in polar coordinates about the centre of the
Earth: the distance `r` and the angle `psi` travelled from the pad. The state
is integrated in the frame that rotates with the Earth, so the speed the model
carries is the speed relative to the atmosphere and to the launch site, while
the orbit is built from the inertial speed, which adds the rotation the pad
already had.

While the pitch programme runs, the guidance holds the flight-path angle on it
and only the magnitude of the velocity is integrated. When the programme ends
the vehicle holds the attitude it reached and both velocity components are
integrated to the end of the flight.

Events matter more than the order of the scheme. Separation, cut-off, the end
of the programme and running a tank dry are step changes in mass, thrust or in
the equations themselves; the step is cut exactly at each of them, because at
around 60 m/s^2 an event misplaced by one step at 10 Hz is worth several m/s.
"""

import math
from dataclasses import dataclass, replace

from .atmosphere import Air, air_at, gravity
from .constants import EARTH_OMEGA, EARTH_RADIUS
from .cutoff import Cutoff
from .integrators import rk4_step
from .orbit import Orbit, orbit_from_state
from .pitch import PitchProgramme
from .state import FlightState
from .telemetry import Telemetry
from .vehicle import LaunchVehicle, Stage


def rotation_in_plane(latitude_deg: float, azimuth_deg: float) -> float:
    """Earth rotation projected on to the launch plane, rad/s.

    Full to the east, nothing due north, a penalty to the west.
    """
    return EARTH_OMEGA * math.cos(math.radians(latitude_deg)) \
        * math.sin(math.radians(azimuth_deg))


@dataclass(frozen=True)
class Segment:
    """What is held constant over one smooth piece of a step."""
    stage: Stage
    index: int
    throttle: float
    # attitude flown once the programme has ended, rad; None while it runs
    attitude: float | None
    # False once the tank is dry: the piece is flown on what the vehicle has
    burning: bool = True


class Mission:
    """One ascent: a vehicle, a pitch programme and a cut-off policy."""

    # a tank running dry is solved to this share of its capacity, in this many passes
    EXHAUSTION_TOLERANCE = 1e-12
    EXHAUSTION_PASSES = 8

    def __init__(self, vehicle: LaunchVehicle, pitch_programme: PitchProgramme,
                 cutoff: Cutoff, target_altitude: float, duration: float,
                 steps_per_second: float = 10, latitude_deg: float = 0.0,
                 azimuth_deg: float = 90.0) -> None:
        self.vehicle = vehicle
        self.pitch_programme = pitch_programme
        self.cutoff = cutoff
        self.target_altitude = target_altitude
        self.duration = duration
        self.steps_per_second = steps_per_second
        self.latitude_deg = latitude_deg
        self.azimuth_deg = azimuth_deg
        self.omega = rotation_in_plane(latitude_deg, azimuth_deg)

    def run(self) -> Telemetry:
        """Fly the mission and return the recorded flight."""
        self.dt = 1.0 / self.steps_per_second
        self.omega = rotation_in_plane(self.latitude_deg, self.azimuth_deg)
        self.cutoff.reset()
        self.telemetry = Telemetry()
        self._burned = [0.0] * len(self.vehicle.stages)
        self._steering_loss = 0.0
        self._throttle = 1.0
        self._attitude = None
        self._guided = True
        # on the pad: at rest relative to the Earth, pointing straight up
        self._y = (0.0, EARTH_RADIUS, 0.0)

        state = FlightState(mass=self.vehicle.lift_off_mass)
        state.inertial_speed = self.omega * EARTH_RADIUS
        self.telemetry.record(state)

        for step in range(int(round(self.duration * self.steps_per_second))):
            # computed, not accumulated, so step times land exactly on the
            # event times the step is cut at
            t = (step + 1) * self.dt
            for begin, end in self._segment_bounds(t - self.dt, t):
                if self._guided and begin >= self.pitch_programme.end_time:
                    self._release_guidance(begin)
                self._integrate(begin, end)
            state = self._collect(state, t)
            self.telemetry.record(state)

        self.final_state = state
        self.orbit: Orbit = orbit_from_state(
            state.radius,
            state.horizontal_speed + self.omega * state.radius,
            state.vertical_speed)
        return self.telemetry

    # --- stepping ---------------------------------------------------------

    def _segment_bounds(self, begin: float, end: float) -> list[tuple[float, float]]:
        """The step, cut at every discontinuity that falls inside it."""
        events = set(self.vehicle.staging_times_within(begin, end))
        scheduled = self.cutoff.scheduled_time()
        if scheduled is not None and begin < scheduled < end:
            events.add(scheduled)
        handover = self.pitch_programme.end_time
        if begin < handover < end:
            events.add(handover)
        bounds = [begin, *sorted(events), end]
        return [(a, b) for a, b in zip(bounds, bounds[1:]) if b > a]

    def _integrate(self, begin: float, end: float) -> None:
        """Advance one smooth piece and commit the propellant it burned."""
        index, stage = self.vehicle.active_stage(begin)
        # the throttle is read in the middle: the bounds sit exactly on the
        # switching instants, where the setting is ambiguous
        middle = 0.5 * (begin + end)
        capacity = stage.propellant_mass
        segment = Segment(stage, index, self._probe_throttle(middle),
                          self._attitude, self._burned[index] < capacity)
        self._throttle = segment.throttle

        derivatives = self._guided_rates if self._guided else self._free_rates
        def rates(t, y):
            return derivatives(t, y, segment)

        y = (*self._y, self._burned[index])
        advanced = rk4_step(rates, begin, y, end - begin)

        # the propellant the piece would take is allowed to run past the tank,
        # because that overshoot is the only thing that says the tank empties
        # inside the piece and where
        if y[-1] < capacity < advanced[-1]:
            dry = self._solve_exhaustion(rates, y, begin, end, advanced[-1], capacity)
            at_dry = rk4_step(rates, begin, y, dry - begin)
            spent = replace(segment, burning=False)
            def coasting(t, y):
                return derivatives(t, y, spent)
            advanced = rk4_step(coasting, dry, (*at_dry[:-1], capacity), end - dry)

        self._y = advanced[:-1]
        self._burned[index] = min(advanced[-1], capacity)

    def _probe_throttle(self, t: float) -> float:
        if self._guided:
            speed, radius, _ = self._y
            angle = self.pitch_programme.sample(t)[0]
            horizontal, vertical = speed * math.cos(angle), speed * math.sin(angle)
        else:
            radius, _, vertical, horizontal = self._y
        inertial = math.hypot(horizontal + self.omega * radius, vertical)
        return self.cutoff.throttle(t, radius - EARTH_RADIUS, inertial)

    def _solve_exhaustion(self, rates, y, begin: float, end: float,
                          burned_at_end: float, capacity: float) -> float:
        """The instant inside the piece at which the tank runs dry.

        Worth solving rather than estimating: a first stage empties a fraction
        of a second before separation under some 60 m/s^2, so a millisecond of
        error costs 0.06 m/s, and a single linear estimate is first order -
        which would drag the whole scheme down to first order exactly where it
        hurts most. Regula falsi on the propellant burned, re-integrating from
        the start of the piece; the flow rate barely varies over a step, so two
        or three passes reach the tolerance.
        """
        low, burned_low = begin, y[-1]
        high, burned_high = end, burned_at_end
        tolerance = capacity * self.EXHAUSTION_TOLERANCE
        dry = end

        for _ in range(self.EXHAUSTION_PASSES):
            span = burned_high - burned_low
            if span <= 0.0:
                break
            dry = low + (high - low) * (capacity - burned_low) / span
            burned = rk4_step(rates, begin, y, dry - begin)[-1]
            if abs(burned - capacity) <= tolerance:
                break
            if burned < capacity:
                low, burned_low = dry, burned
            else:
                high, burned_high = dry, burned

        return min(max(dry, begin), end)

    def _release_guidance(self, t: float) -> None:
        """Hand the vehicle over from the programme to free flight."""
        speed, radius, polar_angle = self._y
        self._attitude = self.pitch_programme.sample(t)[0]
        self._y = (radius, polar_angle,
                   speed * math.sin(self._attitude),
                   speed * math.cos(self._attitude))
        self._guided = False

    # --- equations of motion ----------------------------------------------

    def _guided_rates(self, t: float, y, segment: Segment):
        """dy/dt while the programme runs; y = (speed, radius, angle, burned).

        The programme fixes the direction of the velocity, so only its
        magnitude is integrated. Thrust and drag act along that direction;
        gravity and the centrifugal term of the rotating frame project on to it
        through the flight-path angle. The Coriolis term is perpendicular to
        the velocity and does no work on its magnitude.
        """
        speed, radius, _, burned = y
        angle = self.pitch_programme.sample(t)[0]
        air = air_at(radius - EARTH_RADIUS)
        mass = self.vehicle.mass_on(segment.index, burned)
        thrust, flow = self._propulsion(segment, air)
        drag = self.vehicle.drag(air, radius - EARTH_RADIUS, speed)

        acceleration = (thrust - drag) / mass \
            - (gravity(radius) - self.omega**2 * radius) * math.sin(angle)
        return (acceleration, speed * math.sin(angle),
                speed * math.cos(angle) / radius, flow)

    def _free_rates(self, t: float, y, segment: Segment):
        """dy/dt after the programme; y = (radius, angle, vertical, horizontal, burned).

        Polar equations in the rotating frame. The quadratic terms come from
        the rotating polar basis, the omega terms are the centrifugal and
        Coriolis accelerations of the Earth-fixed frame. Thrust and drag act
        along the held attitude, which is the zero-lift assumption of a vehicle
        flying at a small angle of attack.
        """
        radius, _, vertical, horizontal, burned = y
        speed = math.hypot(vertical, horizontal)
        air = air_at(radius - EARTH_RADIUS)
        mass = self.vehicle.mass_on(segment.index, burned)
        thrust, flow = self._propulsion(segment, air)
        drag = self.vehicle.drag(air, radius - EARTH_RADIUS, speed)

        axial = (thrust - drag) / mass
        omega = self.omega
        radial = axial * math.sin(segment.attitude) - gravity(radius) \
            + horizontal * horizontal / radius \
            + 2.0 * omega * horizontal + omega * omega * radius
        tangential = axial * math.cos(segment.attitude) \
            - vertical * horizontal / radius - 2.0 * omega * vertical
        return (vertical, horizontal / radius, radial, tangential, flow)

    def _propulsion(self, segment: Segment, air: Air) -> tuple[float, float]:
        """Thrust (N) and propellant flow (kg/s) over one piece of a step.

        Whether the tank still has anything in it is settled once, for the
        whole piece, and never at a trial point. A trial point that overshoots
        the capacity would drop the thrust in the middle of the step - the very
        step change that cutting the step at the instant the tank runs dry
        exists to keep out - and it would do so for the four-stage weights
        rather than for the part of the step that is really still burning.
        """
        if not segment.burning or segment.throttle <= 0:
            return 0.0, 0.0
        throttle = min(1.0, segment.throttle)
        return (segment.stage.thrust(air.pressure) * throttle,
                segment.stage.mass_flow(air.pressure, throttle))

    # --- reporting --------------------------------------------------------

    def _collect(self, previous: FlightState, t: float) -> FlightState:
        """Read the integrated state back out as a reported flight state."""
        state = FlightState(t=t)
        if self._guided:
            speed, radius, polar_angle = self._y
            angle, rate, _ = self.pitch_programme.sample(t)
            state.speed = max(0.0, speed)
            state.flight_path_angle, state.flight_path_rate = angle, rate
        else:
            radius, polar_angle, vertical, horizontal = self._y
            state.speed = math.hypot(vertical, horizontal)
            state.flight_path_angle = math.atan2(vertical, horizontal)
            state.flight_path_rate = \
                (state.flight_path_angle - previous.flight_path_angle) / self.dt
        # not finite covers the case this guard exists for: once anything goes
        # to NaN the comparison below is false and the run would carry on
        # producing numbers that are not numbers
        if not math.isfinite(radius) or radius <= 1.0:
            raise ValueError(
                f'the trajectory has left the model at t = {t:.1f} s '
                f'(radius {radius}). This pitch programme cannot be flown by '
                f'this vehicle.')
        state.radius = radius
        state.polar_angle = polar_angle
        state.inertial_speed = math.hypot(
            state.horizontal_speed + self.omega * radius, state.vertical_speed)

        index, stage = self.vehicle.active_stage(t)
        air = air_at(state.altitude)
        state.stage = index
        state.mass = self.vehicle.mass(t, self._burned[index])
        state.thrust = 0.0 if self._burned[index] >= stage.propellant_mass \
            else stage.thrust(air.pressure) * self._throttle
        state.drag = self.vehicle.drag(air, state.altitude, state.speed)
        state.dynamic_pressure = 0.0 if state.altitude > 100_000 \
            else 0.5 * air.density * state.speed**2

        if self._guided:
            self._accumulate_steering(state)
        state.steering_loss = self._steering_loss
        return state

    def _accumulate_steering(self, state: FlightState) -> None:
        """Recover the deflection the programme demands, and what it costs.

        The programme prescribes the flight-path angle without saying how it is
        produced, so the thrust deflection that would produce it is recovered
        from the normal equation of motion. The share of the thrust that this
        deflection points away from the velocity is the steering loss - the
        figure of merit by which pitch programmes are compared.
        """
        radius, speed = state.radius, state.speed
        # the centripetal balance is set by the inertial speed, and cancels
        # gravity exactly once in orbit
        effective_gravity = gravity(radius) - state.inertial_speed**2 / radius
        # Coriolis appears on the trajectory normal as 2*omega*v and unloads
        # the steering for a launch to the east
        normal = effective_gravity * math.cos(state.flight_path_angle) \
            - 2.0 * self.omega * speed

        if state.thrust > 1.0:
            demanded = (state.mass / state.thrust) \
                * (speed * state.flight_path_rate + normal)
            state.steering_angle = math.asin(max(-1.0, min(1.0, demanded)))
            self._steering_loss += (state.thrust / state.mass) \
                * (1.0 - math.cos(state.steering_angle)) * self.dt
