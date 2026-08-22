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
from dataclasses import dataclass

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
    # a watched threshold is looked for at this many points, then halved down
    CUT_OFF_SAMPLES = 8
    CUT_OFF_PASSES = 40

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
        self._steering_rate = 0.0
        self._throttle = 1.0
        self._attitude = None
        self._guided = True
        # on the pad: at rest relative to the Earth, pointing straight up
        self._y = (0.0, EARTH_RADIUS, 0.0)

        state = FlightState(mass=self.vehicle.lift_off_mass)
        # a magnitude, as at every other instant: omega is negative to the west
        state.inertial_speed = abs(self.omega) * EARTH_RADIUS
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
        """Advance one smooth piece, cutting it again at an event inside it.

        A tank running dry and a watched cut-off threshold cannot be put on the
        bounds beforehand: where they fall is what the integration is for. Each
        is solved for, the piece cut there, and the rest advanced as a piece of
        its own, with stage, throttle and tank all read again.
        """
        index, stage = self.vehicle.active_stage(begin)
        # read at the start of the piece, instant and state alike: no piece
        # straddles a scheduled switching instant, and a watched threshold asks
        # about the state the piece begins in
        capacity = stage.propellant_mass
        segment = Segment(stage, index, self._probe_throttle(begin),
                          self._attitude, self._burned[index] < capacity)
        self._throttle = segment.throttle

        derivatives = self._guided_rates if self._guided else self._free_rates
        def rates(t, y):
            return derivatives(t, y, segment)

        y = (*self._y, self._burned[index])
        advanced = rk4_step(rates, begin, y, end - begin)
        event, emptied = self._event_within(rates, y, begin, end, advanced,
                                            segment, capacity)

        if event is None:
            self._y = advanced[:-1]
            self._burned[index] = min(advanced[-1], capacity)
            return

        at_event = rk4_step(rates, begin, y, event - begin)
        self._y = at_event[:-1]
        # dry to the last bit, or the rest of the step relights on the remainder
        self._burned[index] = capacity if emptied else min(at_event[-1], capacity)
        self._integrate(event, end)

    def _event_within(self, rates, y, begin: float, end: float, advanced,
                      segment: Segment, capacity: float) -> tuple[float | None, bool]:
        """The first instant strictly inside the piece at which it stops holding."""
        dry = cut = None
        # the burn is allowed to run past the tank: that overshoot is the only
        # thing that says the tank empties inside the piece, and where
        if y[-1] < capacity < advanced[-1]:
            dry = self._solve_exhaustion(rates, y, begin, end, advanced[-1], capacity)
        if segment.throttle > 0 and self.cutoff.watches:
            cut = self._solve_cut_off(rates, y, begin, end)

        inside = [(t, t is dry) for t in (dry, cut) if t is not None and begin < t < end]
        return min(inside) if inside else (None, False)

    def _watched(self, t: float, y) -> tuple[float, float]:
        """Altitude and inertial speed at a trial point, for a cut-off to read.

        Off a state handed in rather than off the flight, so that the instant a
        threshold is met can be solved for inside a step.
        """
        if self._guided:
            speed, radius = y[0], y[1]
            angle = self.pitch_programme.sample(t)[0]
            horizontal, vertical = speed * math.cos(angle), speed * math.sin(angle)
        else:
            radius, _, vertical, horizontal = y[:4]
        return (radius - EARTH_RADIUS,
                math.hypot(horizontal + self.omega * radius, vertical))

    def _probe_throttle(self, t: float) -> float:
        return self.cutoff.throttle(t, *self._watched(t, self._y))

    def _solve_cut_off(self, rates, y, begin: float, end: float) -> float | None:
        """The first instant inside the piece at which a watched threshold is met.

        Walked rather than tested at the end alone: the inertial speed can rise
        through a threshold and fall back under it inside one piece, near the
        top of a lofted ascent, and the end would then show nothing. Bisection
        rather than the regula falsi a dry tank gets, because a policy says
        whether it has fired and not by how much. Only a watched policy pays.
        """
        low = begin
        for i in range(1, self.CUT_OFF_SAMPLES + 1):
            high = begin + (end - begin) * i / self.CUT_OFF_SAMPLES
            trial = rk4_step(rates, begin, y, high - begin)
            if self.cutoff.fired(*self._watched(high, trial)):
                break
            low = high
        else:
            return None

        for _ in range(self.CUT_OFF_PASSES):
            middle = 0.5 * (low + high)
            trial = rk4_step(rates, begin, y, middle - begin)
            if self.cutoff.fired(*self._watched(middle, trial)):
                high = middle
            else:
                low = middle
        return high

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
        drag = self.vehicle.drag(air, radius - EARTH_RADIUS, speed, segment.index)

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
        drag = self.vehicle.drag(air, radius - EARTH_RADIUS, speed, segment.index)

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

        Whether the tank has anything in it is settled once for the piece and
        never at a trial point: a trial point that overshoots the capacity
        would drop the thrust in the middle of the step, which is the step
        change that cutting the step at the dry instant exists to keep out.
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
            # a magnitude has no sign to turn round: reported as zero this
            # would read as a vehicle at rest while its radius went on falling
            if speed < 0.0:
                raise ValueError(
                    f'the vehicle has run out of speed against its programme at '
                    f't = {t:.1f} s ({speed:.1f} m/s). This pitch programme '
                    f'cannot be flown by this vehicle.')
            state.speed = speed
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
        state.drag = self.vehicle.drag(air, state.altitude, state.speed, index)
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

        rate = 0.0
        if state.thrust > 1.0:
            demanded = (state.mass / state.thrust) \
                * (speed * state.flight_path_rate + normal)
            state.steering_demand = demanded
            state.steering_angle = math.asin(max(-1.0, min(1.0, demanded)))
            rate = (state.thrust / state.mass) * (1.0 - math.cos(state.steering_angle))
        # over the interval, not off its end alone. An interval that begins
        # unpowered and ends alight spans an ignition and was flown before it,
        # so it carries nothing: averaging across that step change would charge
        # the new stage for time the old one flew
        span = 0.0 if self._steering_rate == 0.0 else 0.5 * (self._steering_rate + rate)
        self._steering_loss += span * self.dt
        self._steering_rate = rate
