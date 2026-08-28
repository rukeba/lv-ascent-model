"""When the engines stop.

A cut-off policy answers one question per instant - what the throttle is - and,
when it can, says in advance at what time it will switch. Cut-off is a step
change in acceleration, so a policy that knows its own switching time lets the
integration step be cut exactly there instead of smearing the event over a
whole step.

A policy that cannot say its instant in advance has to watch the flight
instead, and then cut-off is a thing that happens once rather than a condition
that holds. `LatchedCutoff` is where that is settled.

See docs/configuration.md
"""


class Cutoff:
    # whether the flight has to be watched for this policy to fire. False when
    # the policy knows its own instant, which is then a bound of a piece and
    # never falls inside one
    watches = False

    def throttle(self, t: float, altitude: float, inertial_speed: float) -> float:
        return 1.0

    def scheduled_time(self) -> float | None:
        """The switching instant, when it is known before the flight."""
        return None

    def reset(self) -> None:
        """Forget anything held from an earlier flight. Called before each run."""

    def fired(self, altitude: float, inertial_speed: float) -> bool:
        """Whether a policy that watches the flight has fired at this state.

        Asked at trial points that are never reached, so it answers without
        being told. A policy that knows its own instant watches nothing: that
        instant is already a bound of the piece, so it answers no.
        """
        return False

    def describe(self) -> str:
        return 'no cut-off'


class CutoffAtTime(Cutoff):
    def __init__(self, time: float) -> None:
        self.time = time

    def throttle(self, t: float, altitude: float, inertial_speed: float) -> float:
        return 1.0 if t < self.time else 0.0

    def scheduled_time(self) -> float | None:
        return self.time

    def describe(self) -> str:
        return f'at t = {self.time:g} s'


class LatchedCutoff(Cutoff):
    """A threshold watched in flight: once it has fired the engines stay off.

    Both quantities below are crossed on the way up and fall again afterwards,
    and reading the condition afresh each time would relight the engines there.
    So the first crossing is remembered, which is why whether the threshold is
    met is kept apart from the act of firing on it: `Mission` asks `reached` at
    trial points while solving for the instant.

    Holding that state is safe because the throttle is read once per piece of a
    step, in flight order and never at a trial point of the scheme.
    `Mission.run` clears it before each flight.
    """

    def __init__(self) -> None:
        self._cut = False

    @property
    def watches(self) -> bool:
        """Still worth watching: one that has fired has nothing left to see."""
        return not self._cut

    def reset(self) -> None:
        self._cut = False

    def throttle(self, t: float, altitude: float, inertial_speed: float) -> float:
        self._cut = self.fired(altitude, inertial_speed)
        return 0.0 if self._cut else 1.0

    def fired(self, altitude: float, inertial_speed: float) -> bool:
        return self._cut or self.reached(altitude, inertial_speed)

    def reached(self, altitude: float, inertial_speed: float) -> bool:
        """Whether the threshold is met at this instant, taken on its own."""
        raise NotImplementedError


class CutoffAtAltitude(LatchedCutoff):
    def __init__(self, altitude: float) -> None:
        super().__init__()
        self.altitude = altitude

    def reached(self, altitude: float, inertial_speed: float) -> bool:
        return altitude >= self.altitude

    def describe(self) -> str:
        return f'at h = {self.altitude / 1000:g} km'


class CutoffAtInertialSpeed(LatchedCutoff):
    """Cut-off on the inertial speed - the quantity that fixes the orbit."""

    def __init__(self, speed: float) -> None:
        super().__init__()
        self.speed = speed

    def reached(self, altitude: float, inertial_speed: float) -> bool:
        return inertial_speed >= self.speed

    def describe(self) -> str:
        return f'at v = {self.speed:g} m/s inertial'
