"""When the engines stop.

A cut-off policy answers one question per instant - what the throttle is - and,
when it can, says in advance at what time it will switch. Cut-off is a step
change in acceleration, so a policy that knows its own switching time lets the
integration step be cut exactly there instead of smearing the event over a
whole step.

A policy that cannot say its instant in advance has to watch the flight
instead, and then cut-off is a thing that happens once rather than a condition
that holds. `LatchedCutoff` is where that is settled; see the note there for
why the difference matters and why holding the state is safe.
"""


class Cutoff:
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
        being told. A policy that knows its own instant watches nothing, and
        that instant is already a bound of the piece being integrated, so a
        crossing of its can never fall strictly inside one: it answers no.
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

    Both quantities below are crossed on the way up and fall again afterwards -
    the inertial speed as soon as the vehicle coasts uphill, the altitude past
    apogee. Reading the condition afresh each time would relight the engines
    there, on a stage that has propellant left, which is not what a cut-off
    means. So the first crossing is remembered instead.

    Where the threshold is crossed is not known before the step, so it cannot
    be put on the bounds of one the way a scheduled cut-off can. `Mission`
    solves for the instant instead and cuts the step there, asking `reached`
    at trial points - which is why the question of whether the threshold is met
    is kept apart from the act of firing on it.

    Holding that state is safe because the throttle is read once per piece of a
    step, before the piece is integrated and in flight order - never at a trial
    point of the scheme, which is where remembering anything would quietly cost
    the order of accuracy. `Mission.run` clears it before each flight.
    """

    def __init__(self) -> None:
        self._cut = False

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
