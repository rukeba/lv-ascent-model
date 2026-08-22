"""When the engines stop.

A cut-off policy answers one question per instant - what the throttle is - and,
when it can, says in advance at what time it will switch. Cut-off is a step
change in acceleration, so a policy that knows its own switching time lets the
integration step be cut exactly there instead of smearing the event over a
whole step.
"""


class Cutoff:
    def throttle(self, t: float, altitude: float, inertial_speed: float) -> float:
        return 1.0

    def scheduled_time(self) -> float | None:
        """The switching instant, when it is known before the flight."""
        return None

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


class CutoffAtAltitude(Cutoff):
    def __init__(self, altitude: float) -> None:
        self.altitude = altitude

    def throttle(self, t: float, altitude: float, inertial_speed: float) -> float:
        return 1.0 if altitude < self.altitude else 0.0

    def describe(self) -> str:
        return f'at h = {self.altitude / 1000:g} km'


class CutoffAtInertialSpeed(Cutoff):
    """Cut-off on the inertial speed - the quantity that fixes the orbit."""

    def __init__(self, speed: float) -> None:
        self.speed = speed

    def throttle(self, t: float, altitude: float, inertial_speed: float) -> float:
        return 1.0 if inertial_speed < self.speed else 0.0

    def describe(self) -> str:
        return f'at v = {self.speed:g} m/s inertial'
