"""Pitch programmes: the prescribed flight-path angle gamma(t) of the ascent.

Each programme is a different way of parametrising the turn from the vertical
to the horizontal, and it is these parameters that are being optimised. All of
them are tabulated on a uniform time grid once, at construction, and read back
by interpolation, so the shape of a programme never enters the equations of
motion - only its value at an instant does.

The programme ends before the engines do. After its last instant the vehicle
holds the attitude it reached and flies on it until cut-off.
"""

import numpy as np

GRID_STEP = 0.1


class PitchProgramme:
    """Tabulated gamma(t) with its first two time derivatives."""

    def _tabulate(self, time, angle, rate, acceleration) -> None:
        self.time = time
        self.angle = angle
        self.rate = rate
        self.acceleration = acceleration
        self._inverse_step = float(1.0 / (time[1] - time[0]))
        # The same tables as plain floats, and the figures the two readers
        # below take off their ends. Those readers are the most-run lines in
        # the model - four calls an integration step - and taking a number out
        # of a numpy array wraps it in a numpy scalar first; the arithmetic
        # that follows is the same IEEE double either way.
        self._start = float(time[0])
        self._end_time = float(time[-1])
        self._last_angle = float(angle[-1])
        self._angles, self._rates, self._accelerations = (
            np.asarray(table).tolist() for table in (angle, rate, acceleration))
        self._top = len(self._angles) - 2

    @staticmethod
    def _grid(end_time: float) -> np.ndarray:
        # two points at least: the tabulation reads its step off the first pair
        if not end_time >= GRID_STEP:
            raise ValueError(f'a programme has to last at least one grid step '
                             f'of {GRID_STEP:g} s, and not {end_time:g} s')
        grid = np.arange(0.0, end_time + GRID_STEP, GRID_STEP)
        return grid[grid <= end_time + 1e-9]

    @property
    def end_time(self) -> float:
        return self._end_time

    def sample(self, t: float) -> tuple[float, float, float]:
        """Angle, rate and acceleration at an arbitrary instant.

        Interpolated rather than snapped to the nearest tabulated point: a
        multi-stage integrator probes the middle of a step, and a staircase
        there caps the order of accuracy whatever the scheme.
        """
        if t >= self._end_time:
            return self._last_angle, 0.0, 0.0

        position = (t - self._start) * self._inverse_step
        i = min(max(int(position), 0), self._top)
        weight = position - i
        angle, rate, acceleration = self._angles, self._rates, self._accelerations
        return (
            angle[i] + (angle[i + 1] - angle[i]) * weight,
            rate[i] + (rate[i + 1] - rate[i]) * weight,
            acceleration[i] + (acceleration[i + 1] - acceleration[i]) * weight,
        )

    def angle_at(self, t: float) -> float:
        """The angle alone, which is all the equations of motion ask for.

        `sample` interpolates three tables; the rate and the acceleration
        beside the angle are for the reporting. This is the one of the two the
        scheme calls, four times an integration step.
        """
        if t >= self._end_time:
            return self._last_angle
        position = (t - self._start) * self._inverse_step
        i = min(max(int(position), 0), self._top)
        angle = self._angles
        return angle[i] + (angle[i + 1] - angle[i]) * (position - i)

    def describe(self) -> str:
        raise NotImplementedError


class FivePhaseProgramme(PitchProgramme):
    """Turn built from constant angular accelerations.

    Vertical rise to t1; the pitch rate is built up to omega over the fraction
    k2 of the turn; held constant over the fraction k3; then arrested so that
    the angle arrives at final_angle exactly at t4, where the programme ends
    and the fifth phase - free flight on the attitude reached - begins. omega
    follows from the angle that has to be covered, so the shape of the turn is
    set by t1, t4, k2 and k3 alone.
    """

    def __init__(self, t1: float, t4: float, k2: float, k3: float,
                 final_angle_deg: float = 0.0) -> None:
        # every one of these divides something below, and the phases have to
        # come in order: a bad set would otherwise raise out of the arithmetic
        # or build a turn that runs backwards
        if not (t4 > t1 and k2 > 0.0 and k3 >= 0.0 and k2 + k3 < 1.0):
            raise ValueError(
                f'the five phases need t4 > t1, k2 > 0, k3 >= 0 and k2 + k3 < 1, '
                f'not t1={t1:g}, t4={t4:g}, k2={k2:g}, k3={k3:g}')
        self.t1, self.t4, self.k2, self.k3 = t1, t4, k2, k3
        self.final_angle = np.deg2rad(final_angle_deg)

        turn = t4 - t1
        self.t2 = t1 + turn * k2
        self.t3 = self.t2 + turn * k3

        omega = -(np.pi - 2 * self.final_angle) / turn / (1 + k3)
        arrest = -omega / (t4 - self.t3)
        angle_1 = np.pi / 2
        angle_2 = angle_1 + (omega / (2 * k2 * turn)) * (self.t2 - t1) ** 2
        angle_3 = angle_2 + omega * (self.t3 - self.t2)

        t = self._grid(t4)
        angle = np.full_like(t, angle_1)
        rate = np.zeros_like(t)
        acceleration = np.zeros_like(t)

        build_up = (t > t1) & (t <= self.t2)
        d = t[build_up] - t1
        angle[build_up] = angle_1 + (omega / (2 * k2 * turn)) * d**2
        rate[build_up] = (omega / (k2 * turn)) * d
        acceleration[build_up] = omega / (k2 * turn)

        constant = (t > self.t2) & (t <= self.t3)
        angle[constant] = angle_2 + omega * (t[constant] - self.t2)
        rate[constant] = omega

        slow_down = t > self.t3
        d = t[slow_down] - self.t3
        angle[slow_down] = angle_3 + omega * d + (arrest / 2) * d**2
        rate[slow_down] = omega + arrest * d
        acceleration[slow_down] = arrest

        self._tabulate(t, angle, rate, acceleration)

    def describe(self) -> str:
        return (f'five-phase(t1={self.t1:g} s, t4={self.t4:g} s, '
                f'k2={self.k2:g}, k3={self.k3:g})')


class VelocityShareProgramme(PitchProgramme):
    """Turn set by the share of the speed that stays vertical.

    A quartic prescribes eta = v_vertical / v = sin(gamma) directly, and the
    angle follows as gamma = arcsin(eta). The quartic is flat at both ends, so
    the turn starts smoothly and joins the horizontal phase without a kink.
    The parameter s controls how much of the turn is done early.
    """

    # the quartic has an interior stationary point at (s - 3) / 2s, which falls
    # inside the turn for |s| > 3: beyond that the share leaves [0, 1] and a
    # clip would kink the turn, which the differenced rate reads off the grid
    SHARE_LIMIT = 3.0

    def __init__(self, t1: float, tf: float, te: float, s: float) -> None:
        if not -self.SHARE_LIMIT <= s <= self.SHARE_LIMIT:
            raise ValueError(
                f'the velocity share is a turn only for '
                f'|s| <= {self.SHARE_LIMIT:g}, and s = {s:g}')
        self.t1, self.te, self.s = t1, te, s
        # the turn cannot outlast the burn
        self.tf = min(tf, te)
        if self.tf <= t1:
            raise ValueError(
                f'the turn has to start after the vertical rise and end before '
                f'the burn, not t1={t1:g}, tf={tf:g}, te={te:g}')

        t = self._grid(te)
        share = np.ones_like(t)
        turning = (t > t1) & (t < self.tf)
        tau = (t[turning] - t1) / (self.tf - t1)
        share[turning] = s * tau**4 + (2 - 2 * s) * tau**3 + (s - 3) * tau**2 + 1.0
        share[t >= self.tf] = 0.0
        self.share = np.clip(share, 0.0, 1.0)

        angle = np.arcsin(self.share)
        rate = np.gradient(angle, t)
        self._tabulate(t, angle, rate, np.gradient(rate, t))

    def describe(self) -> str:
        return (f'velocity-share(t1={self.t1:g} s, tf={self.tf:g} s, '
                f'te={self.te:g} s, s={self.s:g})')


class BilinearTangentProgramme(PitchProgramme):
    """Turn following tan(gamma) = (a*tau + b) / (c*tau + 1), tau = t - t1.

    The classical optimal-steering law of powered flight, held here as an
    explicit programme rather than solved for: vertical rise to t1, then the
    ratio of two linear functions until te.
    """

    def __init__(self, t1: float, a: float, b: float, c: float, te: float) -> None:
        if te <= t1:
            raise ValueError(f'the turn has to end after it starts, and not '
                             f't1={t1:g}, te={te:g}')
        # the denominator has a pole at tau = -1/c, and a turn that runs
        # through it comes back as a jump of pi in the angle and as division by
        # nothing in the rate
        if c * (te - t1) + 1.0 <= 0.0:
            raise ValueError(f'the denominator of the tangent passes through '
                             f'zero inside the turn: c={c:g}, te-t1={te - t1:g}')
        self.t1, self.a, self.b, self.c, self.te = t1, a, b, c, te

        t = self._grid(te)
        angle = np.full_like(t, np.pi / 2)
        rate = np.zeros_like(t)
        acceleration = np.zeros_like(t)

        turning = t >= t1
        tau = t[turning] - t1
        lower = c * tau + 1
        upper = a * tau + b
        # derivative of arctan(upper/lower) by the quotient rule
        numerator = a - b * c
        denominator = lower**2 + upper**2

        angle[turning] = np.arctan(upper / lower)
        rate[turning] = numerator / denominator
        acceleration[turning] = -numerator * (2 * c * lower + 2 * a * upper) / denominator**2

        self._tabulate(t, angle, rate, acceleration)

    def describe(self) -> str:
        return (f'bilinear-tangent(t1={self.t1:g} s, a={self.a:g}, '
                f'b={self.b:g}, c={self.c:g}, te={self.te:g} s)')


def bilinear_coefficients(t1: float, angle_1_deg: float, t_mid: float,
                          angle_mid_deg: float, te: float,
                          angle_e_deg: float = 0.0) -> tuple[float, float, float]:
    """Coefficients of a bilinear tangent through three prescribed angles."""
    y_1, y_mid, y_e = (np.tan(np.deg2rad(x))
                       for x in (angle_1_deg, angle_mid_deg, angle_e_deg))
    b = y_1
    tau_mid, tau_e = t_mid - t1, te - t1
    # a*tau - c*(y*tau) = y - b at each of the two remaining points
    a, c = np.linalg.solve(
        np.array([[tau_mid, -y_mid * tau_mid], [tau_e, -y_e * tau_e]]),
        np.array([y_mid - b, y_e - b]))
    return float(a), float(b), float(c)
