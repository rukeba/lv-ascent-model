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
        self._inverse_step = 1.0 / (time[1] - time[0])

    @staticmethod
    def _grid(end_time: float) -> np.ndarray:
        grid = np.arange(0.0, end_time + GRID_STEP, GRID_STEP)
        return grid[grid <= end_time + 1e-9]

    @property
    def end_time(self) -> float:
        return float(self.time[-1])

    def sample(self, t: float) -> tuple[float, float, float]:
        """Angle, rate and acceleration at an arbitrary instant.

        Interpolated rather than snapped to the nearest tabulated point: a
        multi-stage integrator probes the middle of a step, and a staircase
        there caps the order of accuracy whatever the scheme.
        """
        if t >= self.end_time:
            return float(self.angle[-1]), 0.0, 0.0

        position = (t - self.time[0]) * self._inverse_step
        i = min(max(int(position), 0), len(self.time) - 2)
        weight = position - i
        return (
            float(self.angle[i] + (self.angle[i + 1] - self.angle[i]) * weight),
            float(self.rate[i] + (self.rate[i + 1] - self.rate[i]) * weight),
            float(self.acceleration[i] + (self.acceleration[i + 1] - self.acceleration[i]) * weight),
        )

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

    def __init__(self, t1: float, tf: float, te: float, s: float) -> None:
        self.t1, self.te, self.s = t1, te, s
        # the turn cannot outlast the burn, nor precede the vertical rise
        self.tf = min(tf, te)
        if self.tf <= t1:
            self.tf = t1 + GRID_STEP

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
