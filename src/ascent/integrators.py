"""The integration step used to advance the equations of motion.

Knows nothing about rockets: it advances a state `y` from `t` to `t + dt`
given a derivative function `f(t, y)`. Fourth-order Runge-Kutta evaluates `f`
four times and its error falls with the fourth power of the step, so halving
the step cuts the error by sixteen - far more than the same work spent on
smaller first-order steps.

That rate assumes `f` is smooth across the step. Cut-off and stage separation
are step changes, so the caller must not step across them; Mission splits the
step at those instants.
"""


def rk4_step(f, t, y, dt):
    """Classic four-stage Runge-Kutta.

    Probes the slope at the start of the step, twice at the middle and once at
    the end, then advances along the weighted average that cancels the error
    terms up to dt^4.
    """
    half = dt / 2.0
    k1 = f(t, y)
    k2 = f(t + half, tuple(yi + ki * half for yi, ki in zip(y, k1)))
    k3 = f(t + half, tuple(yi + ki * half for yi, ki in zip(y, k2)))
    k4 = f(t + dt, tuple(yi + ki * dt for yi, ki in zip(y, k3)))
    return tuple(yi + (a + 2.0 * b + 2.0 * c + d) * dt / 6.0
                 for yi, a, b, c, d in zip(y, k1, k2, k3, k4))
