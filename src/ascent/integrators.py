"""The integration step used to advance the equations of motion.

Knows nothing about rockets: it advances a state `y` from `t` to `t + dt`
given a derivative function `f(t, y, held)`. Fourth-order Runge-Kutta
evaluates `f` four times and its error falls with the fourth power of the
step, so halving the step cuts the error by sixteen - far more than the same
work spent on smaller first-order steps.

That rate assumes `f` is smooth across the step. Cut-off and stage separation
are step changes, so the caller must not step across them; Mission splits the
step at those instants.

`held` is whatever the caller holds fixed over the step. It is handed to `f`
unchanged and never looked into here. It exists so that the derivative
function can stay a plain function of the instant and the state: what it needs
besides those two arrives as an argument rather than out of a closure rebuilt
at every step, or - worse - off the caller between one probe and the next,
which is how a scheme that evaluates the middle of a step quietly falls to
first order.

The two lengths this model carries - four while the pitch programme runs, five
after it - are written out component by component. This is the innermost line
of every trajectory, run some four million times in a search, and there a
generator over `zip` costs several times what the arithmetic inside it does.
`_general` below is the same scheme for any other length, and gives the same
numbers term for term.
"""


def rk4_step(f, t, y, dt, held):
    """Classic four-stage Runge-Kutta.

    Probes the slope at the start of the step, twice at the middle and once at
    the end, then advances along the weighted average that cancels the error
    terms up to dt^4.
    """
    half = dt / 2.0
    middle = t + half
    end = t + dt
    length = len(y)

    if length == 4:
        y0, y1, y2, y3 = y
        a0, a1, a2, a3 = f(t, y, held)
        b0, b1, b2, b3 = f(middle, (y0 + a0 * half, y1 + a1 * half,
                                    y2 + a2 * half, y3 + a3 * half), held)
        c0, c1, c2, c3 = f(middle, (y0 + b0 * half, y1 + b1 * half,
                                    y2 + b2 * half, y3 + b3 * half), held)
        d0, d1, d2, d3 = f(end, (y0 + c0 * dt, y1 + c1 * dt,
                                 y2 + c2 * dt, y3 + c3 * dt), held)
        return (y0 + (a0 + 2.0 * b0 + 2.0 * c0 + d0) * dt / 6.0,
                y1 + (a1 + 2.0 * b1 + 2.0 * c1 + d1) * dt / 6.0,
                y2 + (a2 + 2.0 * b2 + 2.0 * c2 + d2) * dt / 6.0,
                y3 + (a3 + 2.0 * b3 + 2.0 * c3 + d3) * dt / 6.0)

    if length == 5:
        y0, y1, y2, y3, y4 = y
        a0, a1, a2, a3, a4 = f(t, y, held)
        b0, b1, b2, b3, b4 = f(middle, (y0 + a0 * half, y1 + a1 * half,
                                        y2 + a2 * half, y3 + a3 * half,
                                        y4 + a4 * half), held)
        c0, c1, c2, c3, c4 = f(middle, (y0 + b0 * half, y1 + b1 * half,
                                        y2 + b2 * half, y3 + b3 * half,
                                        y4 + b4 * half), held)
        d0, d1, d2, d3, d4 = f(end, (y0 + c0 * dt, y1 + c1 * dt, y2 + c2 * dt,
                                     y3 + c3 * dt, y4 + c4 * dt), held)
        return (y0 + (a0 + 2.0 * b0 + 2.0 * c0 + d0) * dt / 6.0,
                y1 + (a1 + 2.0 * b1 + 2.0 * c1 + d1) * dt / 6.0,
                y2 + (a2 + 2.0 * b2 + 2.0 * c2 + d2) * dt / 6.0,
                y3 + (a3 + 2.0 * b3 + 2.0 * c3 + d3) * dt / 6.0,
                y4 + (a4 + 2.0 * b4 + 2.0 * c4 + d4) * dt / 6.0)

    return _general(f, t, y, dt, held)


def _general(f, t, y, dt, held):
    """The same step, for a state of any other length."""
    half = dt / 2.0
    middle = t + half
    k1 = f(t, y, held)
    k2 = f(middle, tuple(yi + ki * half for yi, ki in zip(y, k1)), held)
    k3 = f(middle, tuple(yi + ki * half for yi, ki in zip(y, k2)), held)
    k4 = f(t + dt, tuple(yi + ki * dt for yi, ki in zip(y, k3)), held)
    return tuple(yi + (a + 2.0 * b + 2.0 * c + d) * dt / 6.0
                 for yi, a, b, c, d in zip(y, k1, k2, k3, k4))
