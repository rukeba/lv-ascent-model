"""The step written out for the two lengths, against the same step in general.

`rk4_step` carries an explicit form for a state of four components and one of
five, which is what the equations of motion of this model use, and `_general`
for anything else. They have to be the same scheme to the last bit, or a
trajectory would depend on how many components it happens to be written in.
"""

import math

from ascent.integrators import _general, rk4_step


def nonlinear(t, y, held):
    """Something with a term of every kind in it, and no zeros to hide behind.

    `held` scales the whole derivative, so a form that dropped it or handed it
    on changed would not give the same answer as the one that did not.
    """
    return tuple(held * (math.sin(t + i) * yi + 0.3 * t - yi * yi / (1.0 + i))
                 for i, yi in enumerate(y))


def test_four_components_are_the_general_step():
    y = (1.5, -0.25, 300.0, 7.125)
    for dt in (1.0, 0.1, 0.025, 1e-3):
        assert (rk4_step(nonlinear, 0.7, y, dt, 1.5)
                == _general(nonlinear, 0.7, y, dt, 1.5))


def test_five_components_are_the_general_step():
    y = (6.371e6, 0.02, 120.0, 4400.0, 1.5e5)
    for dt in (1.0, 0.1, 0.025, 1e-3):
        assert (rk4_step(nonlinear, 12.3, y, dt, 0.75)
                == _general(nonlinear, 12.3, y, dt, 0.75))


def test_a_length_it_has_no_explicit_form_for_still_steps():
    for length in (1, 2, 3, 6, 9):
        y = tuple(0.5 + i for i in range(length))
        assert (rk4_step(nonlinear, 0.0, y, 0.1, 1.0)
                == _general(nonlinear, 0.0, y, 0.1, 1.0))


def test_what_is_held_reaches_every_probe_unchanged():
    """All four evaluations are given it, and given the same thing."""
    seen = []

    def watch(t, y, held):
        seen.append(held)
        return (0.0,) * len(y)

    held = object()
    for y in ((1.0, 2.0, 3.0, 4.0), (1.0, 2.0, 3.0, 4.0, 5.0), (1.0, 2.0)):
        seen.clear()
        rk4_step(watch, 0.0, y, 0.1, held)
        assert seen == [held] * 4


def test_the_step_is_fourth_order():
    """Halving the step cuts the error by sixteen, which is what buys it."""
    def decay(t, y, held):
        return (-2.0 * y[0],)

    def error(dt):
        y = (1.0,)
        for step in range(int(round(1.0 / dt))):
            y = rk4_step(decay, step * dt, y, dt, None)
        return abs(y[0] - math.exp(-2.0))

    coarse, fine = error(0.1), error(0.05)
    assert 12.0 < coarse / fine < 20.0
