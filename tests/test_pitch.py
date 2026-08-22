"""Pitch programmes: the angles they promise at the instants that define them."""

import math

import numpy as np
import pytest

from ascent.pitch import (BilinearTangentProgramme, FivePhaseProgramme,
                          VelocityShareProgramme, bilinear_coefficients)


def test_five_phase_turns_from_vertical_to_horizontal():
    programme = FivePhaseProgramme(t1=20.0, t4=500.0, k2=0.06, k3=0.5)

    assert math.isclose(programme.sample(0.0)[0], math.pi / 2)
    assert math.isclose(programme.sample(19.9)[0], math.pi / 2)
    assert abs(programme.sample(500.0)[0]) < 1e-9
    assert programme.end_time == 500.0


def test_five_phase_rate_starts_and_ends_at_zero():
    programme = FivePhaseProgramme(t1=20.0, t4=500.0, k2=0.06, k3=0.5)

    assert programme.sample(20.0)[1] == 0.0
    assert abs(programme.sample(499.99)[1]) < 1e-4
    # the turn only ever pitches down
    assert np.all(programme.rate <= 1e-12)


def test_velocity_share_stays_within_its_bounds():
    programme = VelocityShareProgramme(t1=20.0, tf=480.0, te=500.0, s=0.9)

    assert np.all(programme.share >= 0.0) and np.all(programme.share <= 1.0)
    assert math.isclose(programme.sample(10.0)[0], math.pi / 2)
    # past the end of the turn the velocity is entirely horizontal
    assert abs(programme.sample(490.0)[0]) < 1e-12


def test_a_velocity_share_outside_its_range_is_not_a_turn():
    """The quartic only stays a turn while it stays inside [0, 1].

    Its interior stationary point sits at (s - 3) / 2s, which falls inside the
    turn once |s| passes 3: the share then leaves [0, 1] partway along and has
    to be clipped back, which puts a kink in the middle of the turn. The rate
    is read off the tabulation by finite differences, so a kink there becomes a
    pitch rate - and a steering loss - that answers to the grid step rather
    than to the programme.
    """
    for s in (-3.0, 0.0, 3.0):
        edge = VelocityShareProgramme(t1=20.0, tf=480.0, te=500.0, s=s)
        assert edge.share.min() >= 0.0
        assert edge.share.max() <= 1.0

    for s in (-3.001, 3.001, 10.0):
        with pytest.raises(ValueError, match='velocity share'):
            VelocityShareProgramme(t1=20.0, tf=480.0, te=500.0, s=s)


def test_a_programme_that_cannot_be_built_says_so():
    """The parameters that divide something have to be checked before they do.

    Every fraction of the five-phase turn ends up under a division bar, and the
    velocity share needs a turn with something in it. Left unchecked they raise
    out of the arithmetic, or quietly build a turn that runs backwards or one
    that has no interior point at all and steps from vertical to horizontal.
    """
    for bad in ({'t4': 20.0}, {'k2': 0.0}, {'k3': -0.1}, {'k2': 0.6, 'k3': 0.5}):
        with pytest.raises(ValueError, match='five phases'):
            FivePhaseProgramme(**{'t1': 20.0, 't4': 500.0, 'k2': 0.06, 'k3': 0.5, **bad})

    for bad in ({'tf': 20.0}, {'tf': 10.0}, {'te': 15.0}):
        with pytest.raises(ValueError, match='vertical rise'):
            VelocityShareProgramme(**{'t1': 20.0, 'tf': 480.0, 'te': 500.0,
                                      's': 0.9, **bad})

    # the bilinear tangent turns from t1 to te, so those two have an order too
    for bad in ({'te': 20.0}, {'te': 10.0}):
        with pytest.raises(ValueError, match='end after it starts'):
            BilinearTangentProgramme(**{'t1': 20.0, 'a': -0.06, 'b': 28.6,
                                        'c': 0.15, 'te': 490.0, **bad})

    # nor can its denominator pass through zero on the way, which would come
    # back as a jump of pi in the angle and a division by nothing in the rate
    with pytest.raises(ValueError, match='passes through'):
        BilinearTangentProgramme(t1=0.0, a=-28.6, b=28.6, c=-1.0, te=2.0)

    # and none of them can be shorter than the grid they are tabulated on
    with pytest.raises(ValueError, match='one grid step'):
        BilinearTangentProgramme(t1=0.0, a=-0.06, b=28.6, c=0.15, te=0.05)


def test_bilinear_tangent_passes_through_its_three_points():
    a, b, c = bilinear_coefficients(t1=20.0, angle_1_deg=89.0, t_mid=150.0,
                                    angle_mid_deg=50.0, te=500.0, angle_e_deg=0.0)
    programme = BilinearTangentProgramme(t1=20.0, a=a, b=b, c=c, te=500.0)

    assert math.isclose(math.degrees(programme.sample(20.0)[0]), 89.0, abs_tol=1e-6)
    assert math.isclose(math.degrees(programme.sample(150.0)[0]), 50.0, abs_tol=1e-3)
    assert abs(math.degrees(programme.sample(500.0)[0])) < 1e-6


def test_sampling_interpolates_between_tabulated_points():
    programme = FivePhaseProgramme(t1=20.0, t4=500.0, k2=0.06, k3=0.5)
    before, after = programme.sample(100.0)[0], programme.sample(100.1)[0]

    middle = programme.sample(100.05)[0]
    assert min(before, after) < middle < max(before, after)
