"""Pitch programmes: the angles they promise at the instants that define them."""

import math

import numpy as np

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
