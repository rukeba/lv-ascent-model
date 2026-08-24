"""The reference velocity budget of the three pitch programmes.

Falcon 9 into a 500 km circular orbit from Cape Canaveral, due east, at 10
steps per second. These pin the whole chain - atmosphere, propulsion, equations
of motion, event handling and the loss accounting - to one decimal place.

They are not the split printed in the dissertation, which has 2326.7 / 29.3 /
579.0 in the first row. The loss projection was corrected after that went to
paper: gravity up by some 250 m/s, steering down by some 50, and the 190 the
total gains is the gap by which the old split failed to close. The flight
itself did not move - the same trajectories, read the way the equations of
motion carry them.

These three sets are the ones the dissertation prints, and they are flown as
printed rather than solved again: they are on paper. So the orbits below are
the ones those parameters reach against the gravitational parameter as it now
stands, and each of the three answers to that in its own place. The five-phase
and the bilinear tangent both drop 0.7 km at the perigee, to 499.4, and hold
their apogees; the velocity share holds its perigee at 500.4 and comes down
0.7 km at the apogee instead, to 506.9.
"""

import pytest

from ascent import (BilinearTangentProgramme, CutoffAtTime, FivePhaseProgramme,
                    Mission, VelocityShareProgramme, load_vehicle, velocity_budget)

# programme, cut-off time, (gravity, aerodynamic, steering) loss, m/s
CASES = (
    ('five-phase',
     FivePhaseProgramme(t1=20.0, t4=502.8, k2=0.056178, k3=0.522859),
     502.8, (2568.8, 29.3, 526.4)),
    ('velocity-share',
     VelocityShareProgramme(t1=20.0, tf=491.691775, te=502.1492, s=0.995106),
     502.1492, (2538.0, 29.7, 411.0)),
    ('bilinear-tangent',
     BilinearTangentProgramme(t1=20.0, a=-1.097246, b=527.99193, c=1.927467, te=501.2),
     501.2, (2500.0, 29.6, 433.0)),
)


@pytest.mark.parametrize('name, programme, cutoff_time, published',
                         CASES, ids=[case[0] for case in CASES])
def test_published_velocity_budget(name, programme, cutoff_time, published):
    mission = Mission(
        vehicle=load_vehicle('config/lv.f9.yaml'),
        pitch_programme=programme,
        cutoff=CutoffAtTime(cutoff_time),
        target_altitude=500_000,
        duration=600.0,
        steps_per_second=10,
        latitude_deg=28.5,
        azimuth_deg=90.0,
    )
    budget = velocity_budget(mission.run(), mission.omega)
    gravity, aerodynamic, steering = published

    assert budget.gravity == pytest.approx(gravity, abs=0.1)
    assert budget.aerodynamic == pytest.approx(aerodynamic, abs=0.1)
    assert budget.steering == pytest.approx(steering, abs=0.1)
    # and each of them arrives at the altitude it was aiming for. Only two of
    # the three arrive circular: the velocity-share set leaves an apogee 6.9 km
    # up, which is the shape of that quartic rather than a miss
    assert mission.orbit.perigee_altitude == pytest.approx(500_000, abs=1_000)
    assert mission.orbit.apogee_altitude == pytest.approx(500_000, abs=10_000)
