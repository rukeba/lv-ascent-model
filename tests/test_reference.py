"""The published velocity budget of the three pitch programmes.

Falcon 9 into a 500 km circular orbit from Cape Canaveral, due east, at 10
steps per second. These are the figures the model was published with; they pin
the whole chain - atmosphere, propulsion, equations of motion, event handling
and the loss accounting - to one decimal place.
"""

import pytest

from ascent import (BilinearTangentProgramme, CutoffAtTime, FivePhaseProgramme,
                    Mission, VelocityShareProgramme, load_vehicle, velocity_budget)

# programme, cut-off time, (gravity, aerodynamic, steering) loss, m/s
CASES = (
    ('five-phase',
     FivePhaseProgramme(t1=20.0, t4=502.8, k2=0.056178, k3=0.522859),
     502.8, (2326.7, 29.3, 578.7)),
    ('velocity-share',
     VelocityShareProgramme(t1=20.0, tf=491.691775, te=502.1492, s=0.995106),
     502.1492, (2290.9, 29.7, 460.0)),
    ('bilinear-tangent',
     BilinearTangentProgramme(t1=20.0, a=-1.097246, b=527.99193, c=1.927467, te=501.2),
     501.2, (2242.1, 29.6, 485.2)),
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
    budget = velocity_budget(mission.run())
    gravity, aerodynamic, steering = published

    assert budget.gravity == pytest.approx(gravity, abs=0.1)
    assert budget.aerodynamic == pytest.approx(aerodynamic, abs=0.1)
    assert budget.steering == pytest.approx(steering, abs=0.1)
    # and each of them puts the vehicle on the orbit it was aiming for
    assert mission.orbit.perigee_altitude == pytest.approx(500_000, abs=1_000)
