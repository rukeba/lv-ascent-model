"""Every catalogue entry, flown again and checked against what it records."""

import pytest

from ascent import velocity_budget
from ascent.config import load_catalogue, mission_from_spec

CATALOGUE = load_catalogue('config/catalogue.yaml')


def name(spec):
    return (f'{spec["vehicle"]}-{spec["target_altitude"] / 1000:g}km-'
            f'{spec["pitch_programme"]["type"]}')


@pytest.mark.parametrize('spec', CATALOGUE, ids=[name(s) for s in CATALOGUE])
def test_entry_flies_as_recorded(spec):
    mission = mission_from_spec(spec, 'config')
    budget = velocity_budget(mission.run())
    recorded = spec['reached']
    target = spec['target_altitude'] / 1000

    assert mission.orbit.perigee_altitude / 1000 == pytest.approx(
        recorded['perigee_km'], abs=0.02)
    assert mission.orbit.apogee_altitude / 1000 == pytest.approx(
        recorded['apogee_km'], abs=0.02)
    assert budget.steering == pytest.approx(recorded['steering_loss'], abs=0.1)
    assert budget.total == pytest.approx(recorded['total_loss'], abs=0.1)
    # and the point of the entry: it reaches the orbit it is filed under
    assert abs(recorded['perigee_km'] - target) <= 1.0
    assert abs(recorded['apogee_km'] - target) <= 1.0


def test_catalogue_covers_every_vehicle_and_programme():
    """A guard against the catalogue being silently truncated.

    Which combinations are present is a property of the programmes - not every
    family reaches every orbit - so this checks breadth rather than a full grid.
    """
    assert {spec['vehicle'] for spec in CATALOGUE} == {'lv.f9', 'lv.a62', 'lv.h3'}
    assert {spec['pitch_programme']['type'] for spec in CATALOGUE} == {
        'five-phase', 'velocity-share', 'bilinear-tangent'}

    for vehicle in ('lv.f9', 'lv.a62', 'lv.h3'):
        altitudes = {spec['target_altitude'] for spec in CATALOGUE
                     if spec['vehicle'] == vehicle}
        assert len(altitudes) >= 5, f'{vehicle} covers only {sorted(altitudes)}'


def test_five_phase_sets_share_one_shape():
    """k2 is a design choice held across the catalogue, not a solved unknown."""
    five_phase = [spec['pitch_programme'] for spec in CATALOGUE
                  if spec['pitch_programme']['type'] == 'five-phase']
    assert five_phase
    assert {programme['k2'] for programme in five_phase} == {0.05}
