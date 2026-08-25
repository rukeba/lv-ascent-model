"""Every catalogue entry, flown again and checked against what it records."""

from pathlib import Path

import pytest

from ascent import velocity_budget
from ascent.config import CATALOGUE_FILES, load_catalogue, mission_from_spec

CATALOGUE = load_catalogue('config')


def name(spec):
    return (f'{spec["vehicle"]}-{spec["target_altitude"] / 1000:g}km-'
            f'{spec["pitch_programme"]["type"]}')


@pytest.mark.parametrize('spec', CATALOGUE, ids=[name(s) for s in CATALOGUE])
def test_entry_flies_as_recorded(spec):
    mission = mission_from_spec(spec, 'config')
    budget = velocity_budget(mission.run(), mission.omega)
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
        assert len(altitudes) >= 3, f'{vehicle} covers only {sorted(altitudes)}'


def test_every_vehicle_is_kept_in_its_own_file():
    """One file a vehicle, and a file holds that vehicle and no other."""
    for path in sorted(Path('config').glob(CATALOGUE_FILES)):
        # catalogue.f9.yaml holds lv.f9, which is the whole of the naming rule
        vehicle = f'lv.{path.name.split(".")[1]}'
        entries = load_catalogue(path)
        assert entries, f'{path} holds no entries'
        assert {spec['vehicle'] for spec in entries} == {vehicle}


# What every instant of an entry is written in: the tenth of a second a
# timeline is issued to, and what the search asks its instants in
TENTH = 0.1


@pytest.mark.parametrize('spec', CATALOGUE, ids=[name(s) for s in CATALOGUE])
def test_every_instant_is_a_whole_tenth_of_a_second(spec):
    """A set on file is a set a vehicle could be given.

    The vertical rise, the end of the programme and the cut-off are instants of
    the flight, and a timeline is issued to a tenth of a second at best. The
    coefficients of the guidance law are not instants and nothing rounds them -
    which is why `tf` is left out here: the velocity share's turn ends where its
    quartic drives the vertical share of the speed to zero, and that is a number
    of the law rather than a moment anything happens at.
    """
    programme = spec['pitch_programme']
    instants = {'t1': programme['t1'], 'cutoff': spec['cutoff']['time']}
    for name_of in ('t4', 'te'):
        if name_of in programme:
            instants[name_of] = programme[name_of]

    for what, instant in instants.items():
        assert instant == pytest.approx(round(instant / TENTH) * TENTH, abs=1e-9),             f'{what} = {instant} is finer than a tenth of a second'
