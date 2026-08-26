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


# Every combination the catalogue holds, written out. Which of them are present
# is not a rule - it is what searching thirty of them returned, and the nine
# that are absent are absent for three different reasons, each named in the
# header of the file it belongs to. That is exactly why this is a list and not
# a count: a rule could be restated, but a set that stopped being found, or one
# that started being found, is a change in what the search does and has to be
# read and agreed to rather than absorbed.
COVERED = {
    'lv.f9': {
        400_000: {'five-phase', 'velocity-share', 'bilinear-tangent'},
        500_000: {'five-phase', 'velocity-share', 'bilinear-tangent'},
        600_000: {'five-phase', 'velocity-share', 'bilinear-tangent'},
        700_000: {'five-phase', 'bilinear-tangent'},
    },
    'lv.a62': {
        400_000: {'velocity-share'},
        500_000: {'velocity-share'},
        600_000: {'velocity-share'},
    },
    'lv.h3': {
        1_000_000: {'five-phase', 'velocity-share', 'bilinear-tangent'},
        1_100_000: {'five-phase', 'velocity-share', 'bilinear-tangent'},
        1_200_000: {'five-phase', 'velocity-share', 'bilinear-tangent'},
    },
}


def test_catalogue_holds_exactly_the_sets_that_were_found():
    """A guard against the catalogue being silently truncated, or grown.

    Counting entries would not do it. The test that flies them is parametrised
    over the file, so an entry that vanished would take its own check with it
    and the suite would pass on what was left; and a breadth check loose enough
    to allow the gaps that are there is loose enough to allow most of a vehicle
    to go missing. So the whole matrix is pinned, both ways.
    """
    held: dict[str, dict[int, set[str]]] = {}
    for spec in CATALOGUE:
        held.setdefault(spec['vehicle'], {}).setdefault(
            spec['target_altitude'], set()).add(spec['pitch_programme']['type'])

    assert held == COVERED

    # and no combination is filed twice, which the sets above would hide
    assert len(CATALOGUE) == sum(len(programmes) for vehicle in COVERED.values()
                                 for programmes in vehicle.values())


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
