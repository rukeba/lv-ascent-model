"""The two estimates that bound a search, against the catalogue they bound."""

import math

import pytest

from ascent.config import load_catalogue, load_vehicle, mission_from_spec
from ascent.estimates import (analytic_altitude, burns, equivalent_time,
                              required_velocity, vacuum_time)

CATALOGUE = load_catalogue('config/catalogue.yaml')
VEHICLES = {name: load_vehicle(f'config/{name}.yaml')
            for name in ('lv.f9', 'lv.a62', 'lv.h3')}


def name(spec):
    return (f'{spec["vehicle"]}-{spec["target_altitude"] / 1000:g}km-'
            f'{spec["pitch_programme"]["type"]}')


def test_required_velocity_is_the_energy_of_the_orbit():
    """The figures the dissertation prints, which depend on the Earth alone."""
    assert required_velocity(400_000) == pytest.approx(8140, abs=1)
    assert required_velocity(1_200_000) == pytest.approx(8514, abs=1)
    # and it rises with altitude, unlike the circular speed it is built from -
    # which is the whole point of folding the lift into it
    heights = [required_velocity(h) for h in range(200_000, 1_400_000, 100_000)]
    assert heights == sorted(heights)


def test_burns_are_the_stages_that_burn():
    """The payload carries no propellant, so it is not a burn."""
    falcon = burns(VEHICLES['lv.f9'])
    assert len(falcon) == 2
    assert falcon[0].begin == 0.0
    # the first stage empties before the second lights, and the gap is a coast
    assert falcon[0].burn_out < falcon[0].end == falcon[1].begin


@pytest.mark.parametrize('vehicle, altitude',
                         [('lv.f9', 500_000), ('lv.a62', 700_000), ('lv.h3', 1_100_000)])
def test_vacuum_time_is_under_the_equivalent_time(vehicle, altitude):
    """Losses only ever cost time, so the ideal is the lower bound."""
    ideal = vacuum_time(VEHICLES[vehicle], altitude)
    with_losses = equivalent_time(VEHICLES[vehicle], altitude)
    assert 0.0 < ideal < with_losses


def test_an_orbit_out_of_reach_is_reported_as_such():
    """The balance never closes, and that is known without flying anything.

    Falcon 9 has the propellant for a 20 000 km orbit and not the trajectory:
    the ideal vacuum figure reaches it, the balance with the losses in it does
    not. Higher still and neither does, which is the criterion in its plainest
    form - the vehicle cannot get there however it is flown.
    """
    assert vacuum_time(VEHICLES['lv.f9'], 20_000_000) is not None
    assert equivalent_time(VEHICLES['lv.f9'], 20_000_000) is None
    assert vacuum_time(VEHICLES['lv.f9'], 100_000_000) is None


@pytest.mark.parametrize('spec', CATALOGUE, ids=[name(s) for s in CATALOGUE])
def test_equivalent_time_brackets_the_catalogue(spec):
    """Every solved cut-off on file falls inside the window the estimate sets.

    This is what the estimate is for: it is not accurate, but it is bounded,
    and `search.TIME_MARGIN_EARLY` and `TIME_MARGIN_LATE` are the band measured
    here. Widening them costs a longer search; narrowing them past this risks
    a search that cannot reach its own answer.
    """
    from ascent.search import TIME_MARGIN_EARLY, TIME_MARGIN_LATE

    estimate = equivalent_time(VEHICLES[spec['vehicle']], spec['target_altitude'])
    assert estimate is not None
    cut_off = spec['cutoff']['time']
    assert estimate * (1.0 - TIME_MARGIN_EARLY) < cut_off \
        < estimate * (1.0 + TIME_MARGIN_LATE)


@pytest.mark.parametrize('spec', CATALOGUE, ids=[name(s) for s in CATALOGUE])
def test_analytic_altitude_reads_high_but_bounded(spec):
    """The integral overstates the altitude, and by how much is what is bounded.

    It leaves out the air, the thrust deficit at sea level and the fall of
    gravity with altitude, all of which push the same way, so it never reads
    low. `search.ALTITUDE_RATIO_LOW` and `ALTITUDE_RATIO_HIGH` are the band
    measured here, and the screen of the search is that band applied backwards.
    """
    from ascent.search import ALTITUDE_RATIO_HIGH, ALTITUDE_RATIO_LOW

    mission = mission_from_spec(spec, 'config')
    predicted = analytic_altitude(VEHICLES[spec['vehicle']], mission.pitch_programme)
    ratio = predicted / spec['target_altitude']
    assert ALTITUDE_RATIO_LOW <= ratio <= ALTITUDE_RATIO_HIGH


def test_analytic_altitude_rises_with_the_cut_off():
    """A longer programme reaches higher, which is what the screen relies on."""
    from ascent.pitch import FivePhaseProgramme

    vehicle = VEHICLES['lv.f9']
    reached = [analytic_altitude(vehicle, FivePhaseProgramme(
        t1=20.0, t4=end, k2=0.05, k3=0.5)) for end in (460.0, 480.0, 500.0, 520.0)]
    assert reached == sorted(reached)
    assert all(math.isfinite(value) for value in reached)
