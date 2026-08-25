"""The two estimates that bound a search, against the catalogue they bound."""

import math

import pytest

from ascent.config import load_catalogue, load_vehicle, mission_from_spec
from ascent.estimates import (analytic_altitude, burns, equivalent_time,
                              required_velocity, vacuum_time)

CATALOGUE = load_catalogue('config')
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


# How far the estimate is out across the whole catalogue, measured: between
# 4.8 per cent high and 9.1 per cent low, written here as the band the cut-off
# on file sits in relative to it, with a hair of rounding either way. These are
# literals on purpose - the test below is what pins the measurement down, so it
# must not be able to move with the constants the search derives from it
ESTIMATE_LOW, ESTIMATE_HIGH = 0.954, 1.101


@pytest.mark.parametrize('spec', CATALOGUE, ids=[name(s) for s in CATALOGUE])
def test_equivalent_time_brackets_the_catalogue(spec):
    """Every solved cut-off on file sits this close to the estimate.

    This is what the estimate is for: it is not accurate, but it is bounded.
    """
    estimate = equivalent_time(VEHICLES[spec['vehicle']], spec['target_altitude'])
    assert estimate is not None
    assert ESTIMATE_LOW <= spec['cutoff']['time'] / estimate <= ESTIMATE_HIGH


def test_the_search_window_covers_the_measured_band():
    """And the window the search looks in covers that band with room over.

    Kept apart from the measurement above so that neither can quietly follow
    the other: widening the margins cannot make the measurement pass, and a
    measurement that drifted would fail on its own.
    """
    from ascent.search import TIME_MARGIN_EARLY, TIME_MARGIN_LATE

    assert 1.0 - TIME_MARGIN_EARLY < ESTIMATE_LOW
    assert ESTIMATE_HIGH < 1.0 + TIME_MARGIN_LATE


# What the integral reads high by across the whole catalogue, measured, with a
# hair of rounding either way. Literals for the same reason as above
ALTITUDE_LOW, ALTITUDE_HIGH = 1.004, 1.186


@pytest.mark.parametrize('spec', CATALOGUE, ids=[name(s) for s in CATALOGUE])
def test_analytic_altitude_reads_high_but_bounded(spec):
    """The integral overstates the altitude, and by how much is what is bounded.

    It leaves out the air, the thrust deficit at sea level and the fall of
    gravity with altitude, all of which push the same way, so it never reads
    low.
    """
    mission = mission_from_spec(spec, 'config')
    predicted = analytic_altitude(VEHICLES[spec['vehicle']], mission.pitch_programme)
    assert ALTITUDE_LOW <= predicted / spec['target_altitude'] <= ALTITUDE_HIGH


def test_the_screen_covers_the_measured_band():
    """And the screen of the search covers that band with room over.

    More room than the window above gets, because the screen is a gate: a node
    it rejects is never flown, so a fourth vehicle reading further out than
    these three would be reported as unable to reach an orbit it can reach.
    """
    from ascent.search import ALTITUDE_RATIO_HIGH, ALTITUDE_RATIO_LOW

    assert ALTITUDE_RATIO_LOW < ALTITUDE_LOW
    assert ALTITUDE_HIGH < ALTITUDE_RATIO_HIGH


def test_analytic_altitude_rises_with_the_cut_off():
    """A longer programme reaches higher, which is what the screen relies on."""
    from ascent.pitch import FivePhaseProgramme

    vehicle = VEHICLES['lv.f9']
    reached = [analytic_altitude(vehicle, FivePhaseProgramme(
        t1=20.0, t4=end, k2=0.05, k3=0.5)) for end in (460.0, 480.0, 500.0, 520.0)]
    assert reached == sorted(reached)
    assert all(math.isfinite(value) for value in reached)
