"""The parameter search: that it finds an orbit, and finds the one it says.

Every search here runs at one step a second rather than ten and over half the
nodes a real search would use. Neither changes what the search does; both
change how long it takes, and the orbit a set reaches is the same to within a
few metres at either step - `test_mission.py` is where that step is pinned
down. A search at the settings the command line defaults to is minutes of
integration, which is not what a test suite is for.
"""

import pytest

from ascent.config import load_catalogue, load_vehicle, mission_from_spec
from ascent.search import (FAMILIES, Axis, BilinearTangent, FivePhase,
                           SearchResult, VelocityShare, _Flight, _refine, search)

FALCON = load_vehicle('config/lv.f9.yaml')
CATALOGUE = load_catalogue('config/catalogue.yaml')


def quick(programme, altitude=500_000, **overrides):
    """A search coarse enough for a test but complete in every step of itself."""
    settings = dict(latitude_deg=28.5, azimuth_deg=90.0, coarseness=0.5,
                    steps_per_second=1)
    settings.update(overrides)
    return search(FALCON, altitude, programme, **settings)


def test_the_five_phase_search_recovers_the_set_on_file():
    """The catalogue set for this orbit, found again from nothing but the orbit.

    The search and the catalogue need not agree: the catalogue preferred the
    smallest steering loss among the sets that reach the orbit, and this
    prefers the earliest cut-off. For the five-phase family there is nothing to
    prefer - two terminal conditions and two unknowns leave no freedom - so
    they agree to the figures the grid resolves.
    """
    result = quick('five-phase')
    assert result.reaches_orbit, f'missed by {result.best.miss:.0f} m'
    assert result.best.cutoff_time == pytest.approx(502.707, abs=0.05)
    assert result.best.parameters['k3'] == pytest.approx(0.5296, abs=0.002)
    assert result.best.steering_loss == pytest.approx(516.9, abs=1.0)


def test_a_family_with_a_parameter_to_spare_reaches_the_orbit_sooner():
    """The velocity share keeps a third parameter, and the search spends it.

    The catalogue set for this orbit cuts off at 502.19 s, chosen for the
    smallest steering loss. Minimising the ascent instead has somewhere to go,
    so the set found here cuts off earlier - and, since an earlier cut-off on
    the same orbit is less propellant spent on the way, for less in total.
    """
    result = quick('velocity-share', tolerance=2_000.0, refinements=5)
    assert result.reaches_orbit, f'missed by {result.best.miss:.0f} m'
    assert result.best.cutoff_time < 502.19
    assert result.best.total_loss < 2996.4


# the catalogue sets for Falcon 9 to 500 km, read in the coordinates the grid
# uses: the share of the cut-off the turn ends at, and the angles the bilinear
# tangent starts at and has reached halfway
SHAPES = (
    (FivePhase(), {'k3': 0.5296}),
    (VelocityShare(), {'turn': 0.9732, 's': 1.1194}),
    (BilinearTangent(), {'start': 88.0, 'middle': 29.5}),
)


@pytest.mark.parametrize('family, shape', SHAPES,
                         ids=[family.name for family, _ in SHAPES])
def test_every_family_solves_its_cut_off_to_a_circular_orbit(family, shape):
    """One node of the grid solved for its cut-off, family by family.

    The node is the catalogue's own set for this orbit, so the cut-off found
    for it should be the catalogue's too. What is checked is not that, though,
    but the terminal condition the cut-off carries and the only one it carries:
    whatever altitude the shape reaches, the orbit at that cut-off is circular.
    """
    result = SearchResult(best=None, vehicle=FALCON, target_altitude=500_000,
                          programme=family.name, latitude_deg=28.5,
                          azimuth_deg=90.0, steps_per_second=1)
    window = (470.0, 570.0)
    flight = _Flight(FALCON, family, 20.0, 500_000, window, 28.5, 90.0, 1, result)

    candidate = flight.at(shape)
    assert candidate is not None, f'{family.name} solved no cut-off at all'
    assert candidate.cutoff_time == pytest.approx(502, abs=3)
    assert candidate.orbit.eccentricity < 1e-4
    assert abs(candidate.residual) <= flight.circular_tolerance
    assert candidate.miss < 3_000.0


def test_the_set_found_flies_again_from_its_own_specification():
    """What the search writes down reproduces what the search measured."""
    result = quick('five-phase')
    assert result.reaches_orbit
    spec = result.specification('lv.f9')
    assert spec['vehicle'] == 'lv.f9'
    assert spec['cutoff']['time'] == result.best.cutoff_time

    mission = mission_from_spec(spec, 'config')
    mission.run()
    assert mission.orbit.perigee_altitude == pytest.approx(
        result.best.orbit.perigee_altitude, abs=100)
    assert mission.orbit.apogee_altitude == pytest.approx(
        result.best.orbit.apogee_altitude, abs=100)


def test_the_search_stays_inside_the_estimated_window():
    result = quick('five-phase', refinements=1)
    early, late = result.window
    assert early < result.equivalent_time < late
    assert early <= result.best.cutoff_time <= late
    # and the estimate is a lower bound on the ascent, not a substitute for it
    assert result.vacuum_time < result.equivalent_time


def test_every_node_ends_in_exactly_one_count():
    """The five outcomes of a node are counted, not derived from each other.

    `screened`, `refused`, `unbracketed` and `no_orbit` are incremented where
    the node ends, and `closed` where it comes out on an orbit; a node that
    fell through all five would show up here as a node unaccounted for.
    """
    result = quick('velocity-share', refinements=1)
    assert result.nodes == (result.screened + result.refused
                            + result.unbracketed + result.no_orbit
                            + result.closed)
    assert result.closed == result.solved
    assert result.screened > 0, 'the altitude integral rejected nothing'
    assert result.closed > 0, 'no node came out on an orbit'
    assert result.flown > result.closed, 'every node takes several trajectories'
    # and the passes and their nodes were walked as they were planned to be
    assert result.planned_nodes == result.nodes
    assert result.passes == result.pass_number


def test_a_set_that_misses_is_not_written_out_as_an_entry():
    """`best` is the closest set found; only a set that reaches is filed.

    One pass of a grid this coarse cannot place the orbit within half a
    kilometre, so this is a search that has an answer to show and none to file.
    """
    result = quick('velocity-share', refinements=0, coarseness=0.3)
    assert not result.reaches_orbit
    assert result.best is not None
    with pytest.raises(ValueError, match='not a catalogue entry'):
        result.specification('lv.f9')


def test_a_tighter_tolerance_tightens_the_cut_off_solve():
    """The circularity the solve stops at is a share of what was asked for."""
    from ascent.search import CIRCULAR_SHARE, _Flight

    for tolerance in (500.0, 100.0):
        result = SearchResult(best=None, vehicle=FALCON, target_altitude=500_000,
                              programme='five-phase', latitude_deg=28.5,
                              azimuth_deg=90.0, steps_per_second=1,
                              tolerance=tolerance)
        flight = _Flight(FALCON, FivePhase(), 20.0, 500_000, (470.0, 570.0),
                         28.5, 90.0, 1, result)
        assert flight.circular_tolerance == tolerance * CIRCULAR_SHARE
        candidate = flight.at({'k3': 0.5296})
        assert abs(candidate.residual) <= flight.circular_tolerance


def test_an_orbit_out_of_reach_is_refused_before_anything_is_flown():
    with pytest.raises(ValueError, match='does not have the propellant'):
        search(FALCON, 20_000_000, 'five-phase')


def test_an_unknown_programme_is_refused():
    with pytest.raises(ValueError, match='unknown pitch programme'):
        search(FALCON, 500_000, 'gravity-turn')


def test_refining_halves_the_step_and_keeps_the_centre():
    axes = {'k3': Axis(0.0, 1.0, 5)}
    refined = _refine(axes, {'k3': 0.5}, {'k3': (0.0, 1.0)})
    assert (refined['k3'].low, refined['k3'].high) == (0.25, 0.75)
    # the centre is a node of the refined grid, so a pass can never do worse
    # than the pass before it
    assert refined['k3'].nodes % 2 == 1


def test_refining_stays_inside_the_range_the_family_gave():
    """A shape on a bound is reported rather than chased past it."""
    axes = {'s': Axis(-3.0, 3.0, 5)}
    refined = _refine(axes, {'s': 3.0}, {'s': (-3.0, 3.0)})
    assert (refined['s'].low, refined['s'].high) == (1.5, 3.0)


def test_the_families_cover_the_catalogue():
    """Every programme the catalogue holds is one the search can look for."""
    assert set(FAMILIES) == {spec['pitch_programme']['type'] for spec in CATALOGUE}
