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
from ascent.estimates import equivalent_time
from ascent.search import (FAMILIES, REFINED_NODES, VERTICAL_RISE, Axis,
                           BilinearTangent, FivePhase, VelocityShare, _count,
                           _Flight, _planned_nodes, _refine, search)

FALCON = load_vehicle('config/lv.f9.yaml')
CATALOGUE = load_catalogue('config/catalogue.yaml')


def quick(programme, altitude=500_000, **overrides):
    """A search coarse enough for a test but complete in every step of itself.

    One step a second rather than ten, half the nodes along each axis of the
    first pass, and the shape of the turn alone. None of the three changes what
    the search does; all three change how long it takes, and a five-phase
    search at the settings the command line defaults to is twelve thousand
    nodes, which is not what a test suite is for. The tests that are about the
    other axes open them and pay for it. The nodes are divided over processes
    as they would be in earnest.
    """
    settings = dict(latitude_deg=28.5, azimuth_deg=90.0, coarseness=0.5,
                    steps_per_second=1, free='none')
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
    assert result.best.cutoff_time == pytest.approx(502.712, abs=0.05)
    assert result.best.parameters['k3'] == pytest.approx(0.5296, abs=0.002)
    assert result.best.steering_loss == pytest.approx(516.8, abs=1.0)


def test_a_family_searches_everything_it_has_unless_it_is_told_not_to():
    """All four numbers of the five-phase turn, and `none` for the old one.

    The axes come in the order the family lists them, so that a grid is laid
    out the same way however the names were typed.
    """
    assert list(FivePhase().axes()) == ['k3', 't1', 'k2', 't4']
    assert list(FivePhase(free='none').axes()) == ['k3']
    assert list(FivePhase(free=('t4', 't1')).axes()) == ['k3', 't1', 't4']
    assert list(VelocityShare().axes()) == ['turn', 's', 't1']
    assert list(BilinearTangent(free=('middle_at',)).axes()) == \
        ['start', 'middle', 'middle_at']


def test_a_family_refuses_an_axis_it_does_not_have():
    """The velocity share has no k2, and says so before anything is flown."""
    with pytest.raises(ValueError, match='cannot search k2'):
        VelocityShare(free=('k2',))
    with pytest.raises(ValueError, match='`all`, `none` or a sequence'):
        FivePhase(free='everything')


def test_the_axes_searched_reach_the_programme():
    """A shape carrying t1, k2 and t4 builds the programme they describe."""
    family = FivePhase()
    shape = {'k3': 0.5, 't1': 14.0, 'k2': 0.01, 't4': 0.9}
    programme = family.build(VERTICAL_RISE, 500.0, shape)
    assert (programme.t1, programme.k2, programme.k3) == (14.0, 0.01, 0.5)
    # the axis is a share of the cut-off; what is flown and what is written
    # down is the instant it stands for
    assert programme.t4 == 450.0
    assert family.parameters(VERTICAL_RISE, 500.0, shape)['t4'] == 450.0

    # and a family told to search none of them holds what the caller passed,
    # whatever a shape happens to carry
    held = FivePhase(free='none').build(VERTICAL_RISE, 500.0, {'k3': 0.5})
    assert (held.t1, held.t4, held.k2) == (VERTICAL_RISE, 500.0, 0.05)


def test_the_fifth_phase_is_the_turn_ending_before_the_cut_off():
    """What opening t4 buys: free flight on the attitude the turn reached.

    The family is named for five phases and flies four of them while the turn
    ends with the burn, which is what it does unless t4 is searched.
    """
    window = (470.0, 570.0)
    flight = _Flight(FALCON, FivePhase(free=('t4',)), VERTICAL_RISE, 500_000,
                     window, 28.5, 90.0, 1, 50.0)

    node = flight.at({'k3': 0.5296, 't4': 0.93}, window)
    assert node.outcome == 'closed', f'came to {node.outcome}'
    found = node.candidate
    assert found.parameters['t4'] == pytest.approx(0.93 * found.cutoff_time)
    assert found.parameters['t4'] < found.cutoff_time


def test_an_axis_multiplies_the_pass_rather_than_adding_to_it():
    """Which is the whole cost of a search, and what `free='none'` buys back."""
    shape_alone = FivePhase(free='none').axes()
    everything = FivePhase().axes()
    assert _count(everything) == _count(shape_alone) * 5 * 15 * 4
    assert _planned_nodes(everything, 10) == \
        _count(everything) + 10 * REFINED_NODES ** 4


def test_the_band_is_every_set_as_cheap_as_the_one_reported():
    """The minimum is flat, and the band is the shape of that flatness.

    One pass over every axis, wide enough in the orbit for a grid this coarse
    to reach it: what is under test is that the band is a set of sets spread
    along the axes rather than the answer repeated. Its width is in the unit
    of the criterion, so a hundred here is a hundred metres per second of
    velocity budget.
    """
    result = quick('five-phase', free='all', refinements=0,
                   coarseness=0.4, tolerance=20_000.0, band_tolerance=200.0)
    assert result.reaches_orbit
    assert len(result.band) > 1, 'the band came out a single set'

    dearest = result.best.total_loss + result.band_tolerance
    assert all(found.miss <= result.tolerance for found in result.band)
    assert all(found.total_loss <= dearest for found in result.band)
    assert any(found.total_loss == result.best.total_loss
               for found in result.band)
    # spread along the axes that were opened, which is what makes it a band
    assert len({found.parameters['t1'] for found in result.band}) > 1


def test_nothing_that_reaches_the_orbit_is_no_band_at_all():
    """The closest set found is worth showing; it is not a band of solutions."""
    result = quick('five-phase', refinements=0, coarseness=0.3,
                   band_tolerance=5.0)
    assert not result.reaches_orbit
    assert result.band == ()


def test_a_search_without_a_band_asked_for_reports_what_it_found():
    """A band of nothing but the answer, which is what the catalogue holds.

    Nothing but, rather than only: the ranking rounds together sets it cannot
    tell apart, so a set a shade cheaper than the one reported belongs to the
    band of it.
    """
    result = quick('five-phase')
    assert result.band_tolerance == 0.0
    assert result.reaches_orbit
    assert result.band, 'the set found is a band of one, not of none'
    assert all(found.total_loss <= result.best.total_loss
               for found in result.band)


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
    (FivePhase(free='none'), {'k3': 0.5296}),
    (VelocityShare(free='none'), {'turn': 0.9732, 's': 1.1194}),
    (BilinearTangent(free='none'), {'start': 87.934, 'middle': 29.506}),
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
    window = (470.0, 570.0)
    flight = _Flight(FALCON, family, 20.0, 500_000, window, 28.5, 90.0, 1, 50.0)

    node = flight.at(shape, window)
    assert node.outcome == 'closed', f'{family.name} came to {node.outcome}'
    candidate = node.candidate
    assert candidate.cutoff_time == pytest.approx(502, abs=3)
    assert candidate.orbit.eccentricity < 1e-4
    assert abs(candidate.residual) <= flight.circular_tolerance
    assert candidate.miss < 3_000.0
    assert node.flights > 2, 'a cut-off takes more than its two bracket ends' 


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

    `screened`, `refused`, `no_cut_off` and `no_orbit` are incremented where
    the node ends, and `closed` where it comes out on an orbit; a node that
    fell through all five would show up here as a node unaccounted for.
    """
    result = quick('velocity-share', refinements=1)
    assert result.nodes == (result.screened + result.refused
                            + result.no_cut_off + result.no_orbit
                            + result.closed)
    assert result.closed == result.solved
    assert result.screened > 0, 'the altitude integral rejected nothing'
    assert result.closed > 0, 'no node came out on an orbit'
    assert result.flown > result.closed, 'every node takes several trajectories'


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
    from ascent.search import CIRCULAR_SHARE

    window = (470.0, 570.0)
    for tolerance in (500.0, 100.0):
        stop = tolerance * CIRCULAR_SHARE
        flight = _Flight(FALCON, FivePhase(free='none'), 20.0, 500_000, window,
                         28.5, 90.0, 1, stop)
        node = flight.at({'k3': 0.5296}, window)
        assert node.outcome == 'closed'
        assert abs(node.candidate.residual) <= stop

    # and the search hands the solve a stop derived from what it was asked for
    result = quick('five-phase', refinements=0, tolerance=100.0)
    assert result.tolerance == 100.0


def test_an_orbit_out_of_reach_is_refused_before_anything_is_flown():
    """And refused on the most generous reading of what the vehicle has.

    The balance the refusal is made on credits the vehicle with the speed the
    pad hands it, which the estimate itself does not carry. At 20 000 km it
    does not close either way; at 5 000 km it closes only with the pad in it,
    and a search that refused there would be refusing an orbit Falcon 9 has
    been flown to.
    """
    with pytest.raises(ValueError, match='does not reach a circular orbit'):
        search(FALCON, 20_000_000, 'five-phase')

    assert equivalent_time(FALCON, 5_000_000) is None
    assert equivalent_time(FALCON, 5_000_000, head_start=408.0) is not None


def test_an_orbit_inside_the_air_is_refused():
    """The model takes the air as gone above 100 km and has no orbit below it."""
    with pytest.raises(ValueError, match='inside the air'):
        search(FALCON, 90_000, 'five-phase')


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


def test_dividing_the_grid_over_processes_finds_the_same_set():
    """A pool answers the nodes of a pass; it does not change what they answer.

    The nodes of a pass are independent and are collected in the order of the
    grid, so a search returns the same set however many processes it was
    divided over - and the same count of nodes and of trajectories, which is
    what says the division was of the work and not of the answer.
    """
    alone = quick('five-phase', refinements=3, workers=1)
    together = quick('five-phase', refinements=3, workers=2)

    assert (alone.workers, together.workers) == (1, 2)
    assert together.best.cutoff_time == alone.best.cutoff_time
    assert together.best.shape == alone.best.shape
    assert (together.nodes, together.flown) == (alone.nodes, alone.flown)


def test_the_work_is_counted_out_before_any_of_it_is_done():
    """The progress a search reports is against a total known in advance.

    `planned_nodes` is read from the first node rather than from the finished
    result, where it has been corrected to what was actually walked: what is
    under test is the figure the progress line divides by while the search is
    still running.
    """
    planned = []
    quick('five-phase', refinements=3,
          report=lambda result: planned or planned.append(
              (result.planned_nodes, result.passes)))

    nodes, passes = planned[0]
    assert passes == 4
    # the first pass over the whole range of the family, then three of five
    # nodes each closing in
    assert nodes == 10 + 3 * 5


def test_the_fallback_runs_when_ranking_by_cost_reaches_nothing():
    """A search that reaches nothing runs the grid again for the orbit alone.

    Forced here by asking for the orbit to a metre, which no grid this coarse
    can meet, so that the branch is exercised whether or not a vehicle near its
    limit is to hand. Both attempts are walked and both are counted.

    Only where the ranking was by what the ascent costs. Rank by the orbit and
    the first attempt was already the fallback, so running it again would walk
    the same nodes to the same answer.
    """
    result = quick('five-phase', refinements=0, tolerance=1.0, criterion='loss')

    assert not result.reaches_orbit
    assert result.attempts == 2
    assert result.pass_number == 2, 'the grid was not run a second time'
    assert result.nodes == 2 * 10
    # and the closest set found is still there to be shown
    assert result.best is not None

    ranked_by_orbit = quick('five-phase', refinements=0, tolerance=1.0)
    assert not ranked_by_orbit.reaches_orbit
    assert ranked_by_orbit.attempts == 1
    assert ranked_by_orbit.nodes == 10


def test_a_set_that_overstresses_the_airframe_is_not_an_answer():
    """`max_dynamic_pressure` takes a set out of the ranking, however quick.

    Falcon 9 to 500 km peaks around 35 kPa on every set that reaches it, so a
    limit of 25 leaves the search nothing to return - and it says so rather
    than returning the quickest set that broke the vehicle.
    """
    unconstrained = quick('five-phase', refinements=3, tolerance=5_000.0)
    assert unconstrained.reaches_orbit
    assert unconstrained.best.peak_dynamic_pressure > 25_000.0
    assert unconstrained.over_pressure == 0

    constrained = quick('five-phase', refinements=3, tolerance=5_000.0,
                        max_dynamic_pressure=25_000.0)
    assert constrained.over_pressure > 0
    assert constrained.best is None
    assert not constrained.reaches_orbit
