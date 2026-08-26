"""The parameter search: that every parameter is on the grid, and that it works.

The searches here are narrow on purpose. What is under test is the machinery -
the grid, the ranges, the ranking, the passes that close in - and not how long
a sweep of a whole family takes, so every one of them holds most of the axes
and sweeps a few nodes of the rest, at one integration step a second rather
than ten. The whole file is a few seconds; a search at the settings the command
line defaults to is ten thousand trajectories, which is not what a test suite
is for.

The step does not change what a search finds. The orbit a set reaches is the
same to within a few metres at either step - `test_mission.py` is where that is
pinned down - and the set recovered below is the catalogue's own to four
figures.
"""

import copy
import dataclasses
import inspect
import subprocess
import sys

import pytest

from ascent.config import (PITCH_PROGRAMMES, load_catalogue, load_vehicle,
                           mission_from_spec)
from ascent.constants import circular_velocity
from ascent.estimates import equivalent_time
from ascent.orbit import Orbit
from ascent.search import (FAMILIES, NODE_LIMIT, REFINED_NODES,
                           BilinearTangent, Candidate, FivePhase, Range,
                           VelocityShare, _centres, _closer, _nodes,
                           _planned_nodes, axis_names, parse_ranges, plan,
                           search)
from ascent.summary import summarise_plan

FALCON = load_vehicle('config/lv.f9.yaml')
CATALOGUE = load_catalogue('config')
SITE = {'latitude_deg': 28.5, 'azimuth_deg': 90.0}

# A five-phase grid narrowed on to the catalogue's own set for 500 km: the
# vertical rise held where the catalogue holds it, and three values each of k2,
# k3 and the end of the turn. Some 310 trajectories, seven seconds.
#
# Both of the shape parameters are searched and neither is held, because
# between them they have to carry both terminal conditions. The cut-off cannot
# help: it is asked for in tenths of a second and a tenth is some eight
# kilometres of apogee, which is sixteen times the tolerance below.
NARROW = {'t1': Range(20.0, 20.0), 'k2': Range(0.03, 0.07, 3),
          'k3': Range(0.50, 0.56, 3), 't4': Range(502.0, 503.0, 3),
          'angle': Range(0.0, 0.0), 'coast': Range(0.0, 0.0)}


def quick(**overrides):
    """A search narrow enough for a test and complete in every step of itself."""
    settings = {'ranges': NARROW, 'refinements': 6, 'steps_per_second': 1,
                'workers': 1, 'tolerance': 1000.0, **SITE}
    settings.update(overrides)
    return search(FALCON, 500_000, 'five-phase', **settings)


@pytest.fixture(scope='module')
def found():
    """One narrow search, shared by every test that only reads its result."""
    return quick()


# --- every parameter is on the grid ---------------------------------------


# What carries each parameter of each programme, where an axis of the grid does
# not carry it under its own name. The velocity share takes the end of its turn
# as a share of the end of the programme, so that no node of the grid can ask
# for a turn that outlasts the burn; the bilinear tangent takes the three
# angles its turn passes through and recovers its coefficients from them,
# because a, b and c are nearly degenerate and a grid over them would spend
# most of its nodes on programmes it had already flown.
CARRIED_BY = {
    'five-phase': {'final_angle_deg': ('angle',)},
    'velocity-share': {'tf': ('turn', 'te')},
    'bilinear-tangent': {name: ('start', 'mid', 'middle', 'angle')
                         for name in 'abc'},
}

# and the axis each family ends its programme on. The engines stop `coast`
# later, which is an axis of every family and nought by default
PROGRAMME_ENDS = {'five-phase': 't4', 'velocity-share': 'te',
                  'bilinear-tangent': 'te'}


# Every axis of every family, in the order a grid walks them. Written out
# rather than derived, so that an axis appearing or disappearing has to be
# said here as well - which is what keeps the prose about them honest.
#
# The velocity share is the one without an `angle`: its quartic drives the
# vertical share of the speed to exactly zero at the end of the turn, so it
# arrives at the horizon by construction and has no parameter to aim.
AXES = {
    'five-phase': ('t1', 'k2', 'k3', 't4', 'angle', 'coast'),
    'velocity-share': ('t1', 'turn', 's', 'te', 'coast'),
    'bilinear-tangent': ('t1', 'start', 'mid', 'middle', 'te', 'angle', 'coast'),
}


@pytest.mark.parametrize('programme', sorted(FAMILIES))
def test_a_family_has_the_axes_it_says_it_has(programme):
    """And every family the search knows has its axes written down here."""
    assert axis_names(programme) == AXES[programme]


@pytest.mark.parametrize('programme', sorted(FAMILIES))
def test_every_parameter_of_a_programme_is_on_the_grid(programme):
    """The point of the search: nothing is held where the caller cannot see it.

    Every argument the pitch programme is built from is an axis of the grid, by
    its own name or through the ones named above, and so is the instant the
    engines stop. A parameter that stopped being searched would fail here
    rather than quietly returning one set per target altitude.
    """
    axes = axis_names(programme)
    carried = CARRIED_BY[programme]
    for name in inspect.signature(PITCH_PROGRAMMES[programme]).parameters:
        by = carried.get(name, (name,))
        missing = [axis for axis in by if axis not in axes]
        assert not missing, f'{name} is carried by {by}, and {missing} is not on the grid'

    assert PROGRAMME_ENDS[programme] in axes
    assert 'coast' in axes, 'the cut-off is not on the grid'


@pytest.mark.parametrize('family, end', [(FivePhase(), 't4'),
                                         (VelocityShare(), 'te'),
                                         (BilinearTangent(), 'te')],
                         ids=sorted(FAMILIES))
def test_the_engines_stop_a_coast_after_the_programme_does(family, end):
    """`coast` is powered flight on the attitude the turn reached.

    Nought by default, which is where every set on file has it and what makes
    the programme end at cut-off; above nought the programme ends first and the
    vehicle holds what it reached until the engines stop.
    """
    values = {name: span.low for name, span in family.ranges((480.0, 520.0)).items()}
    values[end] = 480.0

    programme, cutoff = family.build({**values, 'coast': 0.0})
    assert cutoff == pytest.approx(480.0)
    assert programme.end_time == pytest.approx(480.0, abs=0.1)

    programme, cutoff = family.build({**values, 'coast': 7.5})
    assert cutoff == pytest.approx(487.5)
    assert programme.end_time == pytest.approx(480.0, abs=0.1)


def test_the_grid_asks_for_an_instant_in_tenths_of_a_second():
    """No vehicle begins its turn at 14.2578 s, and none at 14.2676 either.

    A timeline is issued to a tenth of a second at best, so those are one
    instant and not two: one node of the grid, one trajectory, one row of the
    table. The shape of the turn is not rounded - those are coefficients of a
    guidance law rather than moments of a flight - so k2 keeps all three of its
    values here while t1 comes back with one.
    """
    grid = {'t1': Range(14.2578, 14.2676, 2), 'k2': Range(0.03, 0.07, 3)}
    walked = list(_nodes(grid, FivePhase.TIMES))

    assert sorted({one['t1'] for one in walked}) == [14.3]
    assert len({one['k2'] for one in walked}) == 3


def test_every_instant_of_the_set_found_is_a_tenth(found):
    """And the set the search answers with is one a vehicle could be given."""
    best = found.best
    for name in FivePhase.TIMES:
        assert best.values[name] == round(best.values[name], 1)
    assert best.cutoff_time == round(best.cutoff_time, 1)
    assert found.specification('lv.f9')['cutoff']['time'] == best.cutoff_time


def test_the_families_cover_the_catalogue():
    """Every programme the catalogue holds is one the search can look for."""
    assert set(FAMILIES) == {spec['pitch_programme']['type'] for spec in CATALOGUE}


def test_a_family_that_is_not_one_is_refused():
    with pytest.raises(ValueError, match='unknown pitch programme'):
        axis_names('gravity-turn')


# --- ranges ---------------------------------------------------------------


def test_a_range_is_a_count_of_values_between_two_ends():
    span = Range(10.0, 30.0, 5)
    assert span.values() == (10.0, 15.0, 20.0, 25.0, 30.0)
    assert span.step == pytest.approx(5.0)
    assert span.describe() == '10 to 30, 5 values, step 5'


def test_both_ends_of_a_range_are_values_the_search_tries():
    """A count divides the range; it does not walk off the end of it.

    Ten values from 10 to 25 is one every 1.667, and the last of them is 25
    itself rather than whatever the arithmetic left behind.
    """
    span = Range(10.0, 25.0, 10)
    assert span.values()[0] == 10.0
    assert span.values()[-1] == 25.0
    assert span.step == pytest.approx(15.0 / 9.0)


def test_a_parameter_is_held_by_a_range_of_one_value():
    span = Range(0.05, 0.05)
    assert span.nodes == 1
    assert span.values() == (0.05,)
    assert span.step == 0.0
    assert span.describe() == '0.05, held'


def test_a_count_written_as_a_whole_number_of_any_kind_is_the_same_range():
    """The command line reads its numbers as floats and this is where they land."""
    assert Range(0.0, 1.0, 5.0) == Range(0.0, 1.0, 5)


@pytest.mark.parametrize('text, expected', [
    ('t1=10:25:10', ('t1', Range(10.0, 25.0, 10))),
    ('k2=0.05', ('k2', Range(0.05, 0.05, 1))),
    ('s=-3:3:9', ('s', Range(-3.0, 3.0, 9))),
    (' t1 = 12:30:4 ', ('t1', Range(12.0, 30.0, 4))),
])
def test_a_range_is_read_off_the_command_line(text, expected):
    assert Range.parse(text) == expected


@pytest.mark.parametrize('text, complaint', [
    ('t1:10:30:2', 'is not a range'),
    ('=10:30:2', 'is not a range'),
    ('t1=', 'is not a range'),
    ('t1=ten:30:2', 'made of numbers'),
    ('t1=10:30', 'takes one or three'),
    ('t1=10:20:30:40', 'takes one or three'),
    ('t1=10:30:2.5', 'whole number of values'),
    ('t1=10:30:0', 'whole number of values'),
    ('t1=10:30:1', 'does not say which of them'),
    ('t1=20:20:4', 'are one value'),
    ('t1=30:10:2', 'runs from low to high'),
])
def test_a_range_that_is_not_one_says_what_one_looks_like(text, complaint):
    with pytest.raises(ValueError, match=complaint):
        Range.parse(text)


def test_a_parameter_given_two_ranges_is_refused():
    """The second would silently replace the first, which is not an answer."""
    with pytest.raises(ValueError, match='given a range twice'):
        parse_ranges(['t1=10:30:6', 't1=12:20:4'])


# --- the grid, before it is walked ----------------------------------------


def outline(**overrides):
    settings = {'ranges': NARROW, 'refinements': 7, **SITE}
    settings.update(overrides)
    return plan(FALCON, 500_000, 'five-phase', **settings)


def test_the_grid_takes_the_ranges_it_is_given():
    grid = outline(ranges={'t1': Range(10.0, 30.0, 11)}).ranges
    assert grid['t1'] == Range(10.0, 30.0, 11)
    # and every other axis keeps the range the family gave it
    assert grid['k3'] == FivePhase().ranges((0.0, 1.0))['k3']


def test_a_parameter_the_family_does_not_have_is_refused():
    with pytest.raises(ValueError, match='not a parameter of the five-phase'):
        outline(ranges={'kick': Range(1.0, 2.0, 3)})


def test_the_cut_off_is_searched_over_the_window_the_estimate_gives():
    """The dissertation's ascent-time estimate is what bounds the axis.

    Not a guess and not the whole of the burn: the instant the propellant has
    bought the orbit, with a margin either side that the catalogue was measured
    against.
    """
    result = outline(ranges={})
    early, late = result.window
    assert early < equivalent_time(FALCON, 500_000) < late
    assert result.ranges['t4'].low == pytest.approx(early)
    assert result.ranges['t4'].high == pytest.approx(late)
    # and the estimate is a lower bound on the ascent, not a substitute for it
    assert result.vacuum_time < result.equivalent_time


def test_the_work_is_counted_out_before_any_of_it_is_done():
    """The sweep, and then five nodes an axis for every valley of every pass."""
    result = outline(refinements=3, basins=1)
    assert result.passes == 4
    assert result.planned_nodes == 3 * 3 * 3 + 3 * REFINED_NODES ** 3

    # the upper bound of it: a pass closes in on as many valleys as the ranking
    # offers, and where two lie near each other the nodes they share are walked
    # once, so a search that follows several reports rather fewer than this
    followed = outline(refinements=3, basins=4)
    assert followed.planned_nodes == 3 * 3 * 3 + 4 * 3 * REFINED_NODES ** 3


def test_a_coarser_grid_is_a_thinner_one():
    """`coarseness` lengthens the stride of the axes the family gave.

    An axis the caller wrote out is left alone: that step was asked for.
    """
    given = {'k3': Range(0.0, 0.9, 19)}
    fine = outline(ranges=given, coarseness=1.0)
    rough = outline(ranges=given, coarseness=0.5)

    assert rough.ranges['t4'].nodes < fine.ranges['t4'].nodes
    assert rough.ranges['k3'] == fine.ranges['k3'] == given['k3']


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


def test_a_grid_too_large_to_walk_is_refused_before_it_is_walked():
    """A step mistyped by a factor of a hundred is a grid a hundred times over.

    The first thing a pass does is lay every one of its nodes out in memory, so
    a grid past the limit is refused rather than started on - and `plan` is not
    held to it, because looking at such a grid is how it gets found.
    """
    enormous = {'k3': Range(0.0, 0.9, 6_000_001)}
    assert outline(ranges=enormous).ranges['k3'].nodes > NODE_LIMIT

    with pytest.raises(ValueError, match='past the'):
        quick(ranges=enormous)


# --- closing in -----------------------------------------------------------


def test_closing_in_halves_the_step_and_keeps_the_centre():
    grid = {'k3': Range(0.0, 1.0, 5)}
    closer = _closer(grid, {'k3': 0.5}, {'k3': 0.25}, {'k3': (0.0, 1.0)})

    assert (closer['k3'].low, closer['k3'].high) == (0.25, 0.75)
    assert closer['k3'].step == pytest.approx(0.125)
    # the centre is a node of the closer grid, so a pass can never do worse
    # than the pass before it
    assert 0.5 in closer['k3'].values()


def test_closing_in_stays_inside_the_range_it_was_given():
    grid = {'s': Range(-3.0, 3.0, 5)}
    closer = _closer(grid, {'s': 3.0}, {'s': 1.5}, {'s': (-3.0, 3.0)})
    assert (closer['s'].low, closer['s'].high) == (1.5, 3.0)


def test_a_held_parameter_is_not_closed_in_on():
    """A range of one node is what holding a parameter is, and it stays one."""
    grid = {'k2': Range(0.05, 0.05)}
    assert _closer(grid, {'k2': 0.05}, {}, {'k2': (0.05, 0.05)}) == grid


# --- which valleys are closed in on ----------------------------------------

# a sweep whose steps are 10 on one axis and 1 on the other. `te` is an instant
# of the flight, so two sets are two valleys there once they differ by a tenth
# of a second - the resolution the answer is written in - where on `start`,
# which is a coefficient of the guidance law, it takes a whole step of the sweep
SWEEP = {'te': Range(100.0, 200.0, 11), 'start': Range(0.0, 10.0, 11)}
TIMES = ('te',)


def _set(**values):
    """A candidate that is nothing but its coordinates, which is all `_centres`
    reads: the ranking is the order they are handed over in."""
    return Candidate(values=values, parameters={}, cutoff_time=0.0,
                     orbit=Orbit(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), altitude=0.0,
                     speed=0.0, flight_path_angle=0.0, altitude_miss=0.0,
                     speed_miss=0.0, miss=0.0)


def test_two_sets_in_one_cell_of_the_sweep_are_one_valley():
    """And the pass that closes in on the better of them closes in on both.

    Both axes have to fold for the sets to be one valley: the same cut-off to
    the tenth, and within one sweep step on the coefficient.
    """
    found = [_set(te=150.0, start=5.0),
             _set(te=150.0, start=5.5),   # the same instant, half a step over
             _set(te=150.0, start=9.0)]   # four steps over
    assert [centre['start'] for centre in _centres(found, SWEEP, TIMES, 5)] \
        == [5.0, 9.0]


def test_a_set_a_step_away_on_one_axis_alone_is_another_valley():
    """The axes are read together: one of them out of reach is enough."""
    found = [_set(te=150.0, start=5.0), _set(te=150.0, start=7.0)]
    assert len(_centres(found, SWEEP, TIMES, 5)) == 2


def test_a_cut_off_one_tenth_away_is_another_valley():
    """The point of the whole thing, and the reason a tenth is the scale.

    An axis of instants stops being closed in on once its window is under a
    tenth of a second, and from then it does not move. So a set whose cut-off is
    one tenth from the best is a turn that will never be solved unless it is
    followed as a valley of its own - and the orbit answers to that tenth at
    kilometres of apogee. Read at the sweep step, which is five seconds here,
    every one of these would be the same valley as the head.
    """
    found = [_set(te=150.0, start=5.0), _set(te=150.1, start=5.0),
             _set(te=149.9, start=5.0), _set(te=150.2, start=5.0)]
    assert [centre['te'] for centre in _centres(found, SWEEP, TIMES, 9)] \
        == [150.0, 150.1, 149.9, 150.2]


def test_a_coefficient_is_not_read_at_the_tenth_a_cut_off_is():
    """`start` is a coefficient of the guidance law, not a moment of the flight.

    Nothing rounds it and nothing stops closing in on it, so the scale that
    separates two of its valleys is the step of the sweep - a tenth apart on
    that axis is deep inside one valley, not beside it.
    """
    found = [_set(te=150.0, start=5.0), _set(te=150.0, start=5.1)]
    assert len(_centres(found, SWEEP, TIMES, 9)) == 1


def test_the_valleys_are_taken_in_the_order_they_are_ranked_in():
    found = [_set(te=100.0 + 20 * step, start=0.0) for step in range(6)]
    assert [centre['te'] for centre in _centres(found, SWEEP, TIMES, 3)] \
        == [100.0, 120.0, 140.0]


def test_one_valley_is_the_head_of_the_ranking_and_nothing_else():
    """`--basins 1` is the search as it closed in before it followed several."""
    found = [_set(te=150.0, start=5.0), _set(te=190.0, start=9.0)]
    assert _centres(found, SWEEP, TIMES, 1) == [found[0].values]


def test_a_search_with_nothing_found_has_no_valley_to_close_in_on():
    assert _centres([], SWEEP, TIMES, 5) == []


def test_a_search_reports_the_valleys_it_followed_not_the_ones_it_asked_for():
    """Five asked for and one offered is a search that followed one.

    `basins` is the count asked for until a pass has run and what the ranking
    actually gave thereafter, so the summary of a finished search describes
    what it did. A grid this narrow puts every set it finds inside one cell of
    the sweep, which is one valley however many were asked for.
    """
    narrow = {'t1': Range(20.0, 20.0), 'k2': Range(0.056, 0.056),
              'k3': Range(0.52, 0.53, 2), 't4': Range(502.8, 502.9, 2),
              'angle': Range(0.0, 0.0), 'coast': Range(0.0, 0.0)}
    result = search(FALCON, 500_000, 'five-phase', ranges=narrow,
                    refinements=2, steps_per_second=1, workers=1,
                    tolerance=5000.0, basins=5, **SITE)

    assert 1 <= result.basins < 5
    assert f'best {result.basins} sets' in summarise_plan(result)

    # and before anything has run it is the count that was asked for, because
    # that is all a plan can say
    assert 'best 5 sets' in summarise_plan(
        plan(FALCON, 500_000, 'five-phase', basins=5, **SITE))


def test_the_valleys_counted_are_the_valleys_a_pass_walked():
    """The last pass has nothing after it, so what it would pick is not counted.

    Valleys are picked from the ranking at the end of every pass and closed in
    on by the pass after. There is no pass after the last one, so a count taken
    there would report valleys as followed that nothing ever walked - and it
    would be the widest count of the search, since the ranking is largest at the
    end. A search with no refinements at all is the plainest case of it: one
    sweep, no pass that closes in, and so no valley followed however many were
    asked for.
    """
    swept = quick(refinements=0, basins=5)
    assert swept.passes == 1
    assert swept.basins == 5, 'the count asked for, never having been used'
    assert 'and no more' in summarise_plan(swept)

    # and with passes, the count is one a pass actually closed in on
    closed = quick(refinements=2, basins=5)
    assert closed.passes == 3
    assert 1 <= closed.basins <= 5


def test_following_several_valleys_costs_several_passes_and_no_more():
    """What a pass costs is what says whether following more is affordable."""
    grid = {'k3': Range(0.0, 0.9, 19), 't4': Range(400.0, 500.0, 41)}
    one = _planned_nodes(grid, refinements=10, basins=1)
    five = _planned_nodes(grid, refinements=10, basins=5)

    sweep = 19 * 41
    assert one == sweep + 10 * REFINED_NODES ** 2
    assert five - sweep == 5 * (one - sweep)


# --- the table -------------------------------------------------------------


def test_a_column_of_instants_prints_them_as_they_are_asked_for():
    """25.7 and not 25.70: an axis of instants lands on the decimal lattice.

    The spare digit `_decimals` carries is for values that do not, and given
    the values themselves it can see that these do and drop it.
    """
    from ascent.summary import _decimals

    assert _decimals(0.1, [25.7, 502.8, 14.3]) == 1
    # and it is still carried where the values are off the lattice
    assert _decimals(0.1, [25.75, 502.85]) == 2
    assert _decimals(0.1) == 2


@pytest.mark.parametrize('low, step', [(0.5, 1.0), (0.25, 0.5), (12.0, 6.0),
                                       (0.0, 0.05), (500.85, 0.04)])
def test_a_column_has_the_decimals_to_tell_its_nodes_apart(low, step):
    """A column of the table is as precise as the grid behind it, and no less.

    Half a step off the decimal lattice is where this bites. An axis of step 1
    starting at 0.5 is 0.5, 1.5, 2.5, and to no decimals at all that is 0, 2,
    2 - two of its nodes printed as one number. `_decimals` carries a spare
    digit for exactly that, and this is the check that it is not spare.
    """
    from ascent.summary import _decimals

    nodes = Range(low, low + 4 * step, 5).values()
    printed = [f'{value:.{_decimals(step)}f}' for value in nodes]
    assert len(set(printed)) == len(printed), printed


# --- what a search finds --------------------------------------------------


def test_the_shape_of_the_turn_carries_the_two_conditions(found):
    """Because the cut-off, asked for in tenths of a second, cannot.

    A tenth of a second of burn is some eight kilometres of apogee - sixteen
    times the tolerance here - so an instant a vehicle could actually be given
    is far too coarse a thing to place an orbit with. What places it is the
    shape of the turn, k2 and k3, which are coefficients of the guidance law
    and are not rounded to anything. The cut-off gets the ascent to the right
    tenth of a second and stops there, near the 502.71 s the catalogue solved
    for when it was free to place the cut-off to any precision it liked.
    """
    assert found.reaches_orbit, f'missed by {found.best.miss:.0f} m'
    assert found.best.miss < 500.0
    assert found.best.cutoff_time == pytest.approx(502.7, abs=0.5)


# The catalogue's own sets for 500 km in the coordinates the grid uses, each
# with a narrow range about it and the cut-off it should come back with. The
# velocity share's turn ends at 0.9732 of the cut-off; the bilinear tangent
# starts at 87.934 degrees and has reached 29.506 half way along its turn.
#
# The bilinear tangent's cut-off is split to a fortieth of a second where the
# velocity share gets three quarters of one, and that is the family rather than
# the test. It reaches the horizon linearly, so how far the cut-off falls
# past the end of its turn is the eccentricity of the orbit: the floor of the
# valley it is being searched down is some hundredths of a second wide, and a
# grid whose step steps over it finds the wall instead.
# The other two families, each narrowed on to the catalogue's set for 500 km,
# with the cut-off it should come back with and the passes it takes. Two shape
# parameters are searched in each and neither is held, for the reason `NARROW`
# gives above: with the cut-off in tenths of a second, the shape is what has to
# carry both terminal conditions.
#
# The bilinear tangent gets the narrower ranges and the more values in them,
# and that is the family rather than the test. It reaches the horizon linearly,
# so how far the cut-off falls past the end of its turn is the eccentricity of
# the orbit, and the floor of the valley it is searched down is thin.
NARROW_BY_FAMILY = {
    'velocity-share': ({'t1': Range(20.0, 20.0), 'turn': Range(0.96, 0.99, 3),
                        's': Range(1.0, 1.3, 3),
                        'te': Range(501.5, 503.0, 3),
                        'coast': Range(0.0, 0.0)}, 502.1, 6),
    'bilinear-tangent': ({'t1': Range(20.0, 20.0), 'start': Range(87.5, 88.5, 5),
                          'mid': Range(0.5, 0.5), 'middle': Range(29.0, 30.0, 5),
                          'te': Range(500.8, 501.0, 3),
                          'angle': Range(0.0, 0.0),
                          'coast': Range(0.0, 0.0)}, 500.9, 5),
}


@pytest.mark.parametrize('programme', sorted(NARROW_BY_FAMILY))
def test_a_narrow_grid_reaches_the_orbit_in_every_family(programme):
    """Each of the other two families, narrowed on to a set that reaches.

    The five-phase family is tested above and in more detail; this is that the
    other two are searched, closed in on and reported the same way, and that
    each comes back on one particular tenth of a second.

    The tenth is pinned rather than derived, and it is the answer the search
    gives rather than a figure from anywhere else - so it moves when the search
    changes, and it did: the velocity share landed on 502.2 while the passes
    followed one valley and on 502.1 once they followed five. Neither is wrong
    and the second is not better, at 67 m against 62; a pass that closes in on
    several places lays its grids differently, so a valley that was already the
    right one can come back a few metres coarser. What that buys is the
    families and vehicles where one valley lands nowhere near.
    """
    ranges, cutoff, refinements = NARROW_BY_FAMILY[programme]
    result = search(FALCON, 500_000, programme, ranges=ranges,
                    refinements=refinements, steps_per_second=1, workers=1,
                    tolerance=1000.0, **SITE)

    assert result.reaches_orbit, f'missed by {result.best.miss:.0f} m'
    assert result.best.cutoff_time == cutoff
    assert result.best.orbit.eccentricity < 1e-3


def test_the_table_is_ranked_by_how_far_the_orbit_is_from_the_circle(found):
    errors = [candidate.orbit_error for candidate in found.found]
    assert errors == sorted(errors)
    assert found.best is found.found[0]
    assert found.reaching[0] is found.best


def test_a_node_already_walked_is_not_walked_again(found):
    """Every pass that closes in shares nodes with the pass before it.

    It is centred on one of that pass's nodes and reaches a whole width either
    side of it, so between an eighth and a fifth of it has been walked already.
    Those are skipped rather than flown a second time, which is why every set
    that closed an orbit is one row of the table and not several.
    """
    keys = [candidate.key for candidate in found.found]
    assert len(keys) == len(set(keys))
    assert found.closed == len(found.found)
    assert found.revisited > 0, 'the passes never overlapped'


def test_every_node_ends_in_exactly_one_count(found):
    """The five outcomes of a node are counted, not derived from each other."""
    assert found.nodes == (found.screened + found.refused + found.failed
                           + found.no_orbit + found.closed)
    assert found.closed == found.solved
    assert found.flown == found.nodes - found.screened - found.refused


def test_the_terminal_state_is_read_where_the_orbit_is(found):
    """The altitude and the speed reported belong to the orbit reported.

    Read off the last row of the flight, which is the row the orbit was built
    from, rather than off the last row before the cut-off: a flight is a whole
    number of steps and a cut-off is not, and the vehicle is under some 30 m/s^2
    right up to it. At one step a second that row is worth tens of metres per
    second the set was never going to have.
    """
    best = found.best
    mean = 0.5 * (best.orbit.perigee_altitude + best.orbit.apogee_altitude)
    assert best.altitude == pytest.approx(mean, abs=200.0)
    assert best.speed_miss < 2.0


def test_the_speed_is_judged_against_the_orbit_asked_for(found):
    """Not against a circle through wherever the vehicle happened to be.

    A set that levels off twenty kilometres low and goes exactly the speed a
    circle at that altitude wants is on a perfect orbit, and not the one asked
    for. Measured against the target it shows that miss; measured against its
    own altitude it would show nothing at all - which is why the summary says
    the speed is against the orbit asked for and this says the same in figures.
    """
    target = circular_velocity(500_000)
    for candidate in found.found:
        assert candidate.speed_miss == pytest.approx(abs(candidate.speed - target))

    # and the two readings are not the same reading, so the choice is a choice:
    # the grid reaches well under the target as well as over it
    lowest = min(found.found, key=lambda candidate: candidate.altitude)
    assert lowest.altitude < 490_000
    against_its_own = abs(lowest.speed - circular_velocity(lowest.altitude))
    assert abs(lowest.speed_miss - against_its_own) > 5.0


def test_a_set_is_judged_by_all_three_errors(found):
    """The orbit, the altitude at cut-off and the speed there, each in turn."""
    best = found.best
    assert best.reaches(1000.0, 10.0)
    assert not dataclasses.replace(best, miss=2_000.0).reaches(1000.0, 10.0)
    assert not dataclasses.replace(best, altitude_miss=2_000.0).reaches(1000.0, 10.0)
    assert not dataclasses.replace(best, speed_miss=50.0).reaches(1000.0, 10.0)


def test_the_set_found_flies_again_from_its_own_specification(found):
    """What the search writes down reproduces what the search measured."""
    spec = found.specification('lv.f9')
    assert spec['vehicle'] == 'lv.f9'
    assert spec['cutoff']['time'] == found.best.cutoff_time
    assert spec['pitch_programme']['k3'] == found.best.values['k3']

    mission = mission_from_spec(spec, 'config')
    mission.run()
    assert mission.orbit.perigee_altitude == pytest.approx(
        found.best.orbit.perigee_altitude, abs=100)
    assert mission.orbit.apogee_altitude == pytest.approx(
        found.best.orbit.apogee_altitude, abs=100)


def test_a_set_that_misses_is_not_written_out_as_an_entry(found):
    """`best` is the closest set found; only a set that reaches is filed.

    The same search read against a tolerance it does not meet, rather than a
    second search run to miss one: what is under test is the rule, and the rule
    is read off the result.
    """
    strict = copy.copy(found)
    strict.tolerance = 1.0

    assert not strict.reaches_orbit
    assert strict.best is not None
    with pytest.raises(ValueError, match='not a catalogue entry'):
        strict.specification('lv.f9')


def test_a_tolerance_is_written_as_it_was_asked_for(found):
    """Not rounded, because rounding a term changes what the entry claims.

    Every other figure on an entry is a measurement, and rounding one of those
    costs a little precision and nothing else. These two are the terms the set
    was accepted under: a search asked for four tenths of a metre would be filed
    as having been asked for nothing at all, and the entry would then say
    something that is not merely imprecise but false.
    """
    awkward = copy.copy(found)
    # loose enough that the set still reaches, and written to more places than
    # any rounding would keep
    awkward.tolerance, awkward.speed_tolerance = 1234.5678, 12.3456789

    tolerance = awkward.specification('lv.f9')['tolerance']
    assert tolerance == {'orbit_km': 1.2345678, 'speed_ms': 12.3456789}


def test_an_entry_records_what_it_was_searched_at(found):
    """The step it was flown at, and the tolerances it had to meet.

    A set is found against a model and the step is part of which model - the
    orbit moves by a metre or two between five steps a second and ten, and the
    steering loss by up to a metre per second - so an entry that named a step it
    was not searched at would not reproduce the figures written beside it. The
    searches here run at one step a second, which is what makes this worth
    checking: a hard ten would go unnoticed against a default of ten.
    """
    entry = found.specification('lv.f9')

    assert entry['simulation']['steps_per_second'] == found.steps_per_second == 1
    assert entry['tolerance'] == {'orbit_km': found.tolerance / 1000,
                                  'speed_ms': found.speed_tolerance}


def test_the_screen_drops_nodes_without_flying_them():
    """The dissertation's altitude integral, used as a gate.

    A cut-off early enough that the programme cannot reach the target is
    dropped without a trajectory, and `screen=False` is how that is checked:
    the same grid flown whole finds everything the screened one did.
    """
    wide = {**NARROW, 't4': Range(470.0, 524.0, 10)}
    screened = quick(ranges=wide, refinements=0)
    whole = quick(ranges=wide, refinements=0, screen=False)

    assert screened.screened > 0, 'the altitude integral rejected nothing'
    assert whole.screened == 0
    assert whole.flown > screened.flown
    assert screened.best.values == whole.best.values


def test_a_set_that_overstresses_the_airframe_is_not_an_answer():
    """`max_dynamic_pressure` takes a set out of the ranking, however close.

    Falcon 9 to 500 km peaks around 35 kPa on every set that reaches it, so a
    limit of 25 leaves the search nothing to return - and it says so rather
    than returning the closest set that broke the vehicle.
    """
    constrained = quick(refinements=0, max_dynamic_pressure=25_000.0)

    assert constrained.over_pressure > 0
    assert constrained.found == []
    assert constrained.best is None
    assert not constrained.reaches_orbit


def test_a_search_can_be_interrupted_part_way_through_a_pass():
    """And the pool lets go of the rest of the pass rather than walking it.

    The interrupt is raised where one really arrives - in the process that was
    asked, between one node and the next - and what is under test is that it
    comes back out as itself. That it comes back promptly is under test too,
    though not as an assertion: the queue holds most of a pass, and a search
    that waited for all of it would hang this suite rather than fail it.
    """
    def interrupt(result):
        if result.pass_node >= 3:
            raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        quick(refinements=0, workers=2, report=interrupt)


def test_a_worker_does_not_answer_the_interrupt():
    """One press of Ctrl+C, one stack at most - and this is why there is none.

    An interrupt reaches every process of a console at once, so a worker that
    kept the default handler would raise out of the middle of a trajectory and
    print its own stack, a dozen of them for one press of two keys. Checked in
    a process of its own, because what it checks is a process being made deaf
    to Ctrl+C and this suite would like to keep hearing it.
    """
    asked = subprocess.run(
        [sys.executable, '-c', 'import signal; from ascent.search import _begin;'
                               '_begin(None);'
                               'print(signal.getsignal(signal.SIGINT) is signal.SIG_IGN)'],
        capture_output=True, text=True)
    assert asked.stdout.strip() == 'True', asked.stderr


def test_the_two_halves_of_the_summary_are_the_whole_of_it(found):
    """`ascent-search` prints them apart; a script asking for both gets both."""
    from ascent.summary import summarise_found, summarise_plan, summarise_search

    whole = summarise_search(found)
    assert whole.startswith(summarise_plan(found, planned=False))
    assert whole.endswith(summarise_found(found))
    # and the figure about a search that has not run yet is not in either
    assert 'nodes planned' not in whole


def test_dividing_the_grid_over_processes_finds_the_same_set():
    """A pool answers the nodes of a pass; it does not change what they answer.

    The nodes of a pass are independent and are collected in the order of the
    grid, so a search returns the same set however many processes it was
    divided over - and the same count of nodes and of trajectories, which is
    what says the division was of the work and not of the answer.
    """
    alone = quick(refinements=1, workers=1)
    together = quick(refinements=1, workers=2)

    assert (alone.workers, together.workers) == (1, 2)
    assert together.best.values == alone.best.values
    assert together.best.cutoff_time == alone.best.cutoff_time
    assert (together.nodes, together.flown) == (alone.nodes, alone.flown)
