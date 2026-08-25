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

import pytest

from ascent.config import (PITCH_PROGRAMMES, load_catalogue, load_vehicle,
                           mission_from_spec)
from ascent.estimates import equivalent_time
from ascent.search import (FAMILIES, NODE_LIMIT, REFINED_NODES,
                           BilinearTangent, FivePhase, Range, VelocityShare,
                           _closer, axis_names, parse_ranges, plan, search)

FALCON = load_vehicle('config/lv.f9.yaml')
CATALOGUE = load_catalogue('config/catalogue.yaml')
SITE = {'latitude_deg': 28.5, 'azimuth_deg': 90.0}

# The catalogue's own five-phase set for 500 km with the grid narrowed on to it:
# the vertical rise and k2 held where the catalogue holds them, k3 and the end
# of the turn swept over four nodes each. Some 130 trajectories, three seconds.
NARROW = {'t1': Range(20.0, 20.0), 'k2': Range(0.05, 0.05),
          'k3': Range(0.50, 0.56, 0.02), 't4': Range(502.0, 503.5, 0.5),
          'angle': Range(0.0, 0.0), 'coast': Range(0.0, 0.0)}


def quick(**overrides):
    """A search narrow enough for a test and complete in every step of itself."""
    settings = {'ranges': NARROW, 'refinements': 7, 'steps_per_second': 1,
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


def test_the_families_cover_the_catalogue():
    """Every programme the catalogue holds is one the search can look for."""
    assert set(FAMILIES) == {spec['pitch_programme']['type'] for spec in CATALOGUE}


def test_a_family_that_is_not_one_is_refused():
    with pytest.raises(ValueError, match='unknown pitch programme'):
        axis_names('gravity-turn')


# --- ranges ---------------------------------------------------------------


def test_a_range_walks_from_the_low_end_in_steps():
    span = Range(10.0, 30.0, 5.0)
    assert span.nodes == 5
    assert span.values() == (10.0, 15.0, 20.0, 25.0, 30.0)
    assert span.last == 30.0


def test_a_range_stops_at_the_last_whole_step_and_says_so():
    """The top of a range is a ceiling, not necessarily a node."""
    span = Range(10.0, 30.0, 7.0)
    assert span.values() == (10.0, 17.0, 24.0)
    assert span.last == 24.0
    assert '10 to 24 step 7 (3 nodes)' == span.describe()


def test_a_span_that_divides_by_its_step_keeps_its_top_node():
    """0 to 0.9 in steps of 0.05 is nineteen nodes, not eighteen.

    The arithmetic says otherwise - a twentieth is not a binary fraction - so
    the count carries a tolerance, and this is what it is for.
    """
    assert Range(0.0, 0.9, 0.05).nodes == 19
    assert Range(0.0, 0.9, 0.05).values()[-1] == pytest.approx(0.9)


def test_a_parameter_is_held_by_a_range_of_one_node():
    span = Range(0.05, 0.05)
    assert span.nodes == 1
    assert span.values() == (0.05,)
    assert span.describe() == '0.05, held'


@pytest.mark.parametrize('text, expected', [
    ('t1=10:30:2', ('t1', Range(10.0, 30.0, 2.0))),
    ('k2=0.05', ('k2', Range(0.05, 0.05, 0.0))),
    ('s=-3:3:0.5', ('s', Range(-3.0, 3.0, 0.5))),
    (' t1 = 12:30:6 ', ('t1', Range(12.0, 30.0, 6.0))),
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
    ('t1=10:30:0', 'needs a step'),
    ('t1=30:10:2', 'runs from low to high'),
    ('t1=10:30:-2', 'steps forwards'),
])
def test_a_range_that_is_not_one_says_what_one_looks_like(text, complaint):
    with pytest.raises(ValueError, match=complaint):
        Range.parse(text)


def test_a_parameter_given_two_ranges_is_refused():
    """The second would silently replace the first, which is not an answer."""
    with pytest.raises(ValueError, match='given a range twice'):
        parse_ranges(['t1=10:30:2', 't1=12:20:4'])


# --- the grid, before it is walked ----------------------------------------


def outline(**overrides):
    settings = {'ranges': NARROW, 'refinements': 7, **SITE}
    settings.update(overrides)
    return plan(FALCON, 500_000, 'five-phase', **settings)


def test_the_grid_takes_the_ranges_it_is_given():
    grid = outline(ranges={'t1': Range(10.0, 30.0, 2.0)}).ranges
    assert grid['t1'] == Range(10.0, 30.0, 2.0)
    # and every other axis keeps the range the family gave it
    assert grid['k3'] == FivePhase().ranges((0.0, 1.0))['k3']


def test_a_parameter_the_family_does_not_have_is_refused():
    with pytest.raises(ValueError, match='not a parameter of the five-phase'):
        outline(ranges={'kick': Range(1.0, 2.0, 0.5)})


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
    """The sweep, and then five nodes an axis for every pass that closes in."""
    result = outline(refinements=3)
    assert result.passes == 4
    assert result.planned_nodes == 4 * 4 + 3 * REFINED_NODES ** 2


def test_a_coarser_grid_is_a_thinner_one():
    """`coarseness` lengthens the stride of the axes the family gave.

    An axis the caller wrote out is left alone: that step was asked for.
    """
    given = {'k3': Range(0.0, 0.9, 0.05)}
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
    enormous = {'k3': Range(0.0, 0.9, 0.9 / 6_000_000)}
    assert outline(ranges=enormous).ranges['k3'].nodes > NODE_LIMIT

    with pytest.raises(ValueError, match='past the'):
        quick(ranges=enormous)


# --- closing in -----------------------------------------------------------


def test_closing_in_halves_the_step_and_keeps_the_centre():
    grid = {'k3': Range(0.0, 1.0, 0.25)}
    closer = _closer(grid, {'k3': 0.5}, {'k3': 0.25}, {'k3': (0.0, 1.0)})

    assert (closer['k3'].low, closer['k3'].high) == (0.25, 0.75)
    assert closer['k3'].step == pytest.approx(0.125)
    # the centre is a node of the closer grid, so a pass can never do worse
    # than the pass before it
    assert 0.5 in closer['k3'].values()


def test_closing_in_stays_inside_the_range_it_was_given():
    grid = {'s': Range(-3.0, 3.0, 1.5)}
    closer = _closer(grid, {'s': 3.0}, {'s': 1.5}, {'s': (-3.0, 3.0)})
    assert (closer['s'].low, closer['s'].high) == (1.5, 3.0)


def test_a_held_parameter_is_not_closed_in_on():
    """A range of one node is what holding a parameter is, and it stays one."""
    grid = {'k2': Range(0.05, 0.05)}
    assert _closer(grid, {'k2': 0.05}, {}, {'k2': (0.05, 0.05)}) == grid


# --- what a search finds --------------------------------------------------


def test_the_search_recovers_the_set_on_file(found):
    """The catalogue's five-phase set for 500 km, found again from the orbit.

    With the vertical rise and k2 held where the catalogue holds them, the
    family has k3 and the cut-off left for two terminal conditions, so there is
    nothing to prefer and the search has to come back with the set on file.
    """
    assert found.reaches_orbit, f'missed by {found.best.miss:.0f} m'
    assert found.best.cutoff_time == pytest.approx(502.712, abs=0.05)
    assert found.best.values['k3'] == pytest.approx(0.52958, abs=0.002)
    assert found.best.miss < 200.0


# The catalogue's own sets for 500 km in the coordinates the grid uses, each
# with a narrow range about it and the cut-off it should come back with. The
# velocity share's turn ends at 0.9732 of the cut-off; the bilinear tangent
# starts at 87.934 degrees and has reached 29.506 half way along its turn.
#
# The step on the bilinear tangent's cut-off is a fortieth of a second where
# the velocity share gets three quarters of one, and that is the family rather
# than the test. It reaches the horizon linearly, so how far the cut-off falls
# past the end of its turn is the eccentricity of the orbit: the floor of the
# valley it is being searched down is some hundredths of a second wide, and a
# grid whose step steps over it finds the wall instead.
NARROW_BY_FAMILY = {
    'velocity-share': ({'t1': Range(20.0, 20.0), 'turn': Range(0.96, 0.99, 0.015),
                        's': Range(1.1194, 1.1194),
                        'te': Range(501.5, 503.0, 0.75),
                        'coast': Range(0.0, 0.0)}, 502.193),
    'bilinear-tangent': ({'t1': Range(20.0, 20.0), 'start': Range(87.934, 87.934),
                          'mid': Range(0.5, 0.5), 'middle': Range(29.3, 29.7, 0.2),
                          'te': Range(500.85, 500.97, 0.04),
                          'angle': Range(0.0, 0.0),
                          'coast': Range(0.0, 0.0)}, 500.910),
}


@pytest.mark.parametrize('programme', sorted(NARROW_BY_FAMILY))
def test_a_narrow_grid_reaches_the_orbit_in_every_family(programme):
    """Each of the other two families, narrowed on to the set the catalogue has.

    The five-phase family is tested above and in more detail; this is that the
    other two are searched, closed in on and reported the same way, and that
    each comes back with the cut-off on file.
    """
    ranges, cutoff = NARROW_BY_FAMILY[programme]
    result = search(FALCON, 500_000, programme, ranges=ranges, refinements=5,
                    steps_per_second=1, workers=1, tolerance=1000.0, **SITE)

    assert result.reaches_orbit, f'missed by {result.best.miss:.0f} m'
    assert result.best.cutoff_time == pytest.approx(cutoff, abs=0.1)
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


def test_the_screen_drops_nodes_without_flying_them():
    """The dissertation's altitude integral, used as a gate.

    A cut-off early enough that the programme cannot reach the target is
    dropped without a trajectory, and `screen=False` is how that is checked:
    the same grid flown whole finds everything the screened one did.
    """
    wide = {**NARROW, 't4': Range(470.0, 524.0, 6.0)}
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
