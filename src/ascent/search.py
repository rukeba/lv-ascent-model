"""Grid search for the parameters of a pitch programme.

Ask for a vehicle, a circular orbit and one of the three programme families,
and this sweeps a grid over the parameters of that family and reports the sets
that come closest to the orbit, best first.

**Every parameter of the turn is an axis.** Nothing is held behind the
caller's back: the vertical rise, the shape of the turn, the instant the
programme ends and the instant the engines do are all coordinates of the same
grid, and each can be given its own range and its own number of values. What is
held is held because a range said so - a range of one - and the summary prints
every axis with the range it was searched over, so a figure that did not move
is a figure the caller can see was not asked to.

The grid is a map before it is an answer. A pass over it returns every set that
closed an orbit, ranked, and the best of them are printed as a table with the
errors each is judged by; the answer is the head of that table. That is what a
coarse grid is for - it says where in the family the orbit lies - and a second
search narrowed on to what it found is how the set itself is reached.

**What a set is judged by.** Three errors, and they are the three conditions of
a circular orbit at a given altitude:

  - the altitude at cut-off, against the target;
  - the speed at cut-off, against the speed of the circular orbit that was
    asked for - not against a circle through wherever the vehicle happened to
    be, which a set that levelled off twenty kilometres low would satisfy
    exactly while missing the orbit entirely. The inertial speed, because that
    is what the orbit is built from;
  - the orbit itself: how far the apogee and the perigee each ended up from the
    circle asked for. Their sum is the ranking. It is zero only when apogee =
    perigee = target, which is the altitude and the circularity at once, in the
    same relative unit and with no weighting to choose - and the eccentricity,
    which is the spread of the two, is reported beside it.

A set counts as reaching the orbit when the first two are inside the
tolerances given and the apsides are both inside the tolerance on the orbit.
Ranked by the third either way, so a search that reaches nothing still says
what came closest rather than saying nothing at all.

**Two estimates keep the cost down**, and both are quadrature rather than
integration - see `estimates.py`:

  - the ascent-time estimate says where along the time axis the cut-off can
    fall, and that window is the default range of the axis the cut-off is
    searched over. It also says, before anything is flown, whether the vehicle
    has the propellant for the orbit at all;
  - the altitude integral screens every node. It says what altitude the set
    would reach, and a set that cannot reach the target - inside the band the
    integral is known to read high by - is dropped without a trajectory.

**Then the grid closes in.** The best value found becomes the centre of the
next grid with one neighbour either side of it, five values across that, which
is the same two steps at half the spacing; and again, ten times over. A step of
the sweep is worth a hundred kilometres of apogee on the cut-off axis, so the
sweep says where in the family the orbit lies and the passes are what land on
it. They can travel about two sweep steps and no further, which is why the
count on an axis decides which part of the family is searched rather than how
finely - see `CUT_OFF_NODES`.

**And it closes in on more than one place at once.** A pass is a local descent
and answers only the valley it started in, which is enough where the ranking
has one. The cut-off axis is where it does not: a tenth of a second of burn is
kilometres of apogee, so the ranking along it is a row of narrow valleys, each
a cut-off the shape of the turn can very nearly make an orbit of, and a sweep
steps across several of them between two of its own nodes. Following only the
best is how a search lands on the bottom of the wrong one - a set whose apogee
is exact to ten metres and whose perigee is kilometres out. So the passes
follow the best few valleys together, one grid each, walked as a single pass -
see `BASINS` and `_centres`.

What separates one valley from the next is not the same on every axis, and the
difference is the point. On a coefficient of the guidance law it is a step of
the sweep, which is the scale a valley was missed at. On an axis of instants it
is the tenth of a second the answer is written in: such an axis stops being
closed in on once its window is under a tenth, and from then it does not move,
so a cut-off that locked on to the wrong tenth is a search that converges on to
a set a kilometre or two out and cannot do better. Read at the sweep step, every
tenth within five seconds of the best would be one valley and only one of them
would ever be tried.

**The step it integrates at rises as it goes.** A sweep is not measuring
anything - its job is to say which cell of the family the orbit lies in, and its
own step on the cut-off axis is worth tens of kilometres of apogee - so it runs
at one step a second, where a trajectory is known to within a hundred metres or
so and costs a tenth of what it would at ten. The pass after it runs at two, and
every pass from the third on at the step the caller asked for, which is where
the answer is resolved to metres and a coarse step would be noise instead of an
answer rather than noise around one. See `COARSE_STEPS`.

Two things follow, and both are what they look like. A node walked coarsely is
walked again when the step rises: the same set at one step a second and at ten
is two answers, and the second is the one worth having, so a search that ramps
flies more trajectories than one that does not and still takes less time. And
the answer is chosen only from sets flown at the step asked for - see `measured`
- because a coarse set can outrank a fine one on the difference between the two
rules rather than on the difference between the two sets. Everything found is
kept, at whatever step: that is what a table and a CSV are for.

**Instants are asked for in tenths of a second.** The vertical rise, the end of
the programme and the cut-off are rounded there, because that is the finest a
timeline is ever issued to, and two values that come to the same tenth are one
node of the grid rather than two answers differing where nothing can act on the
difference - see `TIME_DECIMALS`. A tenth of a second of burn is some eight
kilometres of apogee, so the cut-off cannot place the orbit and the shape of
the turn does: the coefficients of the guidance law are not instants, nothing
rounds them, and every family keeps exactly two, which is what the two terminal
conditions of a circular orbit need.

The nodes of a pass do not depend on one another, so they are answered over a
pool of processes, two thirds of the cores by default, and collected in the
order of the grid. A search returns the same table however many processes
answered it.
"""

import math
import os
import signal
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from itertools import product

import numpy as np

from .constants import EARTH_RADIUS, circular_velocity
from .cutoff import CutoffAtTime
from .estimates import (analytic_altitude, burns, equivalent_time,
                        required_velocity, vacuum_time)
from .losses import velocity_budget
from .mission import Mission, rotation_in_plane
from .orbit import Orbit
from .pitch import (BilinearTangentProgramme, FivePhaseProgramme, PitchProgramme,
                    VelocityShareProgramme, bilinear_coefficients)
from .telemetry import Telemetry
from .vehicle import DRAG_CEILING, LaunchVehicle

# How far either side of the estimated ascent time the cut-off axis reaches.
# The estimate leaves the rotation of the Earth out and prices the losses off a
# turn that does not depend on the orbit, so it mostly reads low; against the
# catalogue the cut-off on file sits between 0.949 and 1.089 of it, and these
# carry that band. The late end has room to spare. The early end has 0.9 per
# cent, which is thinner than it looks: it is one entry - Ariane 62's bilinear
# tangent at 400 km - that put it there, and a vehicle sitting lower still would
# be searched over a window that did not contain its answer. Widening it is not
# free, since the window is what the cut-off axis spreads its values over, and
# it would move every set on file; see tests/test_estimates.py.
TIME_MARGIN_EARLY = 0.06
TIME_MARGIN_LATE = 0.15

# What the altitude integral reads high by. It leaves out the air, the thrust
# deficit at sea level and the fall of gravity with altitude, all of which push
# the same way; against the catalogue the figure it returns is between 1.005
# and 1.185 times the altitude the flight reaches, and a node is screened out
# only when the target lies outside what these two allow.
#
# They are wider than that measurement because the measurement is of three
# vehicles and the screen is a gate: a node it rejects is never flown, so a
# vehicle whose integral read a little further out than any of these would be
# reported as unable to reach an orbit it can reach.
ALTITUDE_RATIO_LOW = 0.95
ALTITUDE_RATIO_HIGH = 1.40

# Both the perigee and the apogee have to land within this of the target for a
# set to count as reaching the orbit, m - and so does the altitude at cut-off
TOLERANCE = 500.0

# And the inertial speed at cut-off has to land within this of the speed of the
# circular orbit asked for, m/s. Loose beside the tolerance on the orbit
# because it is the weaker of the two conditions: an orbit whose apsides are
# both within half a kilometre of the target is already within a metre or two
# per second of the right speed, and this is here to catch the set that has the
# altitude and is not going fast enough to stay there
SPEED_TOLERANCE = 10.0

# The tenth of a second every instant and every duration of the flight is
# asked for and reported in.
#
# No vehicle is commanded to shut its engines down at 502.6720 s, and none
# begins its turn at 14.2676. A timeline is issued to a tenth of a second at
# best, so an answer written any finer is not a better answer - it is the same
# answer with digits after it that nothing can act on, and two of them that
# differ only there are one answer written twice.
#
# It is what the model already works in: a pitch programme is tabulated on a
# tenth-of-a-second grid, so an instant between two of its points is read back
# by interpolation anyway.
#
# What this leaves the search to steer with is the shape of the turn. Those are
# coefficients of a guidance law rather than instants on a timeline - k2 and k3,
# the fullness of a quartic, the angles a tangent passes through - and nothing
# rounds them. Every family keeps exactly two of them, which is what the two
# terminal conditions of a circular orbit need.
TIME_DECIMALS = 1
TIME_QUANTUM = 10.0 ** -TIME_DECIMALS

# How many of the sets found are printed, best first
TOP = 15

# Passes after the first, each one grid step wide about the best node found so
# far and along every axis that was searched. Ten because of what one step of
# the sweep is worth: the cut-off axis spans the whole window the ascent-time
# estimate allows, some fifty seconds, and near a circular orbit the apogee
# answers to the cut-off at around eighty kilometres a second. Halving that
# step ten times is what turns a sweep that says where the orbit is into a set
# that lands on it, and each pass is five nodes an axis rather than the whole
# grid again.
REFINEMENTS = 10

# Nodes along each axis of a refining pass. Five nodes span two old steps and
# so halve the step, at the cost of five flights an axis; a wider grid closes
# in faster per pass and pays for it as the power of its width.
REFINED_NODES = 5

# How many places on the grid the passes close in on at once.
#
# A pass that closes in is a local descent, and a local descent answers only
# the valley it started in. That is the whole of what one of them can do, and
# for most of this it is enough - the ranking near a circular orbit is one
# broad valley and the head of the sweep is in it.
#
# It is not always one valley. The cut-off axis is the case that matters: a
# tenth of a second of burn is some five kilometres of apogee on a vehicle that
# burns for a thousand seconds, so the ranking along it is a row of narrow
# valleys rather than one, each a cut-off that the shape of the turn can very
# nearly - but not quite - make an orbit of. A sweep steps across several of
# them between two of its own nodes, and which one its best node lands in is
# not the same question as which one holds the orbit. Following only that one
# is how a search comes back with a set whose apogee is exact to ten metres and
# whose perigee is kilometres out: the bottom of the wrong valley.
#
# So the passes follow several. The best node is one of them; the rest are the
# best nodes that are not in the valley of any already taken, which is what
# `_centres` decides and it decides it by the step of the sweep - two sets in
# the same cell of the sweep are the same valley, and the pass that closes in
# on one closes in on both.
#
# Five is what it costs against what it buys. A pass is five nodes an axis
# whatever the grid was, so five valleys are five of those, against a sweep that
# is the whole grid and is where most of the time of a search goes.
BASINS = 5

# The integration steps a search walks up through, coarsest first, before it
# settles on the one it was asked for.
#
# A sweep is not measuring anything. Its job is to say which cell of the family
# the orbit lies in, and its own step on the cut-off axis is worth tens of
# kilometres of apogee - so a trajectory integrated to within 130 m, which is
# what one step a second comes to across this catalogue, tells it everything it
# needs. The passes that follow resolve metres, and there a coarse step is not
# noise around the answer but noise instead of it.
#
# So the sweep runs at one step a second and the pass after it at two, and every
# pass from the third on runs at the step the caller asked for. Measured against
# ten across the whole catalogue, one step a second moves an apsis by up to
# 130 m and two by 28, against a tolerance of 500; and the sweep is where most
# of the nodes of a search are, so this is most of a search made ten times
# cheaper for a figure it cannot use anyway.
#
# The ramp is laid from the end rather than the start, so that the last pass is
# always at the finest step whatever the count of them - a search with no
# refinements at all is one pass, and that pass is the answer.
COARSE_STEPS = (1.0, 2.0)


def step_schedule(passes: int, finest: float) -> tuple[float, ...]:
    """What each pass integrates at, in order. See `COARSE_STEPS`."""
    # never coarser than what was asked for: a caller who wants two steps a
    # second is not helped by a sweep at one and a pass at two
    ramp = [min(step, finest) for step in COARSE_STEPS]
    ramp = ramp[max(0, len(ramp) - (passes - 1)):]
    return tuple(ramp) + (finest,) * (passes - len(ramp))


def halvings(schedule: tuple[float, ...]) -> int:
    """How many times the passes of `schedule` halve what they reach.

    Not one per pass. A pass that steps up to a finer integration keeps the
    reach it had, because it is re-flying the ground the pass before it covered
    and the point of it is to see that ground plainly rather than to narrow on
    a coarse reading of it - so the two step-ups of a full ramp cost two
    halvings, and eleven passes resolve what nine would have.

    Which is why this is worked out rather than assumed: the spacing a search
    reports and the edge it decides a set sits on are both read off the reach it
    actually ended with, and a figure four times finer than the truth would be
    the search overstating what it resolved.
    """
    return sum(1 for at in range(len(schedule) - 1)
               if schedule[at + 1] == schedule[at])


# What one node of the grid can come to. Each is a field of `SearchResult`, and
# every node increments exactly one of them
OUTCOMES = ('screened', 'refused', 'failed', 'no_orbit', 'closed')

# What one pass may come to before a search refuses to start. A grid is a
# product of its axes, so a step mistyped by a factor of a hundred is a grid a
# hundred times larger, and the first thing a pass does is lay every one of its
# nodes out in memory. Set far above any grid worth walking - five million
# nodes is a day of integration before it is anything else - so that it catches
# a mistake and never a search. `--dry-run` says what a grid comes to without
# walking it, and is not held to this at all
NODE_LIMIT = 5_000_000


def default_workers() -> int:
    """Processes a search runs its nodes over: two thirds of the cores.

    Two thirds rather than all of them because a search is minutes long and the
    machine it runs on is being used for something else at the time. The nodes
    of one pass are independent, so they divide over processes exactly, and it
    has to be processes: the work is Python arithmetic, and threads would queue
    up behind the interpreter lock rather than run beside each other.
    """
    return max(1, ((os.cpu_count() or 1) * 2) // 3)


# --- the grid -------------------------------------------------------------


@dataclass(frozen=True)
class Range:
    """One axis of the grid: `nodes` values evenly spaced from `low` to `high`.

    A count rather than a step, because a count is what says what a pass will
    cost - the grid is the product of the counts of its axes - and because both
    ends of the range are then values the search actually tries. `nodes` of 1
    is a single value at `low`, which is how a parameter is held.

    The step follows from the three and is reported beside them, since it is
    the step and not the count that says how finely the answer is resolved.
    """
    low: float
    high: float
    nodes: int = 1

    def __post_init__(self) -> None:
        if not math.isfinite(self.low) or not math.isfinite(self.high):
            raise ValueError(f'a range runs between two numbers, and not '
                             f'between {self.low} and {self.high}')
        if self.nodes != int(self.nodes) or self.nodes < 1:
            raise ValueError(f'a range is walked in a whole number of values, '
                             f'at least one, and not {self.nodes}')
        # written as an int whatever it arrived as, so that two ranges of the
        # same grid compare equal however each of them was built
        object.__setattr__(self, 'nodes', int(self.nodes))

        if self.high < self.low:
            raise ValueError(f'a range runs from low to high, and not from '
                             f'{self.low:g} to {self.high:g}')
        if self.nodes == 1 and self.high != self.low:
            raise ValueError(
                f'one value between {self.low:g} and {self.high:g} does not say '
                f'which of them it is; write it as the value on its own')
        if self.nodes > 1 and self.high == self.low:
            raise ValueError(
                f'{self.nodes} values of {self.low:g} are one value; write it '
                f'as the value on its own')

    @property
    def step(self) -> float:
        """The distance between two neighbouring values, and 0 for one value."""
        return 0.0 if self.nodes == 1 else (self.high - self.low) / (self.nodes - 1)

    def values(self) -> tuple[float, ...]:
        if self.nodes == 1:
            return (float(self.low),)
        # built from the low end and a multiple rather than accumulated, so
        # that a value is where it says it is however far along it lies - and
        # the top end taken as itself, which the arithmetic would otherwise
        # miss by a bit in the last place
        step = self.step
        return tuple(self.low + index * step
                     for index in range(self.nodes - 1)) + (float(self.high),)

    def describe(self) -> str:
        if self.nodes == 1:
            return f'{self.low:g}, held'
        return (f'{self.low:g} to {self.high:g}, {self.nodes} values, '
                f'step {self.step:g}')

    # what a range looks like on the command line, quoted back at whoever gets
    # it wrong. One place, so that the several ways of getting it wrong are all
    # answered with the same thing
    SYNTAX = ('NAME=LOW:HIGH:VALUES for a parameter to search, as in '
              't1=10:25:10 - ten values from 10 to 25, one every 1.667 - or '
              'NAME=VALUE to hold one, as in k2=0.05')

    @staticmethod
    def parse(text: str) -> tuple[str, "Range"]:
        """One axis as it is written on the command line.

            t1=10:25:10     ten values from 10 to 25
            k2=0.05         held at 0.05

        The equals sign separates the parameter from its numbers and the colons
        separate the numbers from each other: where it stops, where it ends, and
        how many values to try between the two.

        The name comes back beside the range rather than being looked up here:
        what names a parameter is the family, and this does not know which
        family is being searched.
        """
        name, equals, rest = (part.strip() for part in text.partition('='))
        if not equals or not name or not rest:
            raise ValueError(f'{text!r} is not a range: write it as {Range.SYNTAX}')
        try:
            numbers = [float(part) for part in rest.split(':')]
        except ValueError:
            raise ValueError(
                f'a range is made of numbers and {rest!r} is not: write it as '
                f'{Range.SYNTAX}') from None

        if len(numbers) == 1:
            return name, Range(numbers[0], numbers[0], 1)
        if len(numbers) == 3:
            return name, Range(*numbers)
        raise ValueError(
            f'{text!r} gives {name} {len(numbers)} numbers, and a range takes '
            f'one or three: write it as {Range.SYNTAX}')


def parse_ranges(texts) -> dict[str, Range]:
    """Every `--range` given, as a table of axis against range."""
    ranges: dict[str, Range] = {}
    for text in texts:
        name, span = Range.parse(text)
        if name in ranges:
            raise ValueError(f'{name} was given a range twice; the second one '
                             f'would silently replace the first')
        ranges[name] = span
    return ranges


def _coarsen(ranges: dict[str, Range], factor: float) -> dict[str, Range]:
    """The same axes with the values along each scaled, for a quicker look.

    A factor below one thins the grid and lengthens the stride, which is the
    quicker and rougher sweep; above one fills it in. Two values is as thin as
    an axis gets: fewer would be an axis that had stopped covering the range it
    was given, and a parameter is held by being given one value rather than by
    being coarsened into one.
    """
    if factor == 1.0:
        return ranges
    return {name: (span if span.nodes == 1 else
                   Range(span.low, span.high,
                         max(2, round(span.nodes * factor))))
            for name, span in ranges.items()}


# --- what a node comes to -------------------------------------------------


@dataclass(frozen=True)
class Candidate:
    """One node of the grid: the set it stands for, and the orbit it reached."""
    # the grid coordinates, which is what a refining pass closes in on
    values: dict[str, float]
    # the whole set as a pitch-programme specification
    parameters: dict[str, float]
    cutoff_time: float
    orbit: Orbit
    # the state at cut-off: altitude (m), inertial speed (m/s), flight-path
    # angle (deg). These are what the first two errors are read off, and the
    # speed is the inertial one - the orbit is built from that and not from the
    # speed relative to a turning Earth
    altitude: float
    speed: float
    flight_path_angle: float
    # how far each of the three conditions was missed by, in its own unit
    altitude_miss: float
    speed_miss: float
    # the larger of the two apsidal errors, m: what the tolerance is read against
    miss: float
    # what this one was integrated at. Not the same for every candidate of a
    # search: the early passes run coarse and the later ones at the step the
    # caller asked for, so everything measured of a set - the orbit, the budget,
    # the errors below - is measured at this and has to be read with it
    steps_per_second: float = 10
    gravity_loss: float = 0.0
    aerodynamic_loss: float = 0.0
    steering_loss: float = 0.0
    # what the ascent asks of the airframe and of the guidance. Neither enters
    # the ranking unless the caller sets a limit on the first
    peak_dynamic_pressure: float = 0.0
    peak_steering_demand: float = 0.0
    # the three errors as relative figures, which is what the table prints and
    # what the ranking below is built from
    altitude_error: float = 0.0
    speed_error: float = 0.0
    apogee_error: float = 0.0
    perigee_error: float = 0.0

    @property
    def total_loss(self) -> float:
        return self.gravity_loss + self.aerodynamic_loss + self.steering_loss

    @property
    def orbit_error(self) -> float:
        """How far the orbit is from the circle asked for: the ranking.

        The apogee and the perigee each measured against the target radius, and
        added. Zero only when both apsides are the target, which is the
        altitude and the circularity at once - the mean of the apsides is the
        energy and the spread of them is the eccentricity - in one relative
        figure with no weighting to argue over.

        Not the altitude and the speed at cut-off, which are the same condition
        read at a single instant and are blind to the shape of the orbit: a set
        at the right altitude and the right speed but a degree off the horizon
        is on an ellipse, and neither of those two figures says so.
        """
        return self.apogee_error + self.perigee_error

    def reaches(self, tolerance: float, speed_tolerance: float) -> bool:
        """Whether this set meets all three conditions of the orbit."""
        return (self.miss <= tolerance and self.altitude_miss <= tolerance
                and self.speed_miss <= speed_tolerance)

    @property
    def key(self) -> tuple:
        """What makes two nodes the same set. See `_key`."""
        return _key(self.values)


@dataclass(frozen=True)
class Node:
    """What one node came to, and what it cost to find out.

    Returned rather than recorded, because a node may be answered in another
    process: the counting is done by whoever collects it.
    """
    values: dict[str, float]
    outcome: str
    candidate: Candidate | None
    flights: int


@dataclass
class SearchResult:
    """What the search found, and what it cost to find it."""
    best: Candidate | None
    # every distinct set that closed an orbit, ranked by `orbit_error`
    found: list[Candidate]
    vehicle: LaunchVehicle
    target_altitude: float
    programme: str
    latitude_deg: float
    azimuth_deg: float
    steps_per_second: float = 10
    # the axes as they were searched, after any `--range` and any coarsening
    ranges: dict[str, Range] = field(default_factory=dict)
    # the estimates the search was bounded by
    required_velocity: float = 0.0
    vacuum_time: float = 0.0
    equivalent_time: float = 0.0
    window: tuple[float, float] = (0.0, 0.0)
    # nodes of the grid visited, dropped by the altitude integral, refused by
    # the family itself, left unflyable by the vehicle, and flown on to
    # something that is not an orbit - an open trajectory, or a perigee under
    # the surface
    nodes: int = 0
    screened: int = 0
    refused: int = 0
    failed: int = 0
    no_orbit: int = 0
    # nodes that came out on an orbit. Counted rather than derived, so that a
    # node falling through every branch would show as an inconsistency
    closed: int = 0
    # sets that reached an orbit and were put aside for asking more of the
    # airframe than the caller allowed
    over_pressure: int = 0
    # nodes a pass would have walked that the search had walked already, and
    # skipped rather than flown a second time
    revisited: int = 0
    # trajectories integrated, which is what the search actually costs
    flown: int = 0
    # where the search has got to. The passes and the nodes of each are known
    # before it starts, so `nodes` against `planned_nodes` is a share of the
    # work done
    passes: int = 1
    pass_number: int = 0
    pass_nodes: int = 0
    pass_node: int = 0
    planned_nodes: int = 0
    tolerance: float = TOLERANCE
    speed_tolerance: float = SPEED_TOLERANCE
    top: int = TOP
    # places on the grid the passes close in on at once. What was asked for
    # until a pass has run, and thereafter the most any one of them actually
    # closed in on - which is fewer where the sets found were all in the same
    # few cells of the sweep, and there is no use reporting five valleys
    # followed when the ranking only ever offered one. See `BASINS`
    basins: int = BASINS
    max_dynamic_pressure: float | None = None
    # processes the nodes of a pass were divided over
    workers: int = 1
    # what each pass integrated at, in order - see `COARSE_STEPS`. Recorded
    # rather than worked out again from `passes`, which a search that stopped
    # early rewrites: laying a fresh ramp over the shortened count would report
    # a sweep that ran at one step a second as having run at ten
    schedule: tuple[float, ...] = ()
    # what each axis being searched is resolved to by the end, one entry per
    # axis of `searched`. The sweep step halved once per pass, except on an
    # axis of instants, which stops at the tenth of a second the flight is
    # asked in
    spacing: dict[str, float] = field(default_factory=dict)
    # axes whose best value came out on a bound of the grid, where a better set
    # may lie just outside
    on_edge: tuple[str, ...] = ()

    @property
    def solved(self) -> int:
        """Nodes for which a trajectory came out on an orbit."""
        return self.closed

    @property
    def reaches_orbit(self) -> bool:
        """Whether the search has an answer, at the step it was asked for.

        Both halves of that matter. `best` is the closest set found and falls
        back to a coarsely flown one where no pass at the finest step has closed
        anything - which is every pass of the ramp before the first fine one,
        and is also what is left if the fine passes re-fly what the sweep liked
        and find it is not an orbit after all, or that it asks more of the
        airframe than the caller allowed. Showing that set is right. Answering
        with it is not: it has been measured to within a hundred metres or so
        and the tolerances it would be judged by are tighter than that.

        `specification` is gated on this, so nothing coarse can be filed.
        """
        return self.best is not None \
            and self.best.steps_per_second == self.steps_per_second \
            and self.best.reaches(self.tolerance, self.speed_tolerance)

    @property
    def measured(self) -> list[Candidate]:
        """The sets flown at the step this search was asked for, best first.

        Not all of them are. The early passes integrate coarsely - see
        `COARSE_STEPS` - so the table holds sets measured to within a hundred
        metres or so beside sets measured to within three, and the two are not
        comparable as answers however well they compare as a map. A coarse set
        that looks the better of two by less than the coarse step is worth is
        not the better of them; it is the same set read with a wider rule.

        So an answer is chosen from these, and everything found is kept in
        `found` for what a table and a CSV are for - saying where in the family
        the orbit lies, which is exactly what a coarse pass is good at.
        """
        return [candidate for candidate in self.found
                if candidate.steps_per_second == self.steps_per_second]

    @property
    def reaching(self) -> list[Candidate]:
        """Every set that meets all three conditions, best first.

        Of those flown at the step asked for. A set that meets the tolerances
        at one step a second has not been shown to meet them at ten.
        """
        return [candidate for candidate in self.measured
                if candidate.reaches(self.tolerance, self.speed_tolerance)]

    @property
    def searched(self) -> dict[str, Range]:
        """The axes that were actually swept, as against those held."""
        return {name: span for name, span in self.ranges.items()
                if span.nodes > 1}

    def specification(self, vehicle_file: str,
                      duration_margin: float = 60.0) -> dict:
        """The set found as a mission specification, ready for the catalogue.

        `vehicle_file` is the stem of the vehicle file the entry should name -
        `lv.f9` rather than `Falcon 9`, which is what the vehicle calls itself.

        Refused unless the set reaches the orbit. `best` is the closest set
        found whether or not it reaches, which is the right thing to show and
        the wrong thing to file: an entry of the catalogue is a set that meets
        its terminal condition, and one that misses would be read as one that
        does not.

        The entry carries the tolerances it was accepted under, beside the orbit
        it reached. Half a kilometre is what almost all of them were asked for
        and it is not a property of the file: a family that no search closes on
        a vehicle can still be worth an entry asked for at two and a half, and
        the difference between the two is exactly the thing a reader has to be
        able to see. `reached` says what the set does and this says what it had
        to do, so an entry means one thing without anyone having to know what
        the search was run with.
        """
        if self.best is None or not self.reaches_orbit:
            raise ValueError(
                'the search found no set that reaches the orbit, and a set '
                'that misses it is not a catalogue entry. A family that will '
                'not close to half a kilometre may still close to more, and '
                'then the entry says which - see `tolerance`')
        best = self.best
        altitude = self.target_altitude
        return {
            'vehicle': vehicle_file,
            # written whole where it is whole, as every entry on file is
            'target_altitude': int(altitude) if altitude == int(altitude) else altitude,
            'launch_site': {'latitude': self.latitude_deg, 'azimuth': self.azimuth_deg},
            'pitch_programme': best.parameters,
            'cutoff': {'type': 'time', 'time': best.cutoff_time},
            # the step this set was actually flown at, and not ten because
            # ten is usual nor the finest of the search because the answer
            # might not have come from a pass that ran there. A set is found
            # against a model, and the step is part of which model: the orbit
            # moves by a metre or two between five steps a second and ten, and
            # the steering loss by up to a metre per second, so an entry filed
            # at a step it was not searched at would not reproduce the figures
            # written beside it
            'simulation': {'duration': round(best.cutoff_time + duration_margin),
                           'steps_per_second': best.steps_per_second},
            # what was asked of it, before what it did: the perigee, the apogee
            # and the altitude at cut-off all inside the first, the inertial
            # speed at cut-off inside the second.
            #
            # Written as they are and not rounded. Every other figure of an
            # entry is a measurement and rounding one costs a little precision;
            # these are the terms the entry was accepted under, and rounding a
            # term changes what the entry claims. A search asked for 0.4 m would
            # be filed as having been asked for nothing at all
            'tolerance': {'orbit_km': self.tolerance / 1000,
                          'speed_ms': self.speed_tolerance},
            'reached': {
                'perigee_km': round(best.orbit.perigee_altitude / 1000, 2),
                'apogee_km': round(best.orbit.apogee_altitude / 1000, 2),
                'gravity_loss': round(best.gravity_loss, 1),
                'aerodynamic_loss': round(best.aerodynamic_loss, 1),
                'steering_loss': round(best.steering_loss, 1),
                'total_loss': round(best.total_loss, 1),
            },
        }


# --- the three families, as a search sees them ----------------------------


# The vertical rise, in seconds. Every family has one and every family searches
# it over the same range: it is the first thing a vehicle does and it is not a
# property of the shape that follows.
RISE = Range(12.0, 30.0, 4)

# Values along the cut-off axis of a sweep, spread over the window the
# ascent-time estimate gives.
#
# This is the one count worth arguing about, and the argument is not about
# precision. The passes that close in halve their spacing each time, so ten of
# them resolve any of these axes far past what the tolerance asks; what they
# cannot do is travel. A pass reaches one step of the sweep either side of the
# best value it found, the next half a step, and so on, so the whole search can
# move about two sweep steps away from where the sweep pointed it and no
# further. What the sweep decides is therefore not how finely the answer is
# resolved but which part of the family it is resolved in.
#
# That makes the count matter in proportion to how small two steps are against
# the whole range. On the vertical rise, four values put two steps at two
# thirds of the range and there is nothing to gain. On the cut-off, forty-one
# values over a window some fifty seconds wide put two steps at a twentieth of
# it - and the cut-off is the steepest axis there is, at some eighty kilometres
# of apogee for every second of burn. Forty-one rather than twenty-five for
# that reason and no other, at a sweep half again as large.
CUT_OFF_NODES = 41


def _window_axis(window: tuple[float, float]) -> Range:
    low, high = window
    return Range(low, high, CUT_OFF_NODES)


class Family:
    """A pitch programme with every one of its parameters laid out as a grid.

    `ranges` is what the family is searched over when the caller says nothing:
    one entry per parameter, and a parameter that is held is held by a range of
    one node rather than by being left out. `build` turns one node into a
    programme and the instant the engines stop; `parameters` turns it into the
    specification that would fly it again.

    `TIMES` names the axes that are instants of the flight or durations of it,
    as against coefficients of the guidance law. Those are rounded to
    `TIME_QUANTUM` wherever the grid produces them, so that the answer is an
    answer a vehicle could be given.

    Two axes worth naming here, because they are the two whose default range is
    a single node.

    `coast` is on every family: how long the vehicle flies on after its
    programme has ended, holding the attitude it reached, before the engines
    stop. Zero by default, which is where every set on file has it and what
    makes the programme end at cut-off.

    `angle` is the flight-path angle the turn is aimed at, in degrees, and it
    is zero by default because a circular orbit is entered along the horizon.
    It is on the two families that can be aimed - the five-phase turn, which
    arrests its pitch rate on to a stated angle, and the bilinear tangent,
    whose third angle is the one it ends at. The velocity share has no such
    parameter and is not given an axis for one: its quartic drives the vertical
    share of the speed to exactly zero at the end of the turn, so the horizon
    is where it arrives by construction rather than by being asked to.

    Neither is fixed by the model, and both are ranges like the rest.
    """
    name: str
    # the axes that are instants of the flight or durations of it. The end of
    # the programme is one, because the engines stop `coast` after it
    TIMES: tuple[str, ...] = ()

    def ranges(self, window: tuple[float, float]) -> dict[str, Range]:
        raise NotImplementedError

    def build(self, values: dict[str, float]) -> tuple[PitchProgramme, float]:
        raise NotImplementedError

    def parameters(self, values: dict[str, float]) -> dict[str, float]:
        raise NotImplementedError


class FivePhase(Family):
    """The turn built from constant angular accelerations.

    Every number of it is searched: the vertical rise `t1`, the share `k2` of
    the turn spent building the pitch rate up, the share `k3` spent at a
    constant rate, and `t4`, where the turn ends. `k2` has a narrow range
    rather than a wide one for a reason of the model rather than of the search:
    driving it to zero costs a model that prices only the angle nothing, and
    what it buys is a phase no vehicle could fly, so it is bounded away from
    zero where a vehicle would bound it.
    """
    name = 'five-phase'
    TIMES = ('t1', 't4', 'coast')

    def ranges(self, window):
        return {
            't1': RISE,
            'k2': Range(0.03, 0.09, 4),
            # up to 0.9 rather than to 1: k2 + k3 = 1 leaves the fourth phase
            # no time to arrest the pitch rate in, and the rate it would need
            # to is divided by that nothing
            'k3': Range(0.0, 0.9, 19),
            't4': _window_axis(window),
            'angle': Range(0.0, 0.0, 1),
            'coast': Range(0.0, 0.0, 1),
        }

    def build(self, values):
        programme = FivePhaseProgramme(
            t1=values['t1'], t4=values['t4'], k2=values['k2'],
            k3=values['k3'], final_angle_deg=values['angle'])
        return programme, values['t4'] + values['coast']

    def parameters(self, values):
        written = {'type': self.name, 't1': values['t1'], 't4': values['t4'],
                   'k2': values['k2'], 'k3': values['k3']}
        # left out where it is zero, which is the default of the programme and
        # how every entry on file is written
        if values['angle']:
            written['final_angle_deg'] = values['angle']
        return written


class VelocityShare(Family):
    """The turn set by the share of the speed that stays vertical.

    `turn` is where the turn ends, as a share of the end of the programme,
    rather than as an instant of its own. The two are not independent - the
    family refuses a turn that outlasts the burn - so a share keeps every node
    of the grid inside the family wherever the end of the programme is searched
    to, where a pair of times would spend half the grid on sets that do not
    exist. The specification written out carries `tf` itself.
    """
    name = 'velocity-share'
    # `turn` is not among them: where the turn ends is a coefficient of the
    # quartic rather than an instant anything happens at, since the share of
    # the speed it drives to zero arrives there flat. The specification written
    # out carries the `tf` it comes to, unrounded, as the law's own number
    TIMES = ('t1', 'te', 'coast')

    def ranges(self, window):
        return {
            't1': RISE,
            # nine rather than six: where the turn ends is this family's
            # steep parameter, and two steps of six is two fifths of what it
            # can be given
            'turn': Range(0.5, 1.0, 9),
            # the quartic has an interior stationary point outside this, where
            # the share leaves [0, 1] and the turn kinks
            's': Range(-3.0, 3.0, 9),
            'te': _window_axis(window),
            'coast': Range(0.0, 0.0, 1),
        }

    def build(self, values):
        programme = VelocityShareProgramme(
            t1=values['t1'], tf=values['turn'] * values['te'],
            te=values['te'], s=values['s'])
        return programme, values['te'] + values['coast']

    def parameters(self, values):
        return {'type': self.name, 't1': values['t1'],
                'tf': values['turn'] * values['te'], 'te': values['te'],
                's': values['s']}


class BilinearTangent(Family):
    """The classical optimal-steering law, gridded through the angles it passes.

    a, b and c are not gridded directly. They are nearly degenerate - scaling b
    and c together leaves almost the same turn - so a grid over them would
    spend most of its nodes on programmes it had already flown. The angles the
    turn passes through are not degenerate, so the axes are the angle `start`
    the turn begins at, the angle `middle` it has reached at the fraction `mid`
    of the way through, and the angle `angle` it ends at; the coefficients are
    recovered from those three.

    `mid` is an axis rather than the midpoint it used to be fixed at. Where
    along the turn the middle angle is prescribed is what decides how much of
    the turn is done early, and it is no more a property of the vehicle than
    the angle itself is.

    The angle the turn starts at is bounded away from the horizon for a reason
    the other two families do not have. This one steps the flight-path angle at
    t1, from the vertical straight to whatever the tangent says, so the angle
    it starts at is also the size of that step; the further from 90 degrees it
    is asked to start, the less the turn resembles anything a vehicle flies.
    """
    name = 'bilinear-tangent'
    # `mid` is not among them, for the reason `turn` is not one of the velocity
    # share's: it says where the middle angle is prescribed, which is how the
    # law is written down rather than a moment of the flight
    TIMES = ('t1', 'te', 'coast')

    def ranges(self, window):
        return {
            't1': RISE,
            # nine rather than five, for the same reason as `turn` above:
            # this family steps its flight-path angle at t1, from the vertical
            # straight to whatever the tangent says, so the angle it starts at
            # is also the size of that step and the orbit answers to it sharply
            'start': Range(80.0, 89.6, 9),
            'mid': Range(0.5, 0.5, 1),
            'middle': Range(5.0, 60.0, 12),
            'te': _window_axis(window),
            'angle': Range(0.0, 0.0, 1),
            'coast': Range(0.0, 0.0, 1),
        }

    def build(self, values):
        a, b, c = self._coefficients(values)
        programme = BilinearTangentProgramme(t1=values['t1'], a=a, b=b, c=c,
                                             te=values['te'])
        return programme, values['te'] + values['coast']

    def parameters(self, values):
        a, b, c = self._coefficients(values)
        return {'type': self.name, 't1': values['t1'], 'a': a, 'b': b, 'c': c,
                'te': values['te']}

    @staticmethod
    def _coefficients(values):
        t1, te = values['t1'], values['te']
        if te <= t1:
            # said here rather than met inside the recovery, where it would
            # come back as a singular matrix and mean nothing
            raise ValueError(f'the turn has to end after it starts, and not '
                             f't1={t1:g}, te={te:g}')
        return bilinear_coefficients(t1, values['start'],
                                     t1 + values['mid'] * (te - t1),
                                     values['middle'], te, values['angle'])


FAMILIES = {family.name: family for family in
            (FivePhase, VelocityShare, BilinearTangent)}


def axis_names(programme: str) -> tuple[str, ...]:
    """The parameters of one family, in the order a grid walks them.

    The names on their own, so that a range given at the command line can be
    checked against the family before anything is estimated: a parameter that
    does not exist is a mistake in the command, and the command is where it
    should be answered rather than several minutes into a search.
    """
    if programme not in FAMILIES:
        raise ValueError(f'unknown pitch programme {programme!r}, expected one '
                         f'of {sorted(FAMILIES)}')
    # any window will do: which parameters a family has does not depend on
    # where the cut-off might fall, only on what they are searched over
    return tuple(FAMILIES[programme]().ranges((1.0, 2.0)))


# --- the search -----------------------------------------------------------


def plan(vehicle: LaunchVehicle, target_altitude: float, programme: str,
         *, latitude_deg: float = 0.0, azimuth_deg: float = 90.0,
         ranges: dict[str, Range] | None = None,
         tolerance: float = TOLERANCE,
         speed_tolerance: float = SPEED_TOLERANCE,
         refinements: int = REFINEMENTS, basins: int = BASINS, top: int = TOP,
         max_dynamic_pressure: float | None = None,
         coarseness: float = 1.0, steps_per_second: float = 10) -> SearchResult:
    """The grid a search would walk, before a single trajectory is flown.

    Everything a search settles before it starts: that the orbit is one this
    vehicle reaches at all, the window the cut-off is bounded to, the range and
    the step of every axis, and how many nodes the passes come to. A grid is
    cheap to get wrong and expensive to walk, so it is worth being able to look
    at one first - `ascent-search --dry-run` is this and nothing else.

    `search` begins by calling this, so the two cannot disagree about what is
    about to be searched.
    """
    if programme not in FAMILIES:
        raise ValueError(f'unknown pitch programme {programme!r}, expected one '
                         f'of {sorted(FAMILIES)}')
    family = FAMILIES[programme]()

    if target_altitude <= DRAG_CEILING:
        raise ValueError(
            f'a circular orbit at {target_altitude / 1000:g} km is inside the '
            f'air, and this model takes the air as gone above '
            f'{DRAG_CEILING / 1000:g} km rather than modelling an orbit in it')

    # what the pad hands the vehicle before the engines do, which the estimate
    # does not carry: it does not turn the Earth, as the dissertation does not.
    # Credited where the answer is a yes or a no, because a refusal has to be
    # made on the most generous reading there is - and a launch to the west,
    # which the pad charges rather than pays, is not made stricter for it
    from_the_pad = max(0.0, rotation_in_plane(latitude_deg, azimuth_deg)
                       * EARTH_RADIUS)
    reachable = equivalent_time(vehicle, target_altitude,
                                head_start=from_the_pad)
    if reachable is None:
        raise ValueError(
            f'{vehicle.name} does not reach a circular orbit at '
            f'{target_altitude / 1000:g} km: the velocity balance never closes, '
            f'whatever the programme is, and not with the {from_the_pad:.0f} m/s '
            f'the pad hands it either. Nothing was integrated.')

    # the plain balance is what the window was calibrated on, and the one with
    # the pad in it stands in on the orbits high enough that the plain one no
    # longer closes at all
    estimate = equivalent_time(vehicle, target_altitude) or reachable

    # the late end of the window is never past the instant the last tank runs
    # dry. A cut-off after that is not a cut-off - the engines have already
    # stopped - and the orbit would answer to the coast instead of to the burn
    dry = burns(vehicle)[-1].burn_out
    window = (estimate * (1.0 - TIME_MARGIN_EARLY),
              min(estimate * (1.0 + TIME_MARGIN_LATE), dry))
    if window[1] <= window[0]:
        raise ValueError(
            f'{vehicle.name} runs its last tank dry at {dry:.1f} s, before the '
            f'{window[0]:.1f} s the ascent to {target_altitude / 1000:g} km is '
            f'estimated to take at the earliest: there is no cut-off to search '
            f'over. Nothing was integrated.')

    grid = _grid(family, window, ranges, coarseness)
    result = SearchResult(
        best=None, found=[], vehicle=vehicle, target_altitude=target_altitude,
        programme=programme, latitude_deg=latitude_deg, azimuth_deg=azimuth_deg,
        steps_per_second=steps_per_second, ranges=grid,
        required_velocity=required_velocity(target_altitude),
        vacuum_time=vacuum_time(vehicle, target_altitude) or 0.0,
        equivalent_time=estimate, window=window, tolerance=tolerance,
        speed_tolerance=speed_tolerance, top=top,
        basins=max(1, basins),
        max_dynamic_pressure=max_dynamic_pressure)
    result.passes = refinements + 1
    result.schedule = step_schedule(result.passes, steps_per_second)
    result.planned_nodes = _planned_nodes(grid, refinements, max(1, basins))
    closed = halvings(result.schedule)
    result.spacing = {
        name: (max(span.step / 2 ** closed, TIME_QUANTUM)
               if name in family.TIMES else span.step / 2 ** closed)
        for name, span in grid.items() if span.nodes > 1}
    return result


def search(vehicle: LaunchVehicle, target_altitude: float, programme: str,
           *, latitude_deg: float = 0.0, azimuth_deg: float = 90.0,
           ranges: dict[str, Range] | None = None,
           tolerance: float = TOLERANCE,
           speed_tolerance: float = SPEED_TOLERANCE,
           refinements: int = REFINEMENTS, basins: int = BASINS, top: int = TOP,
           max_dynamic_pressure: float | None = None,
           coarseness: float = 1.0, steps_per_second: float = 10,
           workers: int | None = None, screen: bool = True,
           report=None) -> SearchResult:
    """Sweep a grid over the parameters of `programme` for a circular orbit.

    Every parameter of the family is an axis. `ranges` replaces the range of
    any of them - `{'t1': Range(10, 30, 11)}` - and a name the family does not
    have is refused rather than ignored. `Family.ranges` is what each family
    offers and what it is searched over when nothing is said; `plan` returns
    that grid without walking it.

    The sets found come back ranked by how far the orbit each reached is from
    the circle asked for, and `best` is the first of them that meets all three
    tolerances, or simply the first if none does.

    `refinements` is how many passes follow the sweep, each one grid step wide
    about the best node and halving the step; `coarseness` scales the nodes
    along every default axis, below one for a quicker and rougher look. It
    leaves an axis given in `ranges` alone: that step was asked for. `screen` is
    the altitude integral, which drops a node that cannot reach the target
    without flying it. `workers` is how many processes the nodes of a pass are
    divided over, two thirds of the cores by default and one for a search that
    runs where it is called. `report` is called with the result after every
    node, so a caller can show progress.
    """
    result = plan(vehicle, target_altitude, programme,
                  latitude_deg=latitude_deg, azimuth_deg=azimuth_deg,
                  ranges=ranges, tolerance=tolerance,
                  speed_tolerance=speed_tolerance, refinements=refinements,
                  basins=basins, top=top,
                  max_dynamic_pressure=max_dynamic_pressure,
                  coarseness=coarseness, steps_per_second=steps_per_second)
    family = FAMILIES[programme]()
    grid = result.ranges

    sweep = math.prod(span.nodes for span in grid.values())
    if sweep > NODE_LIMIT:
        raise ValueError(
            f'the grid asked for comes to {sweep:,} nodes in one pass, past '
            f'the {NODE_LIMIT:,} a search will start on. Look at it with '
            f'--dry-run, which prints every axis and what the passes come to '
            f'without walking any of them. Nothing was integrated.')

    # the ends asked for rather than the last node the step lands on: a sweep
    # walks the nodes its step reaches, and the passes that close in are free
    # to look anywhere inside the range the caller actually named
    bounds = {name: (span.low, span.high) for name, span in grid.items()}

    flight = _Flight(vehicle, family, target_altitude, latitude_deg,
                     azimuth_deg, steps_per_second, screen)
    result.workers = default_workers() if workers is None else max(1, workers)
    pool = (None if result.workers == 1 else
            ProcessPoolExecutor(max_workers=result.workers,
                                initializer=_begin, initargs=(flight,)))
    # how far either side of the best node the next pass looks, one entry per
    # axis being searched at all. It starts at one step of the sweep and halves
    # with every pass; an axis the caller held has no entry and is not closed
    # in on
    reach = {name: span.step for name, span in grid.items() if span.nodes > 1}
    sweep_grid = grid
    grids = [grid]
    # nothing has been closed in on yet, so a search that finds no orbit at all
    # reports the count it was asked for and never used
    followed = 0
    try:
        seen: dict[tuple, Candidate] = {}
        walked: set[tuple] = set()
        schedule = result.schedule
        for pass_number in range(refinements + 1):
            result.pass_number += 1
            _sweep(flight, grids, schedule[pass_number], result, seen, walked,
                   report, pool)
            if not seen:
                # nothing to close in on: a pass that found no orbit at all
                # leaves the next one nowhere to centre itself
                break
            result.found = sorted(seen.values(), key=_rank)
            result.best = _best(result)
            if pass_number == refinements:
                # the last pass has nothing after it to close in, so there is
                # nothing to work out and nothing to count: valleys picked here
                # would be reported as followed without a pass ever walking them
                break
            # centred on the head of the table rather than on the set the
            # search would answer with. The two differ only where the closest
            # orbit found does not yet meet the tolerances, and then it is the
            # closest orbit that says where the orbit is: what the next pass is
            # for is to walk the ranking downhill, and the ranking is the head -
            # and on the head of each of the other valleys being followed, for
            # which see `_centres` and `BASINS`
            centres = _centres(result.found, sweep_grid, family.TIMES, basins)
            # a pass that steps up to a finer one re-flies the ground the pass
            # before it covered, at a step whose answers can be compared with
            # what follows. So it keeps the reach it had rather than halving:
            # closing in on a coarse measurement is closing in on noise, and the
            # first pass at a new step is the one that has to see plainly
            stepping_up = schedule[pass_number + 1] != schedule[pass_number]
            # the most any pass came to rather than the last, so that a search
            # reports the widest it ever looked rather than how narrow the
            # ranking had grown by the end. Counted here, where the pass that
            # walks them is certain to follow
            result.basins = max(followed, len(centres))
            followed = result.basins
            grids = [_closer(sweep_grid, centre, reach, bounds)
                     for centre in centres]
            # five nodes over the two widths either side, so the next pass
            # resolves every axis it is searching twice as finely as this one -
            # except an axis of instants, which stops once its window is
            # narrower than the tenth of a second the flight is asked in,
            # because every value left in it would be the same instant
            reach = reach if stepping_up else {
                name: 0.5 * width for name, width in reach.items()
                if name not in family.TIMES or width >= 2.0 * TIME_QUANTUM}
    except KeyboardInterrupt:
        # the queue holds most of a pass, and a plain shutdown would wait for
        # every node of it before letting the interrupt through - which is the
        # whole pass, and the whole point of stopping was not to walk it
        if pool is not None:
            pool.shutdown(wait=False, cancel_futures=True)
            pool = None
        raise
    finally:
        if pool is not None:
            pool.shutdown()

    # a search that stopped early is reported as what was walked rather than as
    # what was planned - the schedule included, which is why it is cut to length
    # rather than laid again over the shorter count
    result.passes = result.pass_number
    result.schedule = result.schedule[:result.passes]
    result.planned_nodes = result.nodes
    # and the spacing it actually closed down to, which is not one halving a
    # pass: a pass that steps up to a finer integration keeps the reach it had
    closed = halvings(result.schedule)
    result.spacing = {
        name: (max(span.step / 2 ** closed, TIME_QUANTUM)
               if name in family.TIMES else span.step / 2 ** closed)
        for name, span in result.ranges.items() if span.nodes > 1}

    if result.best is not None:
        # against the step the passes closed down to rather than the step they
        # started from: what this is reporting is that the search converged on
        # to a bound of the range it was given and would have gone further, and
        # a sweep step is far too wide to tell that from an interior answer.
        # The count of halvings and not of passes, for the reason `halvings`
        # gives - a threshold four times finer than what was resolved would call
        # an interior answer an edge
        finest = 2 ** closed
        result.on_edge = tuple(
            name for name, (low, high) in bounds.items()
            if result.ranges[name].nodes > 1
            and min(abs(result.best.values[name] - low),
                    abs(result.best.values[name] - high))
            <= result.ranges[name].step / finest)
    return result


def _grid(family: Family, window: tuple[float, float],
          ranges: dict[str, Range] | None, coarseness: float) -> dict[str, Range]:
    """The axes the search will walk: the family's own, with `ranges` in place.

    Coarsening is of the family's own axes only. An axis the caller wrote out
    is a step the caller chose, and scaling it would answer a question that was
    not asked.
    """
    grid = _coarsen(family.ranges(window), coarseness)
    if not ranges:
        return grid

    unknown = [name for name in ranges if name not in grid]
    if unknown:
        raise ValueError(
            f'{", ".join(sorted(unknown))} '
            f'{"is not a parameter" if len(unknown) == 1 else "are not parameters"} '
            f'of the {family.name} turn; it is made of '
            f'{", ".join(grid)}')
    return {name: ranges.get(name, span) for name, span in grid.items()}


def _planned_nodes(grid: dict[str, Range], refinements: int,
                   basins: int = 1) -> int:
    """How many nodes the whole search will visit, known before it starts.

    The upper bound of it. A pass closes in on as many valleys as the ranking
    offers, up to `basins`, and where two of them lie near each other the nodes
    they share are walked once - so a search that follows five reports rather
    fewer than five passes' worth of nodes, and says so as it goes.
    """
    refined = math.prod(REFINED_NODES if span.nodes > 1 else 1
                        for span in grid.values())
    return (math.prod(span.nodes for span in grid.values())
            + refinements * refined * max(1, basins))


def _nodes(grid: dict[str, Range], times: tuple[str, ...] = ()):
    """Every combination of the grid, one set of values at a time.

    Rounded as it is produced rather than as it is used, so that two values of
    an instant that come to the same tenth of a second are the same node - one
    key, one trajectory, one row of the table - and not two answers that differ
    where nothing can act on the difference.
    """
    names = list(grid)
    for point in product(*(grid[name].values() for name in names)):
        yield {name: (round(value, TIME_DECIMALS) if name in times
                      else float(value))
               for name, value in zip(names, point)}


def _key(values: dict[str, float]) -> tuple:
    """What makes two nodes of the grid the same set.

    Every pass that closes in is a grid about a node of the pass before it, and
    it overlaps that pass: the node it is centred on is one of its own, and so
    is every node a whole width away. Between an eighth and a fifth of a pass
    has been walked already, and this is what recognises it - before it is
    flown, so that recognising it is what saves the trajectory.

    Rounded, because a coordinate reached by two different routes through the
    arithmetic differs in the last bit and is the same set.
    """
    return tuple((name, round(value, 9)) for name, value in sorted(values.items()))


def _closer(grid: dict[str, Range], centre: dict[str, float],
            reach: dict[str, float],
            bounds: dict[str, tuple[float, float]]) -> dict[str, Range]:
    """A grid of `REFINED_NODES` nodes about the best one, `reach` either side.

    `reach` halves from pass to pass, so with five nodes over two of them the
    step halves too, and ten passes take a step of a couple of seconds down to
    a couple of milliseconds. That is what the cut-off needs: near a circular
    orbit the apogee answers to it at some eighty kilometres a second, so a
    step of the sweep is worth tens of kilometres of orbit and only the passes
    that close in can resolve it.

    Held inside the range the axis was searched over. An axis walked past its
    own range is an axis the caller did not ask about, and where the best node
    comes out on a bound the search says so - `on_edge` - rather than wandering
    off to look.

    An axis the caller held has no reach and is not closed in on: a parameter
    that was held is held.
    """
    closer = {}
    for name, span in grid.items():
        if name not in reach:
            # held from the start, or an axis of instants that has closed in as
            # far as a tenth of a second lets it. Either way it stays where the
            # best set has it, which for one held from the start is where it
            # always was
            closer[name] = Range(centre[name], centre[name], 1)
            continue
        low, high = bounds[name]
        near, far = (max(low, centre[name] - reach[name]),
                     min(high, centre[name] + reach[name]))
        closer[name] = (Range(near, far, REFINED_NODES) if far > near
                        else Range(near, near, 1))
    return closer


def _centres(found: list[Candidate], sweep: dict[str, Range],
             times: tuple[str, ...], basins: int) -> list[dict[str, float]]:
    """The places the next pass closes in on, best first.

    The head of the ranking, and then the best set that is not already in a
    valley being followed, until `basins` of them are held or the table runs
    out.

    Two sets are the same valley when they sit within one scale of each other on
    every axis that was searched, and the scale is not the same on every axis.

    On a coefficient of the guidance law it is the step of the sweep: that is
    the scale the valleys were missed at, and it does not move. Not the reach of
    the pass, which halves from pass to pass - measuring by that would split one
    valley into finer and finer pieces as the search converged and spend the
    passes on distinctions inside an answer rather than on answers it has not
    looked at.

    On an axis of instants it is the tenth of a second the flight is asked in,
    and this is the one that matters. Such an axis stops being closed in on once
    its window is narrower than a tenth, which happens after six passes or so -
    and from then it does not move again. A cut-off that locked on to the wrong
    tenth cannot be recovered by any amount of shaping the turn, because the
    orbit answers to it at kilometres of apogee per tenth: the search converges,
    but on to a set a kilometre or two out. Read at the sweep step, every tenth
    within five seconds of the best would be the same valley and only one of
    them would ever be tried. Read at the tenth, they are what they are - one
    turn each, solved at its own cut-off.

    An axis the caller held is left out, because a value that is the same
    everywhere makes every pair of sets the same valley.
    """
    # half a tenth on an axis of instants, which is to say: the same tenth, and
    # nothing else. A whole tenth would fold the neighbouring one in, and the
    # neighbouring one is precisely what has to be tried - once such an axis has
    # locked, the grid a pass builds on it is the single value it locked to, so
    # a cut-off one tenth away is covered by nothing at all. On a coefficient a
    # whole step is right, because there the pass reaches one step either side
    # and does cover its neighbours
    scale = {name: (0.5 * TIME_QUANTUM if name in times else span.step)
             for name, span in sweep.items() if span.nodes > 1}
    if not scale or basins <= 1:
        return [found[0].values] if found else []

    centres: list[dict[str, float]] = []
    for candidate in found:
        values = candidate.values
        if any(all(abs(values[name] - taken[name]) <= step
                   for name, step in scale.items())
               for taken in centres):
            continue
        centres.append(values)
        if len(centres) == basins:
            break
    return centres


def _rank(candidate: Candidate) -> tuple[float, float]:
    """The order the sets found are reported in.

    How far the orbit is from the circle asked for, and then - so that two sets
    that reach the same orbit come back in the same order however many
    processes answered them - the earlier cut-off.
    """
    return (candidate.orbit_error, candidate.cutoff_time)


def _best(result: SearchResult) -> Candidate:
    """The set the search answers with.

    The best set that meets all three conditions of the orbit; failing that,
    simply the best, with `reaches_orbit` left to say that it does not meet
    them. A search that reaches nothing still has something to show, and what
    it shows is what a narrower search should be centred on.

    Chosen from the sets flown at the step the search was asked for, and only
    from the whole table where no pass has run at that step yet - which is the
    passes before the last of the coarse ones, where there is nothing else to
    answer with and nothing is being answered yet either.
    """
    reaching = result.reaching
    if reaching:
        return reaching[0]
    measured = result.measured
    return measured[0] if measured else result.found[0]


def _sweep(flight: "_Flight", grids: list[dict[str, Range]], steps: float,
           result: SearchResult, seen: dict[tuple, Candidate], walked: set,
           report, pool) -> None:
    """One pass over the grids: every node screened, the survivors flown.

    `grids` is one grid for the sweep and one per valley thereafter. They are
    walked as a single pass rather than as several, because they are one step
    of the same search: the ranking they feed is shared, the nodes where two of
    them overlap are worth flying once, and a caller watching the progress of a
    search is watching how much of it is left rather than which valley it is in.

    A node this search has already walked at this step is not walked again - see
    `_key` - which is a set of nodes every pass that closes in shares with the
    pass before it, and now also the nodes two valleys have in common where they
    lie near each other. At this step, and not simply at all: the passes do not
    all integrate at the same one, and the same node answered at one step a
    second and at ten is two answers, the second of which is the one worth
    having. A pass that steps up is a pass that flies its inheritance again.

    The rest are independent, so they are answered over a pool of processes
    where there is one, in the order of the grids, so that a search returns the
    same table however many of them there are. A set that asks more of the
    airframe than the caller allowed is put aside here rather than ranked: it is
    not a worse answer, it is not an answer.
    """
    values = []
    for grid in grids:
        for one in _nodes(grid, flight.family.TIMES):
            key = (_key(one), steps)
            if key in walked:
                result.revisited += 1
                continue
            walked.add(key)
            values.append(one)

    result.pass_nodes = len(values)
    result.pass_node = 0
    limit = result.max_dynamic_pressure

    jobs = [(one, steps) for one in values]
    answers = ((flight.at(*job) for job in jobs) if pool is None
               else pool.map(_answer, jobs, chunksize=8))

    for node in answers:
        result.nodes += 1
        result.pass_node += 1
        result.flown += node.flights
        setattr(result, node.outcome, getattr(result, node.outcome) + 1)
        if node.candidate is not None:
            if limit is not None and node.candidate.peak_dynamic_pressure > limit:
                result.over_pressure += 1
            else:
                # one row of the table per node, and no node is answered twice
                seen[node.candidate.key] = node.candidate
        if report is not None:
            report(result)


class _Flight:
    """Everything one node of the grid needs, held in one place.

    A node is answered in three steps: the family builds the programme and says
    when the engines stop, the altitude integral says whether it is aiming
    anywhere near the orbit, and - only then - the trajectory is integrated.

    Nothing here is written to. What a node came to is returned as a `Node` and
    counted by the caller, because the node may have been answered in another
    process, where anything written to would be written to a copy.
    """

    def __init__(self, vehicle, family, target_altitude, latitude_deg,
                 azimuth_deg, steps_per_second, screen=True):
        self.vehicle, self.family = vehicle, family
        self.target_altitude = target_altitude
        self.latitude_deg, self.azimuth_deg = latitude_deg, azimuth_deg
        self.steps_per_second = steps_per_second
        self.target_speed = circular_velocity(target_altitude)
        self.target_radius = EARTH_RADIUS + target_altitude
        # whether the altitude integral is allowed to reject a node unflown
        self.screen = screen

    def at(self, values: dict[str, float], steps: float) -> Node:
        """Answer one node of the grid, flying as little as it takes.

        `steps` is what this pass integrates at, which is not the same for every
        pass - see `COARSE_STEPS`. It comes with the node rather than living on
        the flight because the flight is handed to the worker processes once,
        when the pool starts, and the step changes under them as the passes go.
        """
        try:
            programme, cutoff_time = self.family.build(values)
        except ValueError:
            # the family refuses this set outright: phases out of order, a
            # share the quartic is not a turn over, a tangent through its pole,
            # three angles no bilinear tangent passes through
            return Node(values, 'refused', None, 0)

        if self.screen and not self._worth_flying(programme, values):
            return Node(values, 'screened', None, 0)

        flown = self._fly(programme, cutoff_time, steps)
        if flown is None:
            # the set cannot be flown by this vehicle: it runs out of speed
            # against its own programme, or the trajectory leaves the model
            return Node(values, 'failed', None, 1)

        candidate = self._measure(values, cutoff_time, steps, *flown)
        if candidate is None:
            return Node(values, 'no_orbit', None, 1)
        return Node(values, 'closed', candidate, 1)

    def _worth_flying(self, programme: PitchProgramme,
                      values: dict[str, float]) -> bool:
        """Whether the altitude integral says this set can reach the target.

        The integral is of the vertical component of the velocity over the
        programme, and it reads high - the air, the thrust deficit at sea level
        and the fall of gravity with altitude are all left out of it - so the
        band it is known to read high by turns it into a bound on the flight.

        Only asked of a set whose programme runs to cut-off, which is every set
        with no coast in it. A coast is powered flight the integral does not
        cover, so the figure would read low by however much the vehicle climbed
        over it, and a screen is a gate: it is turned off rather than widened
        by a guess.
        """
        if values.get('coast', 0.0) > 0.0:
            return True
        reached = analytic_altitude(self.vehicle, programme)
        return (ALTITUDE_RATIO_LOW * self.target_altitude <= reached
                <= ALTITUDE_RATIO_HIGH * self.target_altitude)

    def _fly(self, programme: PitchProgramme, cutoff_time: float,
             steps: float) -> tuple[Telemetry, Mission] | None:
        """Integrate one trajectory, or nothing if it cannot be flown."""
        try:
            mission = Mission(
                vehicle=self.vehicle, pitch_programme=programme,
                cutoff=CutoffAtTime(cutoff_time),
                target_altitude=self.target_altitude,
                duration=self._duration(cutoff_time, steps),
                steps_per_second=steps,
                latitude_deg=self.latitude_deg, azimuth_deg=self.azimuth_deg)
            telemetry = mission.run()
        except ValueError:
            return None
        return telemetry, mission

    def _duration(self, cutoff_time: float, steps: float) -> float:
        """How long to fly for: to the first whole step at or past the cut-off.

        No further, because the orbit is read off the end of the flight and
        every second of coast past the cut-off is a second in which something
        could still act on it. Not less either: `Mission` rounds the length of
        a flight to a whole number of steps, and one rounded back below the
        cut-off would leave the state at the end of it from before the engines
        stopped.
        """
        return math.ceil(cutoff_time * steps) / steps

    def _measure(self, values: dict[str, float], cutoff_time: float,
                 steps: float, telemetry: Telemetry,
                 mission: Mission) -> Candidate | None:
        """The three errors of one flight, or nothing if it reached no orbit."""
        orbit = mission.orbit
        if not orbit.is_orbit:
            # an open trajectory, or one whose perigee is under the surface.
            # Neither is an orbit and neither can be ranked among orbits: the
            # apsidal errors of a set that comes back down are not small
            # because it nearly stayed up
            return None

        # the last row of the flight, which is the row the orbit above was
        # built from. Not the last row before the cut-off: a flight is a whole
        # number of steps and the cut-off is not, so that row can be most of a
        # step early - and the vehicle is under some 30 m/s^2 right up to the
        # cut-off, which at a coarse step is tens of metres per second of
        # speed that the set was never going to have. The rows this side of the
        # cut-off are a coast, where nothing but the two-body motion acts and
        # the orbit does not change at all
        at = len(telemetry) - 1
        altitude = float(telemetry.altitude[at])
        speed = float(telemetry.inertial_speed[at])
        budget = velocity_budget(telemetry, mission.omega)

        altitude_miss = abs(altitude - self.target_altitude)
        # against the speed of the orbit asked for, and deliberately not
        # against the circular speed at the altitude this set happens to have
        # reached. A set that levels off twenty kilometres low at exactly the
        # speed a circle there wants is on a perfect orbit and not the one
        # asked for: measured against the target it shows that miss, and
        # measured against its own altitude it would show nothing at all
        speed_miss = abs(speed - self.target_speed)
        apogee_miss = abs(orbit.apogee_altitude - self.target_altitude)
        perigee_miss = abs(orbit.perigee_altitude - self.target_altitude)
        peak_pressure, peak_demand = _demands(telemetry, cutoff_time)

        return Candidate(
            values=values, parameters=self.family.parameters(values),
            cutoff_time=cutoff_time, orbit=orbit, steps_per_second=steps,
            altitude=altitude, speed=speed,
            flight_path_angle=float(telemetry.flight_path_angle[at]),
            altitude_miss=altitude_miss, speed_miss=speed_miss,
            miss=max(apogee_miss, perigee_miss),
            gravity_loss=budget.gravity, aerodynamic_loss=budget.aerodynamic,
            steering_loss=budget.steering,
            peak_dynamic_pressure=peak_pressure, peak_steering_demand=peak_demand,
            altitude_error=altitude_miss / self.target_altitude,
            speed_error=speed_miss / self.target_speed,
            # the two apsidal errors are taken against the radius where the
            # altitude error above is taken against the altitude, and they are
            # not the same denominator on purpose. An apogee and a perigee are
            # radii; dividing a difference of radii by an altitude would make
            # the same miss a different error at every target, which is not
            # what a figure the ranking is built from can afford. The altitude
            # error is a share of the altitude asked for, which is what it
            # reads as. So the two columns say different things and are not to
            # be compared with each other - only each with itself
            apogee_error=apogee_miss / self.target_radius,
            perigee_error=perigee_miss / self.target_radius)


# The flight a worker process answers its nodes with, set once when the process
# starts so that the vehicle and the family are handed over once rather than
# with every node.
_WORKER: _Flight | None = None


def _begin(flight: _Flight) -> None:
    """Set a worker up: the flight it answers with, and deafness to Ctrl+C.

    An interrupt reaches every process of a console at once, so without this
    each worker raises out of the middle of a trajectory and prints its own
    stack, which is a dozen of them for one press of two keys. The process that
    was asked keeps its own handler and is the one that stops the search - it
    shuts the pool down and cancels what has not started - so nothing is left
    running by their ignoring it.
    """
    global _WORKER
    _WORKER = flight
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def _answer(job: tuple[dict[str, float], float]) -> Node:
    values, steps = job
    return _WORKER.at(*job)


def _demands(telemetry: Telemetry, cutoff_time: float) -> tuple[float, float]:
    """What the ascent asked of the airframe and of the guidance.

    The dynamic pressure is the peak over the whole climb, which is what an
    airframe is designed against. The steering demand is the sine of the thrust
    deflection the programme calls for, taken over the powered flight under
    guidance; where it passes one there is no such deflection and the vehicle
    cannot hold the programme, so it says how far the steering loss beside it
    is a measurement at all.
    """
    up_to = telemetry.at(cutoff_time) + 1
    demand = telemetry.steering_demand[:up_to][telemetry.thrust[:up_to] > 0.0]
    return (_peak(telemetry.dynamic_pressure[:up_to]),
            float(np.abs(demand).max()) if len(demand) else 0.0)


def _peak(series: np.ndarray) -> float:
    """The height of a smooth maximum sampled at a uniform step.

    The rows of a flight are a sample of it, and the peak of the dynamic
    pressure falls between two of them as often as on one. Taking the largest
    row reads the peak low by however far the parabola through it and its
    neighbours rises above it, which at a coarse step is not nothing - and this
    figure is a constraint when the caller makes it one, so it is worth the
    three multiplications.
    """
    top = int(np.argmax(series))
    if top == 0 or top == len(series) - 1:
        return float(series[top])
    left, middle, right = (float(value) for value in series[top - 1:top + 2])
    curvature = left - 2.0 * middle + right
    if curvature >= 0.0:
        return middle
    offset = 0.5 * (left - right) / curvature
    return middle - 0.25 * (left - right) * offset
