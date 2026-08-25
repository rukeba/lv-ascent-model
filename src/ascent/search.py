"""Grid search for the parameters of a pitch programme.

Ask for a vehicle, a circular orbit and one of the three programme families,
and this sweeps a grid over the parameters of that family and reports the sets
that come closest to the orbit, best first.

**Every parameter of the turn is an axis.** Nothing is held behind the
caller's back: the vertical rise, the shape of the turn, the instant the
programme ends and the instant the engines do are all coordinates of the same
grid, and each of them can be given its own range and its own step. What is
held is held because a range said so - a range of one node - and the summary
prints every axis with the range it was searched over, so a figure that did not
move is a figure the caller can see was not asked to.

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

**Then the grid closes in.** The best node becomes the centre of a grid one
step wide along every axis that was searched, and the sweep runs again, halving
the step each pass. A step of the first pass is worth tens of kilometres of
apogee; the passes after it are what turn the region the sweep found into a set
that meets the tolerance.

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
# catalogue it sits between 4.8 per cent high and 9.1 per cent low, and these
# carry that band with something to spare.
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
    """One axis of the grid: from `low`, in steps of `step`, up to `high`.

    A step rather than a count of nodes, because a step is what the parameter
    is read in - two seconds of vertical rise, a hundredth of a share - and
    because it is what says how finely the answer is resolved. `high` is a
    ceiling and not necessarily a node: an axis from 10 to 30 in steps of 7
    stops at 24, and the summary prints where it actually stopped.

    A step of zero is an axis of one node, which is how a parameter is held.
    """
    low: float
    high: float
    step: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.low) or not math.isfinite(self.high) \
                or not math.isfinite(self.step):
            raise ValueError(f'a range has to be made of numbers, and not '
                             f'{self.low}:{self.high}:{self.step}')
        if self.high < self.low:
            raise ValueError(f'a range runs from low to high, and not from '
                             f'{self.low:g} to {self.high:g}')
        if self.step < 0.0:
            raise ValueError(f'a range steps forwards, and not by {self.step:g}')
        if self.step == 0.0 and self.high != self.low:
            raise ValueError(
                f'a range of {self.low:g} to {self.high:g} needs a step to say '
                f'how it is walked; a step of zero is a single value, and then '
                f'the two ends have to be the same')
        # a step small enough against the span that the count of nodes is no
        # longer a number. Answered here, where the range is written, rather
        # than met further down as an arithmetic error out of `nodes`
        if self.step > 0.0 and not math.isfinite((self.high - self.low) / self.step):
            raise ValueError(
                f'a step of {self.step:g} over {self.low:g} to {self.high:g} is '
                f'more nodes than there are numbers')

    @property
    def nodes(self) -> int:
        if self.step <= 0.0:
            return 1
        # the tolerance is what puts the top of the range on the grid when the
        # span is a whole number of steps and the arithmetic says otherwise -
        # 0 to 0.9 in steps of 0.05 is nineteen nodes, not eighteen
        return int(math.floor((self.high - self.low) / self.step + 1e-9)) + 1

    @property
    def last(self) -> float:
        """The top node, which is the ceiling only when the step divides it."""
        return self.low + (self.nodes - 1) * self.step

    def values(self) -> tuple[float, ...]:
        # built from the low end and a multiple rather than accumulated, so
        # that a node is where it says it is however many steps along it lies
        return tuple(self.low + index * self.step for index in range(self.nodes))

    def describe(self) -> str:
        if self.nodes == 1:
            return f'{self.low:g}, held'
        return (f'{self.low:g} to {self.last:g} step {self.step:g} '
                f'({self.nodes} nodes)')

    # what a range looks like on the command line, quoted back at whoever gets
    # it wrong. One place, so that the two ways of getting it wrong - the wrong
    # punctuation and the wrong count of numbers - are answered the same way
    SYNTAX = ('NAME=LOW:HIGH:STEP for a parameter to search, as in t1=10:30:2 '
              '- from 10 to 30 in steps of 2 - or NAME=VALUE to hold one, as '
              'in k2=0.05')

    @staticmethod
    def parse(text: str) -> tuple[str, "Range"]:
        """One axis as it is written on the command line.

            t1=10:30:2      from 10 to 30 in steps of 2
            k2=0.05         held at 0.05

        The equals sign separates the parameter from its numbers and the colons
        separate the numbers from each other, in the order a Python slice reads
        in: low, high, step.

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
            return name, Range(numbers[0], numbers[0], 0.0)
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
    """The same axes with the nodes along each scaled, for a quicker look.

    A factor below one lengthens the stride and thins the grid, which is the
    quicker and rougher sweep; above one it shortens it. Applied to the step
    rather than to a count of nodes, because a step is what an axis is made of
    here - and it is what the summary prints back.
    """
    if factor == 1.0:
        return ranges
    coarsened = {}
    for name, span in ranges.items():
        if span.nodes == 1:
            coarsened[name] = span
            continue
        # never past the whole axis in one stride: an axis has to keep both of
        # its ends, or a coarse pass would stop being a pass over the family
        step = min(span.step / factor, span.high - span.low)
        coarsened[name] = Range(span.low, span.high, step)
    return coarsened


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
    max_dynamic_pressure: float | None = None
    # processes the nodes of a pass were divided over
    workers: int = 1
    # axes whose best value came out on a bound of the grid, where a better set
    # may lie just outside
    on_edge: tuple[str, ...] = ()

    @property
    def solved(self) -> int:
        """Nodes for which a trajectory came out on an orbit."""
        return self.closed

    @property
    def reaches_orbit(self) -> bool:
        return self.best is not None \
            and self.best.reaches(self.tolerance, self.speed_tolerance)

    @property
    def reaching(self) -> list[Candidate]:
        """Every set found that meets all three conditions, best first."""
        return [candidate for candidate in self.found
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
        """
        if self.best is None or not self.reaches_orbit:
            raise ValueError(
                'the search found no set that reaches the orbit, and a set '
                'that misses it is not a catalogue entry')
        best = self.best
        altitude = self.target_altitude
        return {
            'vehicle': vehicle_file,
            # written whole where it is whole, as every entry on file is
            'target_altitude': int(altitude) if altitude == int(altitude) else altitude,
            'launch_site': {'latitude': self.latitude_deg, 'azimuth': self.azimuth_deg},
            'pitch_programme': best.parameters,
            'cutoff': {'type': 'time', 'time': best.cutoff_time},
            'simulation': {'duration': round(best.cutoff_time + duration_margin),
                           'steps_per_second': 10},
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
RISE = Range(12.0, 30.0, 6.0)

# Nodes along the cut-off axis of a first pass, spread over the window the
# ascent-time estimate gives. A step of that window is worth tens of kilometres
# of apogee, which is what the refining passes are for; a first pass finer than
# this costs more than closing in on what it found.
CUT_OFF_NODES = 25


def _window_axis(window: tuple[float, float]) -> Range:
    low, high = window
    return Range(low, high, (high - low) / (CUT_OFF_NODES - 1))


class Family:
    """A pitch programme with every one of its parameters laid out as a grid.

    `ranges` is what the family is searched over when the caller says nothing:
    one entry per parameter, and a parameter that is held is held by a range of
    one node rather than by being left out. `build` turns one node into a
    programme and the instant the engines stop; `parameters` turns it into the
    specification that would fly it again.

    Two axes every family has. `coast` is how long the vehicle flies on after
    its programme has ended, holding the attitude it reached, before the
    engines stop - zero by default, which is where every set on file has it and
    what makes the programme end at cut-off. `angle` is the flight-path angle
    the turn is aimed at, in degrees, and it is zero for the same reason: a
    circular orbit is entered along the horizon. Neither is fixed by the model,
    and both are ranges like the rest.
    """
    name: str

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

    def ranges(self, window):
        return {
            't1': RISE,
            'k2': Range(0.03, 0.09, 0.02),
            # up to 0.9 rather than to 1: k2 + k3 = 1 leaves the fourth phase
            # no time to arrest the pitch rate in, and the rate it would need
            # to is divided by that nothing
            'k3': Range(0.0, 0.9, 0.05),
            't4': _window_axis(window),
            'angle': Range(0.0, 0.0),
            'coast': Range(0.0, 0.0),
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

    def ranges(self, window):
        return {
            't1': RISE,
            'turn': Range(0.5, 1.0, 0.1),
            # the quartic has an interior stationary point outside this, where
            # the share leaves [0, 1] and the turn kinks
            's': Range(-3.0, 3.0, 0.75),
            'te': _window_axis(window),
            'coast': Range(0.0, 0.0),
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

    def ranges(self, window):
        return {
            't1': RISE,
            'start': Range(80.0, 89.6, 2.4),
            'mid': Range(0.5, 0.5),
            'middle': Range(5.0, 60.0, 5.0),
            'te': _window_axis(window),
            'angle': Range(0.0, 0.0),
            'coast': Range(0.0, 0.0),
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
         refinements: int = REFINEMENTS, top: int = TOP,
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
        max_dynamic_pressure=max_dynamic_pressure)
    result.passes = refinements + 1
    result.planned_nodes = _planned_nodes(grid, refinements)
    return result


def search(vehicle: LaunchVehicle, target_altitude: float, programme: str,
           *, latitude_deg: float = 0.0, azimuth_deg: float = 90.0,
           ranges: dict[str, Range] | None = None,
           tolerance: float = TOLERANCE,
           speed_tolerance: float = SPEED_TOLERANCE,
           refinements: int = REFINEMENTS, top: int = TOP,
           max_dynamic_pressure: float | None = None,
           coarseness: float = 1.0, steps_per_second: float = 10,
           workers: int | None = None, screen: bool = True,
           report=None) -> SearchResult:
    """Sweep a grid over the parameters of `programme` for a circular orbit.

    Every parameter of the family is an axis. `ranges` replaces the range of
    any of them - `{'t1': Range(10, 30, 2)}` - and a name the family does not
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
                  top=top, max_dynamic_pressure=max_dynamic_pressure,
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
    try:
        seen: dict[tuple, Candidate] = {}
        walked: set[tuple] = set()
        for _ in range(refinements + 1):
            result.pass_number += 1
            _sweep(flight, grid, result, seen, walked, report, pool)
            if not seen:
                # nothing to close in on: a pass that found no orbit at all
                # leaves the next one nowhere to centre itself
                break
            result.found = sorted(seen.values(), key=_rank)
            result.best = _best(result)
            # centred on the head of the table rather than on the set the
            # search would answer with. The two differ only where the closest
            # orbit found does not yet meet the tolerances, and then it is the
            # closest orbit that says where the orbit is: what the next pass is
            # for is to walk the ranking downhill, and the ranking is the head
            centre = result.found[0].values
            grid = _closer(grid, centre, reach, bounds)
            # five nodes over the two widths either side, so the next pass
            # resolves every axis it is searching twice as finely as this one
            reach = {name: 0.5 * width for name, width in reach.items()}
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
    # what was planned
    result.passes = result.pass_number
    result.planned_nodes = result.nodes

    if result.best is not None:
        # against the step the passes closed down to rather than the step they
        # started from: what this is reporting is that the search converged on
        # to a bound of the range it was given and would have gone further, and
        # a sweep step is far too wide to tell that from an interior answer
        finest = 2 ** max(result.passes - 1, 0)
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


def _planned_nodes(grid: dict[str, Range], refinements: int) -> int:
    """How many nodes the whole search will visit, known before it starts."""
    refined = math.prod(REFINED_NODES if span.nodes > 1 else 1
                        for span in grid.values())
    return math.prod(span.nodes for span in grid.values()) + refinements * refined


def _nodes(grid: dict[str, Range]):
    """Every combination of the grid, one set of values at a time."""
    names = list(grid)
    for point in product(*(grid[name].values() for name in names)):
        yield {name: float(value) for name, value in zip(names, point)}


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
            closer[name] = span
            continue
        low, high = bounds[name]
        near, far = (max(low, centre[name] - reach[name]),
                     min(high, centre[name] + reach[name]))
        closer[name] = Range(near, far, (far - near) / (REFINED_NODES - 1)
                             if far > near else 0.0)
    return closer


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
    """
    reaching = result.reaching
    return reaching[0] if reaching else result.found[0]


def _sweep(flight: "_Flight", grid: dict[str, Range], result: SearchResult,
           seen: dict[tuple, Candidate], walked: set, report, pool) -> None:
    """One pass over the grid: every node screened, the survivors flown.

    A node this search has already walked is not walked again - see `_key` -
    which is a set of nodes every pass that closes in shares with the pass
    before it. The rest are independent, so they are answered over a pool of
    processes where there is one, in the order of the grid, so that a search
    returns the same table however many of them there are. A set that asks more
    of the airframe than the caller allowed is put aside here rather than
    ranked: it is not a worse answer, it is not an answer.
    """
    values = []
    for one in _nodes(grid):
        key = _key(one)
        if key in walked:
            result.revisited += 1
            continue
        walked.add(key)
        values.append(one)

    result.pass_nodes = len(values)
    result.pass_node = 0
    limit = result.max_dynamic_pressure

    answers = ((flight.at(one) for one in values) if pool is None
               else pool.map(_answer, values, chunksize=8))

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

    def at(self, values: dict[str, float]) -> Node:
        """Answer one node of the grid, flying as little as it takes."""
        try:
            programme, cutoff_time = self.family.build(values)
        except ValueError:
            # the family refuses this set outright: phases out of order, a
            # share the quartic is not a turn over, a tangent through its pole,
            # three angles no bilinear tangent passes through
            return Node(values, 'refused', None, 0)

        if self.screen and not self._worth_flying(programme, values):
            return Node(values, 'screened', None, 0)

        flown = self._fly(programme, cutoff_time)
        if flown is None:
            # the set cannot be flown by this vehicle: it runs out of speed
            # against its own programme, or the trajectory leaves the model
            return Node(values, 'failed', None, 1)

        candidate = self._measure(values, cutoff_time, *flown)
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

    def _fly(self, programme: PitchProgramme,
             cutoff_time: float) -> tuple[Telemetry, Mission] | None:
        """Integrate one trajectory, or nothing if it cannot be flown."""
        try:
            mission = Mission(
                vehicle=self.vehicle, pitch_programme=programme,
                cutoff=CutoffAtTime(cutoff_time),
                target_altitude=self.target_altitude,
                duration=self._duration(cutoff_time),
                steps_per_second=self.steps_per_second,
                latitude_deg=self.latitude_deg, azimuth_deg=self.azimuth_deg)
            telemetry = mission.run()
        except ValueError:
            return None
        return telemetry, mission

    def _duration(self, cutoff_time: float) -> float:
        """How long to fly for: to the first whole step at or past the cut-off.

        No further, because the orbit is read off the end of the flight and
        every second of coast past the cut-off is a second in which something
        could still act on it. Not less either: `Mission` rounds the length of
        a flight to a whole number of steps, and one rounded back below the
        cut-off would leave the state at the end of it from before the engines
        stopped.
        """
        return math.ceil(cutoff_time * self.steps_per_second) / self.steps_per_second

    def _measure(self, values: dict[str, float], cutoff_time: float,
                 telemetry: Telemetry, mission: Mission) -> Candidate | None:
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
            cutoff_time=cutoff_time, orbit=orbit,
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


def _answer(values: dict[str, float]) -> Node:
    return _WORKER.at(values)


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
