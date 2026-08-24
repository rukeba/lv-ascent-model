"""Grid search for the parameters of a pitch programme.

Ask for a vehicle, a circular orbit and one of the three programme families,
and this returns the sets of parameters that fly that vehicle into that orbit,
best first. Best at how close the orbit came to the one asked for: the apogee
and the perigee against the circle, which is the terminal condition itself and
the only ranking that means anything while the parameters are still being
looked over. `CRITERIA` is where the others are, for when the question has
moved on from whether a set reaches the orbit to what it costs to fly.

A search is a table before it is an answer. What comes back is every distinct
set that closed an orbit, in that order, each with the three errors it is
judged by - the altitude at cut-off, the speed there, and the shape of the
orbit the two of them made - so that the choice between a set that is closer
and one that is cheaper stays with whoever is reading.

The grid runs over the shape of the turn. The cut-off is not one of its axes:
it is what the terminal condition on the speed fixes, and it is solved for at
every node instead. That division is not a convenience - it is how the two
conditions of a circular orbit divide between the parameters. The speed at
cut-off answers to the cut-off time and to almost nothing else, at some tens
of metres per second for each second of burn, so a grid fine enough to resolve
it along that axis would be enormous and a grid coarse enough to afford would
resolve nothing. The altitude reached, on the other hand, is what the shape of
the turn decides. So the cut-off is solved and the shape is searched.

The shape is not all a family has. The vertical rise, the share of the
five-phase turn spent building the pitch rate up, the instant the turn ends:
each of these was held while the catalogue was solved, and each is an axis of
the grid now. `free` is what says so - `all` by default, `none` for the search
that solved the catalogue, or the names of the ones to open. The nodes of a
pass are the product over the axes, so the difference is not small: the
five-phase grid goes from 19 nodes in its first pass to 5,700, and from 5 in
each pass after it to 625.

`band_tolerance` is the other half of that question. What comes back then is
not the one set but every set the passes flew that reaches the orbit and cuts
off within that of it - a range along each parameter rather than a value, which
is what a flat minimum looks like when it is not flattened into its best node.

Two estimates keep the cost down, and both are quadrature rather than
integration - see `estimates.py`:

  - the ascent-time estimate bounds the cut-off. It gives the interval the
    solve brackets the root in, and it says before anything is flown whether
    the vehicle has the propellant for the orbit at all;
  - the altitude integral screens every node of the grid. It says what altitude
    the shape would reach at either end of that interval, and a shape that
    cannot reach the target anywhere inside it is dropped without a flight.

The grid is then refined: the best node becomes the centre of a grid one step
wide, and the search runs again, halving the step it resolves the shape to. The
cut-off is re-solved from scratch at every node of every pass, so it never
inherits the resolution of the pass before it: the passes place the shape, and
the cut-off is as sharp on the first of them as on the last.

The nodes of a pass do not depend on one another - each is its own cut-off
solved over its own handful of trajectories - so they are answered over a pool
of processes, two thirds of the cores by default, and collected in the order of
the grid. A search returns the same set however many processes answered it.

Which node a pass refines about, where the ranking is by what the ascent costs
rather than by the orbit, is not simply the cheapest one that reached the orbit. At the resolution of an early pass, whether a node lands on the orbit at
all is largely luck, and a set half a kilometre out but two seconds quicker is
the better thing to look near. What the passes follow instead is the cut-off
each node would need to reach the target, read off the line its own pass draws
between the altitude reached and the instant of cut-off. Where that leads into
a corner of the family from which the orbit cannot be reached - which happens
on a vehicle near its limit - the grid is run a second time for the orbit
alone, and the better of the two answers is the one reported.
"""

import math
import os
from collections.abc import Sequence
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

# How far either side of the estimated ascent time the cut-off is bracketed.
# The estimate leaves the rotation of the Earth out and prices the losses off a
# turn that does not depend on the orbit, so it mostly reads low; against the
# catalogue it sits between 4.8 per cent high and 9.1 per cent low, and these
# carry that band with something to spare.
#
# The axes a search can open move the ascent - thirty seconds of vertical rise
# is gravity the estimate does not price, and a turn that ends early is speed
# spent differently - so the window was measured again over them, on three
# vehicles and eight orbits: every set that came within 20 km of its target cut
# off inside it. What clips it on Falcon 9 and Ariane 62 is not this margin
# anyway but the instant the last tank is dry, some 5 and 11 per cent past the
# estimate.
TIME_MARGIN_EARLY = 0.06
TIME_MARGIN_LATE = 0.15

# What the altitude integral reads high by. It leaves out the air, the thrust
# deficit at sea level and the fall of gravity with altitude, all of which push
# the same way; against the catalogue the figure it returns is between 1.005
# and 1.185 times the altitude the flight reaches, and a node is screened out
# only when the target lies outside what these two allow.
#
# The axes a search can open ask it about turns no entry of the catalogue
# holds - a rise of thirty seconds, k2 at three thousandths, a turn that ends
# well before the burn - so it was measured again over them: three vehicles,
# eight orbits, eleven hundred flights each, and the figure comes out between
# 0.961 and 1.222 times the altitude flown. It does not read high everywhere
# any more, which is what moved the low end from 0.95 to where it is.
#
# Both are wider than either measurement because the screen is a gate: a node
# it rejects is never flown, so a vehicle whose integral read a little further
# out than any of these would be reported as unable to reach an orbit it can
# reach. The margin costs some of the screening - a first pass of the velocity
# share still goes 83 per cent unflown against 87 at the measured band.
ALTITUDE_RATIO_LOW = 0.92
ALTITUDE_RATIO_HIGH = 1.40

# Both the perigee and the apogee have to land within this of the target for a
# set to count as reaching the orbit, m
TOLERANCE = 500.0

# What the sets a search found are ranked by, and so what the search reports
# first. Each takes the set and the search it belongs to.
#
# `orbit` is the default and is the terminal condition itself: how far the
# apogee and the perigee ended up from the circle that was asked for, added, in
# units of its radius. It is nought only when both apsides and the target are
# the same circle, so an eccentric orbit at the right mean altitude is caught by
# it where the altitude alone would pass. It is what the search this one grew
# out of ranked on, and while the parameters of a programme are still being
# looked over it is the only ranking that says anything: a set that does not
# reach the orbit is not cheap, it is wrong.
#
# The other two rank sets that all reach the orbit, by what they cost to fly.
# They are worth having and they are not the place to start: fix the orbit on
# Falcon 9 at 500 km and the cut-off of every set that reaches it falls in a
# quarter of a second, so `time` ranks by rounding, while `loss` spans some
# 60 m/s and walks the turn to the bounds of the grid on both t1 and k2.
CRITERIA = {
    'orbit': lambda candidate, result:
        candidate.errors(result.target_altitude)[2],
    'loss': lambda candidate, result: candidate.total_loss,
    'time': lambda candidate, result: candidate.cutoff_time,
}

# The sine of the thrust deflection a set may ask of the guidance and still
# count as an answer. At one there is no deflection that holds the programme -
# the thrust would have to be at right angles to the velocity - so a set past
# it is arithmetic rather than a trajectory.
#
# It is nevertheless None by default, which imposes nothing and reports the
# figure, because a limit here does not divide the good sets from the bad but
# whole vehicles from each other. Fifteen of the forty-two sets on file pass
# one, and every set of every kind that reaches 1100 km on H3 passes it: a
# search of that vehicle with the limit on returns nothing at all and says the
# orbit cannot be reached, which is not what the measurement means. Pass 1.0
# to ask for a programme the thrust can actually hold
MAX_STEERING_DEMAND = None

# The vertical rise is a construction choice rather than a solved unknown, s
VERTICAL_RISE = 20.0

# The cut-off is solved until the orbit is this share of the tolerance away
# from circular, measured as the gap between the semi-major axis and the radius
# at cut-off. That gap is what the apogee and the perigee each end up away from
# the mean, so a tenth of the tolerance leaves nine tenths of it for the
# altitude - and asking for a tighter orbit tightens the solve with it
CIRCULAR_SHARE = 0.1
CUT_OFF_PASSES = 40

# Nodes along each axis of a refining pass. The first pass has to cover the
# whole range of a family and is as wide as the family says; the passes after
# it only have to close in on a node already found. Five nodes halve the step
# each pass at the cost of five flights an axis, and over the whole search that
# is the cheapest way to a given resolution: a wider grid closes in faster per
# pass but pays for it as the square of its width
REFINED_NODES = 5

# How far either side of the cut-off already solved the next one is looked for
# before the whole window is fallen back on, s
NEIGHBOURING_CUT_OFF = 2.0

# What one node of the grid costs in trajectories. The cut-off solve takes a
# handful of them and a screened node takes none, and across the searches here
# it comes out near twelve. Nothing depends on it: it is what a search says it
# is about to cost, before it has flown anything to find out
FLIGHTS_PER_NODE = 12

# What one node of the grid can come to. Each is a field of `SearchResult`, and
# every node increments exactly one of them
OUTCOMES = ('screened', 'refused', 'no_cut_off', 'no_orbit', 'closed')


def default_workers() -> int:
    """Processes a search runs its nodes over: two thirds of the cores.

    Two thirds rather than all of them because a search is minutes long and the
    machine it runs on is being used for something else at the time. The nodes
    of one pass are independent - each is its own cut-off solved over its own
    handful of trajectories - so they divide over processes exactly, and it has
    to be processes: the work is Python arithmetic, and threads would queue up
    behind the interpreter lock rather than run beside each other.
    """
    return max(1, ((os.cpu_count() or 1) * 2) // 3)


@dataclass(frozen=True)
class Axis:
    """One axis of the grid: what is searched, over what, at what step."""
    low: float
    high: float
    nodes: int


@dataclass(frozen=True)
class Candidate:
    """One node of the grid, with the cut-off that closes its orbit."""
    # the grid coordinates, which is what the grid is refined about
    shape: dict[str, float]
    # the whole set as a pitch-programme specification
    parameters: dict[str, float]
    cutoff_time: float
    orbit: Orbit
    # the larger of the two terminal errors, m: what the tolerance is read against
    miss: float
    gravity_loss: float = 0.0
    aerodynamic_loss: float = 0.0
    steering_loss: float = 0.0
    # how far the orbit is from circular: the semi-major axis less the radius
    # at cut-off, m. What the cut-off was solved to drive to zero
    residual: float = 0.0
    # the state at cut-off, which is what the terminal conditions are conditions
    # on: altitude m, inertial speed m/s, flight-path angle deg
    altitude: float = 0.0
    speed: float = 0.0
    flight_path_angle: float = 0.0
    # what the ascent asks of the airframe and of the guidance. Neither enters
    # the ranking unless the caller sets a limit on the first, but a quicker
    # ascent is a flatter one and both are what it is paid for
    peak_dynamic_pressure: float = 0.0
    peak_steering_demand: float = 0.0

    @property
    def total_loss(self) -> float:
        return self.gravity_loss + self.aerodynamic_loss + self.steering_loss

    @property
    def described(self) -> dict[str, float]:
        """Everything that names this set: the specification and the grid.

        The two overlap and neither holds the other. The bilinear tangent is
        searched through the angles its turn passes and specified through the
        coefficients those angles give, so a table built from the
        specification alone cannot say where on the grid its rows came from,
        and one built from the grid alone cannot be flown.
        """
        named = {name: value for name, value in self.parameters.items()
                 if name != 'type'}
        named.update({name: value for name, value in self.shape.items()
                      if name not in named})
        return named

    def errors(self, target_altitude: float) -> tuple[float, float, float]:
        """The three terminal errors, relative: altitude, speed, orbit shape.

        The first two are read at cut-off and against the circular orbit that
        was asked for - the altitude of it, and the speed that holds it. The
        third is the shape: how far the apogee and the perigee each ended up
        from the radius of that orbit, added, so that it is nought only when
        the two apsides and the target are the same circle. An eccentric orbit
        at the right mean altitude shows up there and in neither of the others,
        which is why the old search this one grew out of ranked on it.
        """
        radius = EARTH_RADIUS + target_altitude
        altitude = abs(self.altitude - target_altitude) / target_altitude
        speed = abs(self.speed - circular_velocity(target_altitude)) \
            / circular_velocity(target_altitude)
        if not self.orbit.is_closed:
            return (altitude, speed, math.inf)
        shape = (abs(self.orbit.apogee_altitude - target_altitude)
                 + abs(self.orbit.perigee_altitude - target_altitude)) / radius
        return (altitude, speed, shape)


@dataclass(frozen=True)
class Node:
    """What one node of the grid came to, and what it cost to find out.

    Returned rather than recorded, because a node may be answered in another
    process: the counting is done by whoever collects it.
    """
    shape: dict[str, float]
    outcome: str
    candidate: Candidate | None
    flights: int


@dataclass
class SearchResult:
    """What the search found, and what it cost to find it."""
    best: Candidate | None
    vehicle: LaunchVehicle
    target_altitude: float
    programme: str
    latitude_deg: float
    azimuth_deg: float
    steps_per_second: float = 10
    # the grid of the first pass, name by name: what was searched and how
    # finely. The passes after it are `REFINED_NODES` along every one of these
    axes: dict[str, Axis] = field(default_factory=dict)
    # the estimates the search was bounded by
    required_velocity: float = 0.0
    vacuum_time: float = 0.0
    equivalent_time: float = 0.0
    window: tuple[float, float] = (0.0, 0.0)
    # nodes of the grid visited, dropped by the altitude integral, refused by
    # the family itself, left with no cut-off inside the window that closes a
    # circular orbit, and closed on something that is not an orbit at all - a
    # perigee under the surface
    nodes: int = 0
    screened: int = 0
    refused: int = 0
    no_cut_off: int = 0
    no_orbit: int = 0
    # nodes that came out on an orbit. Counted rather than derived, so that a
    # node falling through every branch would show as an inconsistency
    closed: int = 0
    # sets that reached an orbit and were put aside for asking more of the
    # airframe than the caller allowed
    over_pressure: int = 0
    # trajectories integrated, which is what the search actually costs
    flown: int = 0
    # where the search has got to. The passes and the nodes of each are known
    # before it starts, so `nodes` against `planned_nodes` is a share of the
    # work done - close enough to a share of the time, since the trajectories a
    # node takes vary little from one node to the next
    passes: int = 1
    pass_number: int = 0
    # 2 once the search has run the grid a second time, for the orbit alone
    attempts: int = 1
    pass_nodes: int = 0
    pass_node: int = 0
    planned_nodes: int = 0
    tolerance: float = TOLERANCE
    criterion: str = 'orbit'
    # how many of the sets found are worth printing, best first
    top: int = 10
    max_dynamic_pressure: float | None = None
    max_steering_demand: float | None = MAX_STEERING_DEMAND
    # sets that reached an orbit and were put aside for asking of the guidance
    # a deflection the thrust cannot give
    over_demand: int = 0
    # how much dearer than the set reported another may be and still be
    # counted part of the band, in the unit of the criterion: m/s of velocity
    # budget, or seconds of ascent. Nought reports the one set, as before
    band_tolerance: float = 0.0
    # every distinct set the search flew that closed an orbit at all, best
    # first by the criterion. What the top of the report is taken from, and
    # what the band is drawn from
    found: list[Candidate] = field(default_factory=list)
    band: tuple[Candidate, ...] = ()
    # processes the nodes of a pass were divided over
    workers: int = 1
    # axes whose best value came out on a bound of the grid, where a better set
    # may lie just outside
    on_edge: tuple[str, ...] = ()

    @property
    def reaching(self) -> list[Candidate]:
        """The sets found that meet the tolerance, best first."""
        return [candidate for candidate in self.found
                if candidate.miss <= self.tolerance]

    @property
    def solved(self) -> int:
        """Nodes for which a cut-off was found and the orbit closed.

        Every node ends in exactly one of the five counts above, and this is
        the last of them under its older name.
        """
        return self.closed

    @property
    def reaches_orbit(self) -> bool:
        return self.best is not None and self.best.miss <= self.tolerance

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


# What the vertical rise covers where it is searched rather than held. Every
# family has it and every family holds it at `VERTICAL_RISE` unless it is
# asked for. Twelve seconds is about as short a rise as a vehicle at a thrust
# to weight of one and a half has cleared the tower by; past thirty the rise
# is most of the gravity loss of the whole ascent, and no set on file is near
# either end
RISE_AXIS = Axis(12.0, 30.0, 5)


class Family:
    """A pitch programme with the shape of its turn laid out as a grid.

    None of the three has the cut-off as an axis: it is what the terminal
    condition on the speed solves for. What is gridded is the shape - one
    number for the five-phase turn, two for each of the others - and, where
    the caller asks for them, the numbers the family would otherwise hold.

    Those are the axes named in `FREE`, and they are searched unless the
    caller says otherwise: `free='all'` is the default, `free='none'` holds
    every one of them where the catalogue holds it, and a sequence of names
    opens those and holds the rest. `none` is the search that solved
    `config/catalogue.yaml` before any of this was gridded, which is why it is
    still reachable and still tested.

    The default is `all` because holding them was never a statement about the
    physics. The vertical rise and the five-phase k2 are construction choices
    that were fixed to make the problem square - two unknowns for two terminal
    conditions - and a turn ending at cut-off was a degree of freedom nobody
    had the nodes to pay for. None of that says the minimum lies where they
    were fixed, and a study of how the turn changes with altitude cannot be
    made of parameters that were pinned before it started.

    The nodes of a pass are the product over the axes, so every axis searched
    multiplies the pass. The counts in `FREE` are coarser than the shape's own
    for that reason - the passes halve their step whatever they start at, and
    a first pass as fine along four axes as along one is a search nobody
    waits for.
    """
    name: str
    # what this family holds constant and a search may open, with the nodes of
    # the first pass along each
    FREE: dict[str, Axis] = {}

    def __init__(self, *, free: "Sequence[str] | str" = 'all') -> None:
        names = self.names(free)
        unknown = [name for name in names if name not in self.FREE]
        if unknown:
            raise ValueError(
                f'the {self.name} family cannot search '
                f'{", ".join(unknown)}: what it can search beside its own '
                f'shape is {", ".join(self.FREE) if self.FREE else "nothing"}')
        # in the order `FREE` gives rather than the order they were asked for,
        # so that a grid is laid out the same way however the flags were typed
        self.free = {name: axis for name, axis in self.FREE.items()
                     if name in names}

    @classmethod
    def names(cls, free: "Sequence[str] | str") -> tuple[str, ...]:
        """The axes `free` stands for: all of them, none, or the ones named."""
        if free == 'all':
            return tuple(cls.FREE)
        if free == 'none':
            return ()
        if isinstance(free, str):
            raise ValueError(f'free is `all`, `none` or a sequence of axis '
                             f'names, and not {free!r}')
        return tuple(free)

    def axes(self) -> dict[str, Axis]:
        raise NotImplementedError

    def build(self, t1: float, end: float,
              shape: dict[str, float]) -> PitchProgramme:
        raise NotImplementedError

    def parameters(self, t1: float, end: float,
                   shape: dict[str, float]) -> dict[str, float]:
        raise NotImplementedError

    def rise(self, t1: float, shape: dict[str, float]) -> float:
        """The vertical rise: off the grid where it is one of the axes."""
        return self.searched('t1', shape, t1)

    def searched(self, name: str, shape: dict[str, float], held: float) -> float:
        """What one number of the turn is: off the grid, or where it is held.

        Asked of `free` rather than of the shape, so that a shape carrying more
        than this family searches - one built by hand, or kept from a search
        that opened more - cannot quietly override what the caller held.
        """
        return shape[name] if name in self.free else held


class FivePhase(Family):
    """The turn built from constant angular accelerations.

    Four numbers describe it and the grid runs over all four: the vertical
    rise t1, the share k2 of the turn spent building the pitch rate up, the
    share k3 spent holding it, and the instant t4 the turn ends at - that last
    one as a share of the cut-off, which is solved for rather than searched.

    Only k3 was gridded while the catalogue was solved, and `free='none'` is
    still that search. What held the other three was the shape of the problem
    rather than the physics: two terminal conditions leave room for two
    unknowns, so k3 and the cut-off were the unknowns and the rest were fixed
    where they looked reasonable.

    k2 is the one to watch. Driving it towards zero always pays, because a
    step in pitch rate costs nothing to a model that prices only the angle,
    and what it buys is a phase no vehicle could fly - so a search that ranks
    by the ascent alone walks k2 to the bottom of its axis and reports a turn
    that starts with a jerk. The axis is there to show how wide the band is;
    what sits at the bottom of it is a bound, not an optimum, and the peak
    steering demand reported beside the set is where that shows.

    t4 is what gives the family the fifth phase it is named for: the turn ends
    at t4 and the vehicle flies on the attitude it reached until the cut-off,
    so a t4 below the cut-off is a stretch of free flight and a t4 at it is
    none. Held at the cut-off, as the catalogue holds it, the family has four
    phases and the fifth is empty.
    """
    name = 'five-phase'

    FREE = {
        't1': RISE_AXIS,
        # the share of the turn spent building the pitch rate up. The bottom
        # of the range is a step in the rate in all but name and the top is
        # three times what every set on file holds it at. Fifteen nodes rather
        # than the five the other opened axes get: a thousandth of this share
        # is a visibly different trajectory, so a first pass that steps it in
        # hundredths steps over most of what it is looking for
        'k2': Axis(0.003, 0.15, 15),
        # where the turn ends, as a share of the cut-off rather than as an
        # instant: the cut-off is solved for at every node, and a share is the
        # one way of naming the end of the turn that stays inside the burn
        # wherever the solve lands. One is the turn ending with the burn
        't4': Axis(0.85, 1.0, 4),
    }

    def __init__(self, k2: float = 0.05, *,
                 free: "Sequence[str] | str" = 'all') -> None:
        # keyword-only, because `FivePhase(('t1',))` would otherwise read a
        # list of axes as the value to hold k2 at and say nothing about it
        super().__init__(free=free)
        self.k2 = k2

    def axes(self):
        # the family's own range, less a thousandth at the top: k2 + k3 = 1
        # leaves the fourth phase no time to arrest the pitch rate in, and the
        # rate it would need to is divided by that nothing. Where k2 is
        # searched too, the top of k3 is what the smallest k2 on its axis
        # allows, and the nodes past 1 - k2 are refused as they are reached -
        # an axis cannot depend on another, and the family already says no
        smallest = self.free['k2'].low if 'k2' in self.free else self.k2
        return {'k3': Axis(0.0, 1.0 - smallest - 1e-3, 19), **self.free}

    def build(self, t1, end, shape):
        return FivePhaseProgramme(t1=self.rise(t1, shape),
                                  t4=self._end_of_turn(end, shape),
                                  k2=self.searched('k2', shape, self.k2),
                                  k3=shape['k3'])

    def parameters(self, t1, end, shape):
        return {'type': self.name, 't1': self.rise(t1, shape),
                't4': self._end_of_turn(end, shape),
                'k2': self.searched('k2', shape, self.k2), 'k3': shape['k3']}

    def _end_of_turn(self, end: float, shape: dict[str, float]) -> float:
        """Where the turn ends: at the cut-off, or at the share of it searched."""
        return self.searched('t4', shape, 1.0) * end


class VelocityShare(Family):
    """The turn set by the share of the speed that stays vertical.

    The turn ends at `turn` times the cut-off and the quartic's fullness is s.
    Taking the end of the turn as a share of the cut-off rather than as a time
    of its own keeps every node inside the family's own rule that the turn end
    before the burn does, wherever the cut-off is solved to.
    """
    name = 'velocity-share'

    # the end of the turn is already an axis here, as a share of the cut-off,
    # so the vertical rise is the whole of what this family holds
    FREE = {'t1': RISE_AXIS}

    def axes(self):
        return {'turn': Axis(0.5, 1.0, 11), 's': Axis(-3.0, 3.0, 13),
                **self.free}

    def build(self, t1, end, shape):
        return VelocityShareProgramme(t1=self.rise(t1, shape),
                                      tf=shape['turn'] * end, te=end,
                                      s=shape['s'])

    def parameters(self, t1, end, shape):
        return {'type': self.name, 't1': self.rise(t1, shape),
                'tf': shape['turn'] * end, 'te': end, 's': shape['s']}


class BilinearTangent(Family):
    """The classical optimal-steering law, gridded through the angles it passes.

    a, b and c are not gridded directly. They are nearly degenerate - scaling b
    and c together leaves almost the same turn - so a grid over them would
    spend most of its nodes on programmes it had already flown. The angles the
    turn passes through are not degenerate, so the axes are the angle it starts
    at and the angle it has reached halfway, and the coefficients are recovered
    from those.

    The third angle the recovery needs is the one at cut-off, and it is zero:
    that is the horizontal velocity a circular orbit asks for, imposed on the
    family rather than searched for. It is also what makes the numerator of the
    tangent cancel at the end of the turn, which is what levels the vehicle out.

    The angle the turn starts at is bounded away from the horizon for a reason
    the other two families do not have. This one steps the flight-path angle at
    t1, from the vertical straight to whatever the tangent says, so the angle
    it starts at is also the size of that step; the further from 90 degrees it
    is asked to start, the less the turn resembles anything a vehicle flies.
    The sets on file start between 84.7 and 89.2 degrees, and a search that
    comes out on this bound is told so rather than allowed past it.
    """
    name = 'bilinear-tangent'

    FREE = {
        't1': RISE_AXIS,
        # where along the turn the middle angle is prescribed, as a share of
        # it. Half way is a choice of coordinates rather than of shape - the
        # law is the same three coefficients wherever the angle is read off -
        # but which turns the grid can reach depends on it, and a middle read
        # early bends the first half of the turn where one read late bends the
        # second. The ends are left out: an angle prescribed next to either
        # end of the turn is two conditions on almost the same instant, and
        # the recovery of the coefficients is ill-conditioned there
        'middle_at': Axis(0.2, 0.8, 5),
    }

    def axes(self):
        return {'start': Axis(78.0, 89.6, 12), 'middle': Axis(2.0, 60.0, 13),
                **self.free}

    def build(self, t1, end, shape):
        a, b, c = self._coefficients(t1, end, shape)
        return BilinearTangentProgramme(t1=self.rise(t1, shape), a=a, b=b, c=c,
                                        te=end)

    def parameters(self, t1, end, shape):
        a, b, c = self._coefficients(t1, end, shape)
        return {'type': self.name, 't1': self.rise(t1, shape), 'a': a, 'b': b,
                'c': c, 'te': end}

    def _coefficients(self, t1, end, shape):
        rise = self.rise(t1, shape)
        middle_at = rise + self.searched('middle_at', shape, 0.5) * (end - rise)
        return bilinear_coefficients(rise, shape['start'], middle_at,
                                     shape['middle'], end, 0.0)


FAMILIES = {family.name: family for family in
            (FivePhase, VelocityShare, BilinearTangent)}


# --- the search -----------------------------------------------------------


def search(vehicle: LaunchVehicle, target_altitude: float, programme: str,
           *, latitude_deg: float = 0.0, azimuth_deg: float = 90.0,
           t1: float = VERTICAL_RISE, k2: float = 0.05,
           free: "Sequence[str] | str" = 'all', band_tolerance: float = 0.0,
           ranges: "dict[str, Axis] | None" = None, criterion: str = 'orbit',
           top: int = 10, tolerance: float = TOLERANCE, refinements: int = 10,
           max_dynamic_pressure: float | None = None,
           max_steering_demand: float | None = MAX_STEERING_DEMAND,
           coarseness: float = 1.0, steps_per_second: float = 10,
           workers: int | None = None, report=None) -> SearchResult:
    """Parameters that fly `vehicle` into a circular orbit at `target_altitude`.

    What comes back is every set the search flew that closed an orbit, ranked
    by `criterion` - by default how far the orbit it closed is from the one
    asked for, and otherwise by what the ascent cost. `result.found` is that
    list, `result.best` its head, and `reaches_orbit` says whether the head
    meets `tolerance`.

    The grid is run twice if the first run does not reach the orbit: once
    preferring the cheapest set within reach, once preferring the closest.
    Ranked by the orbit, the first of those already is the second, and the grid
    is run once.

    `max_dynamic_pressure` puts the airframe into the constraint: a set that
    asks more of it than that is put aside however cheap it is. Left out, the
    peak is reported and nothing more, which is how the rest of the model
    treats the figure. `max_steering_demand` does the same for the guidance and
    is not left out by default: a set that asks a deflection the thrust cannot
    give is not a trajectory. None lifts it.

    `ranges` replaces what a family says an axis covers, one axis at a time -
    `{'k2': Axis(0.04, 0.08, 9)}` - which is how a search is narrowed on to
    what a coarser one found. The bounds it gives are bounds on the refining
    passes too, so a narrowed axis stays narrowed.

    `free` says which of the numbers a family would otherwise hold - the
    vertical rise, the five-phase k2, the end of the turn - are axes of the
    grid beside the shape; `Family.FREE` is what each family offers. All of
    them by default. `free='none'` holds every one where the catalogue holds
    it and searches the shape alone, which is the search that solved the
    catalogue and costs a fraction of this one.

    `top` is how many of the sets found are worth printing, which is a
    reporting choice rather than a search one: the whole of `result.found` is
    there whatever it is set to.

    `band_tolerance` is how much dearer than the set reported another may be
    and still be reported beside it, in the unit of the criterion: `result.band`
    is every set visited that reached the orbit and came inside that. It is
    what says how flat the minimum is, and it costs nothing - the sets are the
    ones the passes flew anyway.

    `refinements` is how many passes follow the first, each halving the step
    the shape is resolved to; `coarseness` scales the nodes along every axis of
    that first pass, below one for a quicker and rougher search. `workers` is
    how many processes the nodes of a pass are divided over, two thirds of the
    cores by default and one for a search that runs where it is called. `report` is
    called with the result after every node, so a caller can show progress: the
    passes and the nodes of each are known before the search starts, so how far
    it has got is known too.
    """
    if programme not in FAMILIES:
        raise ValueError(f'unknown pitch programme {programme!r}, expected one '
                         f'of {sorted(FAMILIES)}')
    if criterion not in CRITERIA:
        raise ValueError(f'unknown criterion {criterion!r}, expected one of '
                         f'{sorted(CRITERIA)}')
    family = (FivePhase(k2, free=free) if programme == 'five-phase'
              else FAMILIES[programme](free=free))

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
    # stopped - and the orbit would answer to the coast instead of to the burn,
    # which is what would take the monotony the solve below stands on
    window = (estimate * (1.0 - TIME_MARGIN_EARLY),
              min(estimate * (1.0 + TIME_MARGIN_LATE),
                  burns(vehicle)[-1].burn_out))
    result = SearchResult(
        best=None, vehicle=vehicle, target_altitude=target_altitude,
        programme=programme, latitude_deg=latitude_deg, azimuth_deg=azimuth_deg,
        steps_per_second=steps_per_second,
        required_velocity=required_velocity(target_altitude),
        vacuum_time=vacuum_time(vehicle, target_altitude) or 0.0,
        equivalent_time=estimate, window=window, tolerance=tolerance,
        criterion=criterion, top=top,
        max_dynamic_pressure=max_dynamic_pressure,
        max_steering_demand=max_steering_demand,
        band_tolerance=band_tolerance)

    axes = _narrow(_coarsen(family.axes(), coarseness), ranges)
    result.axes = axes
    bounds = {name: (axis.low, axis.high) for name, axis in axes.items()}
    result.passes = refinements + 1
    result.planned_nodes = _planned_nodes(axes, refinements)
    flight = _Flight(vehicle, family, t1, target_altitude, window,
                     latitude_deg, azimuth_deg, steps_per_second,
                     tolerance * CIRCULAR_SHARE)

    result.workers = default_workers() if workers is None else max(1, workers)
    pool = (None if result.workers == 1 else
            ProcessPoolExecutor(max_workers=result.workers,
                                initializer=_begin, initargs=(flight,)))
    try:
        settled = _passes(flight, axes, bounds, refinements, result, report,
                          pool, by_time=True)
        if not result.reaches_orbit and criterion != 'orbit':
            # minimising the cost has led into a corner of the family where
            # the orbit cannot be reached at all - which happens where a
            # vehicle is near its limit, the cheapest sets of a pass lying just
            # outside what it can still close. Run the grid again for the orbit
            # alone. Not where the orbit was already the criterion: that second
            # run would be the first one over again
            result.attempts = 2
            result.passes += refinements + 1
            result.planned_nodes += _planned_nodes(axes, refinements)
            # and without the altitude integral this time: the band it
            # rejects on is measured rather than derived, and whatever the
            # first run failed to find, it will not have been for want of
            # trying a shape the integral mistrusted
            settled = _passes(flight,
                              _narrow(_coarsen(family.axes(), coarseness),
                                      ranges),
                              bounds, refinements, result, report, pool,
                              by_time=False, screen=False) or settled
    finally:
        if pool is not None:
            pool.shutdown()

    # a pass that solved nothing stops the search where it stands, so the count
    # of passes and of nodes is corrected to what was actually walked rather
    # than left saying what was planned
    result.passes = result.pass_number
    result.planned_nodes = result.nodes

    if result.best is not None and settled is not None:
        result.on_edge = tuple(
            name for name, (low, high) in bounds.items()
            if min(abs(result.best.shape[name] - low),
                   abs(result.best.shape[name] - high)) <= _step(settled[name]))
    # the centre of a pass is a node of the pass that follows it, so a set
    # kept from one pass to the next was flown by both; it is one set
    result.found = _distinct(result.found, lambda c: _order(c, result))
    result.band = _band(result)
    return result


def _order(candidate: Candidate, result: SearchResult) -> tuple:
    """Where a set comes in the table, and so which of them is the answer.

    Whether it reached the orbit comes first, because the terminal condition is
    a condition and not a preference: a set half a kilometre out is not a
    cheaper answer than one on the orbit, it is not an answer. Among the sets
    that reached it, the criterion decides.

    Among the sets that missed, what decides is how far they missed by,
    whatever the criterion - the cheapest way to miss an orbit is of no use to
    anyone, and where nothing reached, the closest sets are the whole of what
    there is to look at.
    """
    if candidate.miss > result.tolerance:
        return (True, candidate.miss, 0.0)
    return (False, *_rank(candidate, result))


def _distinct(candidates: list[Candidate], value) -> list[Candidate]:
    """The same sets with each node of the grid counted once, best first."""
    seen, distinct = set(), []
    for candidate in sorted(candidates, key=value):
        key = tuple(round(value, 9) for value in candidate.shape.values())
        if key not in seen:
            seen.add(key)
            distinct.append(candidate)
    return distinct


def _narrow(axes: dict[str, Axis],
            ranges: "dict[str, Axis] | None") -> dict[str, Axis]:
    """The axes with what the caller narrowed them to put in place.

    An axis the family does not have is refused rather than added: a grid is
    the family's own coordinates, and a name that is not one of them is a
    mistake at the command line rather than a search worth running.
    """
    if not ranges:
        return axes
    unknown = [name for name in ranges if name not in axes]
    if unknown:
        raise ValueError(f'cannot narrow {", ".join(unknown)}: this search '
                         f'runs over {", ".join(axes)}')
    for name, given in ranges.items():
        axis = axes[name]
        if given.low < axis.low - 1e-12 or given.high > axis.high + 1e-12:
            raise ValueError(
                f'cannot narrow {name} to {given.low:g}-{given.high:g}: the '
                f'family searches {axis.low:g}-{axis.high:g}, and a range '
                f'outside that is a different family rather than a narrower '
                f'search - the screen and the terminal conditions are read '
                f'against what the family declares')
    return {name: ranges.get(name, axis) for name, axis in axes.items()}


def _band(result: SearchResult) -> tuple[Candidate, ...]:
    """Every set the search flew that reaches the orbit and costs as little.

    As little meaning within `band_tolerance` of the set reported, measured in
    whatever the criterion is - metres per second of budget, or seconds of
    ascent. At nought, which is the default, that is the set reported and
    whatever ties with it. Sets a shade cheaper than the one reported belong to
    the band too, where the ranking rounded them together: it prefers the
    earlier cut-off only down to the step the trajectory was integrated at, and
    inside a step it takes the closer orbit.

    A set is in the band only if it reaches the orbit. The closest set found is
    worth showing when nothing reaches, and it is not one of a band of
    solutions.

    The band is drawn from the nodes the passes visited and from nothing else.
    That is what makes it free, and it is also its limit: the first pass covers
    the whole range of the family at the step `Family.FREE` gives, and every
    pass after it covers one step of the pass before, so the band is a coarse
    map of the whole range with a fine one about the minimum inside it. It says
    the minimum is flat and roughly over what; it does not say the band ends
    exactly where its widest set does.
    """
    if result.best is None or not result.reaches_orbit:
        return ()
    dearest = _banded(result.best, result) + result.band_tolerance
    return tuple(candidate for candidate in result.reaching
                 if _banded(candidate, result) <= dearest)


def _banded(candidate: Candidate, result: SearchResult) -> float:
    """What a set scores for the band, rounded as the ranking rounds it.

    Which matters only for the ascent time, where the ranking treats two
    cut-offs inside one integration step as the same ascent: rounding here as
    well is what keeps a band of nothing from dropping the sets the ranking had
    already called ties.
    """
    value = CRITERIA[result.criterion](candidate, result)
    if result.criterion != 'time':
        return value
    step = 1.0 / result.steps_per_second
    return math.floor(value / step) * step


def _passes(flight: "_Flight", axes: dict[str, Axis],
            bounds: dict[str, tuple[float, float]], refinements: int,
            result: SearchResult, report, pool,
            by_time: bool, screen: bool = True) -> dict[str, Axis] | None:
    """Sweep the grid, refine about the best node, sweep again.

    `by_time` says what the next pass is centred on: the cheapest node within
    reach of the orbit, or simply the closest to it. The first is what the
    search is for; the second is the fallback when the first has run out of
    family before it ran out of orbit.
    """
    # the first pass has nowhere to look but the whole window; each one after
    # it starts in the neighbourhood of the cut-off the pass before settled on
    bracket = flight.window
    for _ in range(refinements + 1):
        result.pass_number += 1
        solved = _sweep(flight, axes, bracket, result, report, pool, screen)
        if not solved:
            return None
        result.best = _best_so_far(result, solved)
        centre = (_refine_about(solved, axes, result) if by_time
                  else min(solved, key=lambda candidate: candidate.miss))
        axes = _refine(axes, centre.shape, bounds)
        bracket = (max(flight.window[0], centre.cutoff_time - NEIGHBOURING_CUT_OFF),
                   min(flight.window[1], centre.cutoff_time + NEIGHBOURING_CUT_OFF))
    return axes


class _Flight:
    """Everything one node of the grid needs, held in one place.

    A node is answered in three steps: the family builds the programme, the
    altitude integral says whether it is aiming anywhere near the orbit, and -
    only then - the cut-off that closes the orbit is solved for by integrating
    the trajectory a handful of times.

    Nothing here is written to. What a node came to is returned as a `Node` and
    counted by the caller, because the node may have been answered in another
    process, where anything written to would be written to a copy.
    """

    def __init__(self, vehicle, family, t1, target_altitude, window,
                 latitude_deg, azimuth_deg, steps_per_second, circular_tolerance,
                 screen=True):
        self.vehicle, self.family, self.t1 = vehicle, family, t1
        self.target_altitude, self.window = target_altitude, window
        self.latitude_deg, self.azimuth_deg = latitude_deg, azimuth_deg
        self.steps_per_second = steps_per_second
        self.circular_tolerance = circular_tolerance
        # whether the altitude integral is allowed to reject a node unflown.
        # The band it rejects on is measured, not derived, so the second run of
        # the grid turns it off: whatever the first run failed to find, it will
        # not have been for want of trying a shape the integral mistrusted
        self.screen = screen
        self._flights = 0

    def at(self, shape: dict[str, float], bracket: tuple[float, float],
           screen: bool | None = None) -> Node:
        """Answer one node of the grid, flying as little as it takes.

        `bracket` is where the cut-off is looked for first - the neighbourhood
        of the one the last pass settled on - with the whole window behind it.
        `screen` overrides what this flight was built with, which is how the
        second run of the grid drops the altitude integral: the flight itself
        was handed to the worker processes when they started and cannot be
        changed under them, so the pass says what it wants with every node.
        """
        self._flights = 0
        early, late = self.window
        try:
            soonest = analytic_altitude(
                self.vehicle, self.family.build(self.t1, early, shape))
            latest = analytic_altitude(
                self.vehicle, self.family.build(self.t1, late, shape))
        except (ValueError, np.linalg.LinAlgError):
            # the family refuses this shape outright: phases out of order, a
            # share the quartic is not a turn over, a tangent through its pole,
            # or three angles the coefficients cannot be recovered from - which
            # a range narrowed by hand can ask for
            return self._node(shape, 'refused', None)

        # the screen. The altitude the integral reports rises with the cut-off,
        # so these two bound what the shape can reach anywhere in the window,
        # and the band the integral is known to read high by turns them into a
        # bound on the flight. A shape that cannot reach the target inside the
        # window is dropped without a trajectory
        screening = self.screen if screen is None else screen
        if screening and not (
                soonest / ALTITUDE_RATIO_HIGH <= self.target_altitude
                <= latest / ALTITUDE_RATIO_LOW):
            return self._node(shape, 'screened', None)

        return self._close_the_orbit(shape, bracket)

    def _node(self, shape, outcome: str, candidate: Candidate | None) -> Node:
        return Node(shape, outcome, candidate, self._flights)

    def _close_the_orbit(self, shape: dict[str, float],
                         bracket: tuple[float, float]) -> Node:
        """Solve for the cut-off that leaves the vehicle on a circular orbit.

        Tried first in the neighbourhood of the cut-off the last pass settled
        on, and over the whole window only when that neighbourhood turns out
        not to hold the root. Neighbouring shapes cut off at neighbouring
        instants, so the narrow bracket almost always holds it and is worth
        half the trajectories the wide one takes.
        """
        brackets = [bracket]
        if bracket != self.window:
            brackets.append(self.window)
        for low, high in brackets:
            solved = self._solve_between(low, high, shape)
            if solved is None:
                continue
            if not math.isfinite(solved.miss):
                # a cut-off was found and what it closes is not an orbit: the
                # perigee is under the surface, or the trajectory is not closed
                # at all. Its own outcome, so that every node has exactly one
                return self._node(shape, 'no_orbit', None)
            return self._node(shape, 'closed', solved)
        return self._node(shape, 'no_cut_off', None)

    def _solve_between(self, low: float, high: float,
                       shape: dict[str, float]) -> Candidate | None:
        """The cut-off inside one bracket that circularises the orbit.

        The quantity driven to zero is the semi-major axis less the radius at
        cut-off. On a circular orbit the two are the same; a vehicle cut off
        too early falls short of the axis and coasts down from an apogee, one
        cut off too late overshoots it and climbs away from a perigee. It rises
        with the cut-off time and with nothing else - some tens of kilometres
        for every second of burn - so a bracket that straddles zero holds
        exactly one root, and the regula falsi finds it in the Illinois form,
        which halves the value at whichever end has been kept and so stops the
        method creeping up on the root from one side.
        """
        at_low, at_high = self._measure(low, shape), self._measure(high, shape)
        if at_low is None or at_high is None \
                or at_low[0] > 0.0 or at_high[0] < 0.0:
            return None

        (below, under), (above, over) = at_low, at_high
        closest = under if abs(below) < abs(above) else over

        for _ in range(CUT_OFF_PASSES):
            if abs(closest.residual) <= self.circular_tolerance \
                    or high - low <= 1e-9:
                break
            middle = low + (high - low) * (-below) / (above - below)
            if not low < middle < high:
                # a residual that is not finite takes the interpolation with
                # it; halving the bracket is what there is left to do
                middle = 0.5 * (low + high)

            measured = self._measure(middle, shape)
            if measured is None:
                # the trial cannot be flown at all, which says nothing about
                # which side of it the root lies. Rather than give up the
                # bracket - which still holds a root, both of its ends having
                # flown - the probe is pulled back towards the end that flew
                # earliest and the pass tried again on what is left of the
                # iterations
                pulled = 0.5 * (low + middle)
                if pulled <= low:
                    return None
                measured = self._measure(pulled, shape)
                if measured is None:
                    continue
                middle = pulled
            residual, candidate = measured
            if abs(residual) < abs(closest.residual):
                closest = candidate
            if residual < 0.0:
                low, below, above = middle, residual, above * 0.5
            else:
                high, above, below = middle, residual, below * 0.5

        # the loop can also run out, or the bracket collapse on a root the
        # trajectories cannot resolve. What comes back then is the closest the
        # solve got, which is not a circular orbit and must not be ranked as
        # one - so it is no answer at all
        if abs(closest.residual) > self.circular_tolerance:
            return None
        return closest

    def _measure(self, end: float,
                 shape: dict[str, float]) -> tuple[float, "Candidate"] | None:
        """Fly one set: how far its orbit is from circular, and what it reached."""
        flown = self._fly(end, shape)
        if flown is None:
            return None
        telemetry, mission = flown
        orbit = mission.orbit
        parameters = self.family.parameters(self.t1, end, shape)
        if not orbit.is_closed:
            return (math.inf, Candidate(shape, parameters, end, orbit, math.inf,
                                        residual=math.inf))
        # the radius at cut-off, read off the last row before it. Every
        # programme here leaves the vehicle in the horizon, so the altitude
        # there is all but flat in time and the part of a step between that row
        # and the cut-off itself is worth metres rather than kilometres
        radius = telemetry.radius[telemetry.at(end)]
        residual = float(orbit.semi_major_axis - radius)

        if not orbit.is_orbit:
            return (residual, Candidate(shape, parameters, end, orbit,
                                        math.inf, residual=residual))
        budget = velocity_budget(telemetry, mission.omega)
        miss = max(abs(orbit.perigee_altitude - self.target_altitude),
                   abs(orbit.apogee_altitude - self.target_altitude))
        return (residual, Candidate(shape, parameters, end, orbit, miss,
                                    budget.gravity, budget.aerodynamic,
                                    budget.steering, residual,
                                    *_terminal_state(telemetry, end),
                                    *_demands(telemetry, end)))

    def _duration(self, end: float) -> float:
        """How long to fly for: to the first whole step at or past the cut-off.

        No further, because the orbit is read off the end of the flight and
        every second of coast past the cut-off is a second in which something
        could still act on it. Above the air nothing does - which is why five
        seconds of coast made no difference to any set here - but the target is
        the caller's to choose, and a step is a bound that holds whatever they
        chose.

        Not less either: `Mission` rounds the length of a flight to a whole
        number of steps, and one rounded back below the cut-off would leave the
        state at the end of it from before the engines stopped.
        """
        return math.ceil(end * self.steps_per_second) / self.steps_per_second

    def _fly(self, end: float,
             shape: dict[str, float]) -> tuple[Telemetry, Mission] | None:
        """Integrate one trajectory, or nothing if it cannot be flown."""
        try:
            mission = Mission(
                vehicle=self.vehicle,
                pitch_programme=self.family.build(self.t1, end, shape),
                cutoff=CutoffAtTime(end), target_altitude=self.target_altitude,
                # `end` is an arbitrary instant and a programme is tabulated on
                # a tenth-of-a-second grid, so the programme ends on the last
                # grid point at or before it and the remainder is flown on the
                # attitude reached. That is how every set in the catalogue was
                # solved and how every one of them is flown back, so the set
                # reported here reproduces itself from its own specification -
                # which `tests/test_search.py` checks
                duration=self._duration(end),
                steps_per_second=self.steps_per_second,
                latitude_deg=self.latitude_deg, azimuth_deg=self.azimuth_deg)
            # counted before it is flown, not after: one that leaves the model
            # does so part of the way through and has cost what it cost
            self._flights += 1
            telemetry = mission.run()
        except ValueError:
            # the set cannot be flown by this vehicle: it runs out of speed
            # against its own programme, or the trajectory leaves the model
            return None
        return telemetry, mission


# The flight a worker process answers its nodes with, set once when the process
# starts so that the vehicle and the family are handed over once rather than
# with every node.
_WORKER: _Flight | None = None


def _begin(flight: _Flight) -> None:
    global _WORKER
    _WORKER = flight


def _answer(work: tuple[dict[str, float], tuple[float, float], bool]) -> Node:
    shape, bracket, screen = work
    return _WORKER.at(shape, bracket, screen)


def _terminal_state(telemetry: Telemetry,
                    end: float) -> tuple[float, float, float]:
    """Altitude, inertial speed and flight-path angle at the cut-off.

    Interpolated between the rows either side of it, because the cut-off falls
    between two of them and the rows are what was recorded. At the forty-odd
    metres per second squared a stage is pulling by then, the row before the
    cut-off is several metres per second short of it - which is nothing to the
    orbit, read off the state at cut-off itself, and is the whole of what these
    three are: the state the terminal conditions are conditions on.
    """
    row = telemetry.at(end)
    columns = (telemetry.altitude, telemetry.inertial_speed,
               telemetry.flight_path_angle)
    if row < 1 or telemetry.t[row] >= end:
        return tuple(float(column[row]) for column in columns)
    # forward from the two rows before the cut-off, not across it. The engines
    # stop at `end`, so a line drawn to the row after it is half powered and
    # half coast and lands between the two; the rows behind it are both under
    # thrust, and the step to carry forward is a fraction of one
    weight = (end - telemetry.t[row]) / (telemetry.t[row] - telemetry.t[row - 1])
    return tuple(float(column[row] + (column[row] - column[row - 1]) * weight)
                 for column in columns)


def _demands(telemetry: Telemetry, end: float) -> tuple[float, float]:
    """What the ascent asked of the airframe and of the guidance.

    The dynamic pressure is the peak over the whole climb, which is what an
    airframe is designed against. The steering demand is the sine of the thrust
    deflection the programme calls for, taken over the powered flight under
    guidance; where it passes one there is no such deflection and the vehicle
    cannot hold the programme, so it says how far the steering loss beside it
    is a measurement at all.

    Both are the height of a curve sampled at the step the flight was
    integrated at, so both are read off the parabola through the largest row
    and its neighbours rather than off that row: a peak between two rows is
    read low otherwise, and at a coarse step by enough to matter to a limit
    imposed on either figure.
    """
    up_to = telemetry.at(end) + 1
    demand = telemetry.steering_demand[:up_to][telemetry.thrust[:up_to] > 0.0]
    return (_peak(telemetry.dynamic_pressure[:up_to]),
            _peak(np.abs(demand)) if len(demand) else 0.0)


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


def _sweep(flight: _Flight, axes: dict[str, Axis], bracket: tuple[float, float],
           result: SearchResult, report, pool,
           screen: bool = True) -> list[Candidate]:
    """One pass over the grid: every node screened, the survivors solved.

    The nodes are independent, so they are answered over a pool of processes
    where there is one - in the order of the grid, so that a search returns the
    same set however many of them there are. A set that asks more of the
    airframe or of the guidance than is allowed is put aside here rather than
    ranked: it is not a dearer answer, it is not an answer.
    """
    shapes = list(_nodes(axes))
    result.pass_nodes = len(shapes)
    result.pass_node = 0
    limit = result.max_dynamic_pressure
    demand = result.max_steering_demand

    answers = ((flight.at(shape, bracket, screen) for shape in shapes)
               if pool is None
               else pool.map(_answer,
                             [(shape, bracket, screen) for shape in shapes],
                             chunksize=1))

    solved = []
    for node in answers:
        result.nodes += 1
        result.pass_node += 1
        result.flown += node.flights
        setattr(result, node.outcome, getattr(result, node.outcome) + 1)
        if node.candidate is not None:
            if limit is not None and node.candidate.peak_dynamic_pressure > limit:
                result.over_pressure += 1
            elif demand is not None \
                    and node.candidate.peak_steering_demand > demand:
                result.over_demand += 1
            else:
                solved.append(node.candidate)
                # kept whether or not it meets the tolerance: the report is a
                # table of what was found, best first, and a set that missed by
                # a kilometre is the most useful row on it when nothing met it
                result.found.append(node.candidate)
        if report is not None:
            report(result)
    return solved


def _best_so_far(result: SearchResult, solved: list[Candidate]) -> Candidate:
    """The best set found by the whole search up to and including this pass.

    By the same order the table of sets comes back in, so that the set reported
    is the one at the head of it - which is what `_order` is for. Nothing has
    reached the orbit at the start of a search, and then the closest is what
    there is to show and `reaches_orbit` is what says it does not count.
    """
    seen = solved if result.best is None else [*solved, result.best]
    return min(seen, key=lambda candidate: _order(candidate, result))


def _rank(candidate: Candidate, result: SearchResult) -> tuple[float, float]:
    """The order sets that reach the orbit are preferred in.

    The smaller velocity budget wins, or the earlier cut-off where that is what
    was asked for - and the cut-off only down to the step the trajectory was
    integrated at. Two cut-offs less than one step apart are the same ascent as
    far as this model can resolve, and treating them as different would trade a
    hundredth of a second of ascent for half a kilometre of orbit. Inside a
    step, then, the closer orbit wins. The budget needs no such rounding: it is
    resolved to a hundredth of a metre per second and sets differ in it by
    tens.
    """
    if result.criterion == 'time':
        step = 1.0 / result.steps_per_second
        return (math.floor(candidate.cutoff_time / step), candidate.miss)
    return (CRITERIA[result.criterion](candidate, result), candidate.miss)


def _refine_about(solved: list[Candidate], axes: dict[str, Axis],
                  result: SearchResult) -> Candidate:
    """The node the next pass is centred on: the cheapest within reach of it.

    Every node here already sits on a circular orbit - that is what its cut-off
    was solved for - but at its own altitude rather than at the target, so
    ranking them by the criterion alone would prefer whichever of them stopped
    short. Two corrections make the ranking mean something.

    The first is the altitude the node fell short by, priced in whatever the
    criterion is. Across the nodes of one pass the altitude reached and what it
    cost lie on a line - both are readings of the same energy - and the line is
    measured off the pass itself, so each node can be read against it to give
    what it would cost at the target.

    The second is which nodes are allowed to compete at all. A node counts as
    within reach when its miss is no larger than the altitude one step of the
    grid is worth - because a step is exactly how far the next pass can walk
    it. That is what stops a coarse pass from settling for whichever node
    happened to land on the orbit while a much quicker one sat a step away, and
    what makes the same rule tighten to the tolerance itself as the grid
    closes in.
    """
    reached = [0.5 * (c.orbit.perigee_altitude + c.orbit.apogee_altitude)
               for c in solved]
    steps = max(1, min(axis.nodes for axis in axes.values()) - 1)
    band = max(result.tolerance, (max(reached) - min(reached)) / steps)
    within = [(c, h) for c, h in zip(solved, reached)
              if c.miss <= band]
    if not within:
        return min(solved, key=lambda candidate: candidate.miss)

    if result.criterion == 'orbit':
        # nothing to project: the criterion is the distance to the target, and
        # what a node would score at the target is nought for all of them. The
        # closest node by the ranking, which is not quite the closest by the
        # larger of its two errors
        return min(solved, key=lambda candidate: _rank(candidate, result))
    value = CRITERIA[result.criterion]
    slope = _cost_per_metre([value(candidate, result) for candidate in solved],
                            reached)
    if slope is None:
        return min(within, key=lambda pair: _rank(pair[0], result))[0]
    return min(within, key=lambda pair: value(pair[0], result)
               + (result.target_altitude - pair[1]) * slope)[0]


def _cost_per_metre(values: list[float],
                    reached: list[float]) -> float | None:
    """What a metre more of orbit costs, in whatever the criterion is.

    Measured off the pass rather than assumed: it is a property of the vehicle
    and of the orbit, and it is what lets a node that stopped short be compared
    with one that overshot. None when the pass has closed in so far that its
    nodes no longer spread far enough to measure it, or when the fit comes out
    the wrong way round, and then there is nothing left to correct.
    """
    if len(values) < 3 or max(reached) - min(reached) < 1.0:
        return None
    slope = float(np.polyfit(reached, values, 1)[0])
    return slope if slope > 0.0 else None


def _nodes(axes: dict[str, Axis]):
    """Every combination of the grid, one shape at a time."""
    names = list(axes)
    grids = [np.linspace(axes[name].low, axes[name].high, axes[name].nodes)
             for name in names]
    for point in product(*grids):
        yield {name: float(value) for name, value in zip(names, point)}


def _step(axis: Axis) -> float:
    """The distance between two neighbouring nodes of an axis."""
    return (axis.high - axis.low) / max(axis.nodes - 1, 1)


def _refine(axes: dict[str, Axis], centre: dict[str, float],
            bounds: dict[str, tuple[float, float]]) -> dict[str, Axis]:
    """A grid one step wide about the best node, and narrower than it was.

    Held inside the range the family gave, which for most of these axes is
    where the programme stops being a turn at all - a share of the speed
    outside the range the quartic is monotone over, a turn that ends after the
    burn does. A node that comes out on one of those bounds is reported rather
    than chased past it.
    """
    refined = {}
    for name, axis in axes.items():
        step = _step(axis)
        low, high = bounds[name]
        # five nodes over two old steps, whatever the pass before had: fewer
        # would span the same two steps without shortening them, and the pass
        # would resolve the shape no further than the one before it
        refined[name] = Axis(max(low, centre[name] - step),
                             min(high, centre[name] + step), REFINED_NODES)
    return refined


def _count(axes: dict[str, Axis]) -> int:
    """How many nodes one pass over these axes visits."""
    return math.prod(axis.nodes for axis in axes.values())


def _planned_nodes(axes: dict[str, Axis], refinements: int) -> int:
    """How many nodes the whole search will visit, known before it starts."""
    refined = {name: Axis(axis.low, axis.high, REFINED_NODES)
               for name, axis in axes.items()}
    return _count(axes) + refinements * _count(refined)


def _coarsen(axes: dict[str, Axis], factor: float) -> dict[str, Axis]:
    """The same axes with the nodes along each scaled, for a quicker look."""
    if factor == 1.0:
        return axes
    return {name: Axis(axis.low, axis.high,
                       max(3, int(round(axis.nodes * factor))))
            for name, axis in axes.items()}
