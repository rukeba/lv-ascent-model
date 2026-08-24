"""Grid search for the parameters of a pitch programme.

Ask for a vehicle, a circular orbit and one of the three programme families,
and this returns the parameters that fly that vehicle into that orbit in the
shortest time. The orbit is the constraint - the perigee and the apogee both
within a tolerance of the target altitude, which is what makes it circular and
at the right height - and the ascent time is what is minimised among the sets
that meet it.

The grid runs over the shape of the turn, and only over the shape. The cut-off
is not one of its axes: it is what the terminal condition on the speed fixes,
and it is solved for at every node instead. That division is not a convenience
- it is how the two conditions of a circular orbit divide between the
parameters. The speed at cut-off answers to the cut-off time and to almost
nothing else, at some tens of metres per second for each second of burn, so a
grid fine enough to resolve it along that axis would be enormous and a grid
coarse enough to afford would resolve nothing. The altitude reached, on the
other hand, is what the shape of the turn decides. So the cut-off is solved and
the shape is searched.

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

Which node a pass refines about is not simply the quickest one that reached the
orbit. At the resolution of an early pass, whether a node lands on the orbit at
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
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from itertools import product

import numpy as np

from .constants import EARTH_RADIUS
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
# reported as unable to reach an orbit it can reach. The margin costs some of
# the screening - a first pass of the velocity share still goes 83 per cent
# unflown against 87 at the measured band, one of the five-phase 16 against 42.
ALTITUDE_RATIO_LOW = 0.95
ALTITUDE_RATIO_HIGH = 1.40

# Both the perigee and the apogee have to land within this of the target for a
# set to count as reaching the orbit, m
TOLERANCE = 500.0

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
    # what the ascent asks of the airframe and of the guidance. Neither enters
    # the ranking unless the caller sets a limit on the first, but a quicker
    # ascent is a flatter one and both are what it is paid for
    peak_dynamic_pressure: float = 0.0
    peak_steering_demand: float = 0.0

    @property
    def total_loss(self) -> float:
        return self.gravity_loss + self.aerodynamic_loss + self.steering_loss


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
    max_dynamic_pressure: float | None = None
    # processes the nodes of a pass were divided over
    workers: int = 1
    # axes whose best value came out on a bound of the grid, where a better set
    # may lie just outside
    on_edge: tuple[str, ...] = ()

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


@dataclass(frozen=True)
class Axis:
    low: float
    high: float
    nodes: int


class Family:
    """A pitch programme with the shape of its turn laid out as a grid.

    Every family here ends its programme at cut-off, so the cut-off is a
    parameter of all three - and none of them has it as an axis. What is left
    is the shape: one number for the five-phase turn, two for each of the
    others. The vertical rise is not an axis either: it is a construction
    choice, a few seconds long, and no terminal condition has a lever on it.
    """
    name: str

    def axes(self) -> dict[str, Axis]:
        raise NotImplementedError

    def build(self, t1: float, end: float,
              shape: dict[str, float]) -> PitchProgramme:
        raise NotImplementedError

    def parameters(self, t1: float, end: float,
                   shape: dict[str, float]) -> dict[str, float]:
        raise NotImplementedError


class FivePhase(Family):
    """The turn built from constant angular accelerations.

    k2 - the share of the turn spent building the pitch rate up - is held
    rather than searched. Driving it towards zero always pays, because a step
    in pitch rate costs nothing to a model that prices only the angle, and what
    it buys is a phase no vehicle could fly. So it is a design choice, as it is
    in the catalogue, and the single axis left is the share spent at a constant
    rate.
    """
    name = 'five-phase'

    def __init__(self, k2: float = 0.05) -> None:
        self.k2 = k2

    def axes(self):
        # the family's own range, less a thousandth at the top: k2 + k3 = 1
        # leaves the fourth phase no time to arrest the pitch rate in, and the
        # rate it would need to is divided by that nothing
        return {'k3': Axis(0.0, 1.0 - self.k2 - 1e-3, 19)}

    def build(self, t1, end, shape):
        return FivePhaseProgramme(t1=t1, t4=end, k2=self.k2, k3=shape['k3'])

    def parameters(self, t1, end, shape):
        return {'type': self.name, 't1': t1, 't4': end,
                'k2': self.k2, 'k3': shape['k3']}


class VelocityShare(Family):
    """The turn set by the share of the speed that stays vertical.

    The turn ends at `turn` times the cut-off and the quartic's fullness is s.
    Taking the end of the turn as a share of the cut-off rather than as a time
    of its own keeps every node inside the family's own rule that the turn end
    before the burn does, wherever the cut-off is solved to.
    """
    name = 'velocity-share'

    def axes(self):
        return {'turn': Axis(0.5, 1.0, 11), 's': Axis(-3.0, 3.0, 13)}

    def build(self, t1, end, shape):
        return VelocityShareProgramme(t1=t1, tf=shape['turn'] * end, te=end,
                                      s=shape['s'])

    def parameters(self, t1, end, shape):
        return {'type': self.name, 't1': t1, 'tf': shape['turn'] * end,
                'te': end, 's': shape['s']}


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

    def axes(self):
        return {'start': Axis(78.0, 89.6, 12), 'middle': Axis(2.0, 60.0, 13)}

    def build(self, t1, end, shape):
        a, b, c = self._coefficients(t1, end, shape)
        return BilinearTangentProgramme(t1=t1, a=a, b=b, c=c, te=end)

    def parameters(self, t1, end, shape):
        a, b, c = self._coefficients(t1, end, shape)
        return {'type': self.name, 't1': t1, 'a': a, 'b': b, 'c': c, 'te': end}

    @staticmethod
    def _coefficients(t1, end, shape):
        return bilinear_coefficients(t1, shape['start'], 0.5 * (t1 + end),
                                     shape['middle'], end, 0.0)


FAMILIES = {family.name: family for family in
            (FivePhase, VelocityShare, BilinearTangent)}


# --- the search -----------------------------------------------------------


def search(vehicle: LaunchVehicle, target_altitude: float, programme: str,
           *, latitude_deg: float = 0.0, azimuth_deg: float = 90.0,
           t1: float = VERTICAL_RISE, k2: float = 0.05,
           tolerance: float = TOLERANCE, refinements: int = 10,
           max_dynamic_pressure: float | None = None,
           coarseness: float = 1.0, steps_per_second: float = 10,
           workers: int | None = None, report=None) -> SearchResult:
    """Parameters that fly `vehicle` into a circular orbit at `target_altitude`.

    Among the sets whose perigee and apogee both land within `tolerance` of the
    target, the one that reaches cut-off soonest.

    The grid is run twice if the first run does not reach the orbit: once
    preferring the quickest set within reach, once preferring the closest.

    `max_dynamic_pressure` puts the airframe into the constraint: a set that
    asks more of it than that is put aside however quick it is. Left out, the
    peak is reported and nothing more, which is how the rest of the model
    treats the figure.

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
    family = FivePhase(k2) if programme == 'five-phase' else FAMILIES[programme]()

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
        max_dynamic_pressure=max_dynamic_pressure)

    axes = _coarsen(family.axes(), coarseness)
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
        if not result.reaches_orbit:
            # minimising the ascent has led into a corner of the family where
            # the orbit cannot be reached at all - which happens where a
            # vehicle is near its limit, the quickest sets of a pass lying just
            # outside what it can still close. Run the grid again for the orbit
            # alone
            result.attempts = 2
            result.passes += refinements + 1
            result.planned_nodes += _planned_nodes(axes, refinements)
            settled = _passes(flight, _coarsen(family.axes(), coarseness),
                              bounds, refinements, result, report, pool,
                              by_time=False) or settled
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
    return result


def _passes(flight: "_Flight", axes: dict[str, Axis],
            bounds: dict[str, tuple[float, float]], refinements: int,
            result: SearchResult, report, pool,
            by_time: bool) -> dict[str, Axis] | None:
    """Sweep the grid, refine about the best node, sweep again.

    `by_time` says what the next pass is centred on: the quickest node within
    reach of the orbit, or simply the closest to it. The first is what the
    search is for; the second is the fallback when the first has run out of
    family before it ran out of orbit.
    """
    # the first pass has nowhere to look but the whole window; each one after
    # it starts in the neighbourhood of the cut-off the pass before settled on
    bracket = flight.window
    for _ in range(refinements + 1):
        result.pass_number += 1
        solved = _sweep(flight, axes, bracket, result, report, pool)
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

    def at(self, shape: dict[str, float],
           bracket: tuple[float, float]) -> Node:
        """Answer one node of the grid, flying as little as it takes.

        `bracket` is where the cut-off is looked for first - the neighbourhood
        of the one the last pass settled on - with the whole window behind it.
        """
        self._flights = 0
        early, late = self.window
        try:
            soonest = analytic_altitude(
                self.vehicle, self.family.build(self.t1, early, shape))
            latest = analytic_altitude(
                self.vehicle, self.family.build(self.t1, late, shape))
        except ValueError:
            # the family refuses this shape outright: phases out of order, a
            # share the quartic is not a turn over, a tangent through its pole
            return self._node(shape, 'refused', None)

        # the screen. The altitude the integral reports rises with the cut-off,
        # so these two bound what the shape can reach anywhere in the window,
        # and the band the integral is known to read high by turns them into a
        # bound on the flight. A shape that cannot reach the target inside the
        # window is dropped without a trajectory
        if self.screen and not (
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


def _answer(work: tuple[dict[str, float], tuple[float, float]]) -> Node:
    shape, bracket = work
    return _WORKER.at(shape, bracket)


def _demands(telemetry: Telemetry, end: float) -> tuple[float, float]:
    """What the ascent asked of the airframe and of the guidance.

    The dynamic pressure is the peak over the whole climb, which is what an
    airframe is designed against. The steering demand is the sine of the thrust
    deflection the programme calls for, taken over the powered flight under
    guidance; where it passes one there is no such deflection and the vehicle
    cannot hold the programme, so it says how far the steering loss beside it
    is a measurement at all.
    """
    up_to = telemetry.at(end) + 1
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


def _sweep(flight: _Flight, axes: dict[str, Axis], bracket: tuple[float, float],
           result: SearchResult, report, pool) -> list[Candidate]:
    """One pass over the grid: every node screened, the survivors solved.

    The nodes are independent, so they are answered over a pool of processes
    where there is one - in the order of the grid, so that a search returns the
    same set however many of them there are. A set that asks more of the
    airframe than the caller allowed is put aside here rather than ranked: it
    is not a slower answer, it is not an answer.
    """
    shapes = list(_nodes(axes))
    result.pass_nodes = len(shapes)
    result.pass_node = 0
    limit = result.max_dynamic_pressure

    answers = ((flight.at(shape, bracket) for shape in shapes) if pool is None
               else pool.map(_answer, [(shape, bracket) for shape in shapes],
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
            else:
                solved.append(node.candidate)
        if report is not None:
            report(result)
    return solved


def _best_so_far(result: SearchResult, solved: list[Candidate]) -> Candidate:
    """The best set found by the whole search up to and including this pass."""
    seen = solved if result.best is None else [*solved, result.best]
    reaching = [candidate for candidate in seen
                if candidate.miss <= result.tolerance]
    if reaching:
        return min(reaching, key=lambda candidate: _rank(candidate, result))
    # nothing has reached the orbit yet: the closest is what there is to show,
    # and `reaches_orbit` is what says it does not count
    return min(seen, key=lambda candidate: candidate.miss)


def _rank(candidate: Candidate, result: SearchResult) -> tuple[float, float]:
    """The order sets that reach the orbit are preferred in.

    The earlier cut-off wins - the ascent time is the thing being minimised -
    but only down to the step the trajectory was integrated at. Two cut-offs
    less than one step apart are the same ascent as far as this model can
    resolve, and treating them as different would trade a hundredth of a second
    of ascent for half a kilometre of orbit. Inside a step, then, the closer
    orbit wins.
    """
    step = 1.0 / result.steps_per_second
    return (math.floor(candidate.cutoff_time / step), candidate.miss)


def _refine_about(solved: list[Candidate], axes: dict[str, Axis],
                  result: SearchResult) -> Candidate:
    """The node the next pass is centred on: the quickest within reach of it.

    Every node here already sits on a circular orbit - that is what its cut-off
    was solved for - but at its own altitude rather than at the target, so
    ranking them by the cut-off alone would prefer whichever of them stopped
    short. Two corrections make the ranking mean something.

    The first is the altitude the node fell short by, priced in seconds. Across
    the nodes of one pass the altitude reached and the instant of cut-off are
    two readings of the same energy and lie on a line; the line is measured off
    the pass itself, and each node is read against it to give the cut-off it
    would need to reach the target.

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

    slope = _altitude_per_second(solved, reached)
    if slope is None:
        return min(within, key=lambda pair: _rank(pair[0], result))[0]
    return min(within, key=lambda pair: pair[0].cutoff_time
               + (result.target_altitude - pair[1]) / slope)[0]


def _altitude_per_second(solved: list[Candidate],
                         reached: list[float]) -> float | None:
    """How much higher an orbit a second more of burn buys, m/s.

    Measured off the pass rather than assumed: it is a property of the vehicle
    and of the orbit, some fifteen to twenty kilometres a second for the ones
    here. None when the pass has closed in so far that its nodes no longer
    spread far enough to measure it, and then there is nothing left to correct.
    """
    times = [candidate.cutoff_time for candidate in solved]
    if len(solved) < 3 or max(times) - min(times) < 1e-3:
        return None
    slope = float(np.polyfit(times, reached, 1)[0])
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
