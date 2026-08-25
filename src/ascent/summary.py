"""The console summary of a flight: the same figures the HTML report shows.

Everything here is read back from the recorded telemetry, so the summary
describes the run that was actually flown rather than what was asked for. The
rows are built once, as blocks, and then either printed as lines or laid out
as cards by the report: the two cannot disagree about a figure.
"""

import math
from dataclasses import dataclass

import numpy as np

from .constants import STANDARD_GRAVITY, circular_velocity
from .losses import velocity_budget
from .mission import Mission
from .telemetry import Telemetry
from .vehicle import LaunchVehicle

LABEL_WIDTH = 24


@dataclass(frozen=True)
class Block:
    """One titled group of label-and-value rows."""
    title: str
    rows: tuple[tuple[str, str], ...]


def heading(mission: Mission) -> str:
    return f'{mission.vehicle.name} to {mission.target_altitude / 1000:g} km'


def summary_blocks(mission: Mission, telemetry: Telemetry) -> list[Block]:
    """Everything the summary says, before it is turned into text."""
    budget = velocity_budget(telemetry, mission.omega)
    cut_off = telemetry.at(budget.burnout_time)
    vehicle = mission.vehicle
    first_stage = vehicle.stages[0]
    rotation = mission.omega * telemetry.radius[0]

    acceleration = (telemetry.thrust - telemetry.drag) / telemetry.mass
    peak_q, peak_a = int(np.argmax(telemetry.dynamic_pressure)), int(np.argmax(acceleration))
    design_q = vehicle.design_dynamic_pressure

    blocks = [Block('setup', (
        *_site(mission, rotation),
        ('pitch programme', mission.pitch_programme.describe()),
        ('engine cut-off', mission.cutoff.describe()),
        ('integration', f'rk4, {mission.steps_per_second:g} steps/s, '
                        f'{mission.duration:g} s of flight'),
        ('lift-off mass', f'{vehicle.lift_off_mass:,.0f} kg'),
        ('payload', f'{vehicle.payload_mass:,.0f} kg'),
        ('thrust to weight',
         f'{first_stage.thrust_sea_level / (vehicle.lift_off_mass * STANDARD_GRAVITY):.2f}'),
    )), Block('flight', (
        ('max dynamic pressure',
         f'{telemetry.dynamic_pressure[peak_q] / 1000:.1f} kPa at t = {telemetry.t[peak_q]:.1f} s, '
         f'h = {telemetry.altitude[peak_q] / 1000:.1f} km'
         + (f' (design {design_q / 1000:g} kPa)' if design_q else '')),
        ('max acceleration',
         f'{acceleration[peak_a] / STANDARD_GRAVITY:.2f} g at t = {telemetry.t[peak_a]:.1f} s'),
        ('stage separation',
         ', '.join(f'{s.ignition_time:g} s' for s in vehicle.stages[1:]) or 'none'),
        ('engine cut-off', f'{budget.burnout_time:.1f} s'),
        ('target altitude reached',
         _first_time(telemetry, telemetry.altitude, mission.target_altitude)),
        ('orbital speed reached',
         _first_time(telemetry, telemetry.inertial_speed,
                     circular_velocity(mission.target_altitude))),
    )), Block('at cut-off', (
        ('altitude', f'{telemetry.altitude[cut_off] / 1000:.2f} km'),
        ('speed, relative', f'{telemetry.speed[cut_off]:.1f} m/s'),
        ('speed, inertial', f'{telemetry.inertial_speed[cut_off]:.1f} m/s'),
        ('flight-path angle', f'{telemetry.flight_path_angle[cut_off]:.3f} deg'),
        ('downrange angle', f'{telemetry.polar_angle[cut_off]:.2f} deg'),
        ('mass', f'{telemetry.mass[cut_off]:,.0f} kg'),
        ('propellant left', _propellant_left(vehicle, telemetry, cut_off)),
    ))]

    orbit = mission.orbit
    if orbit.is_closed:
        blocks.append(Block('orbit', (
            ('perigee', f'{orbit.perigee_altitude / 1000:.2f} km'),
            ('apogee', f'{orbit.apogee_altitude / 1000:.2f} km'),
            ('eccentricity', f'{orbit.eccentricity:.5f}'),
            ('period', f'{orbit.period / 60:.1f} min'),
            ('circularisation dv', f'{orbit.circularisation_dv:.1f} m/s'),
            ('closes an orbit',
             'yes' if orbit.is_orbit else 'no, perigee below the surface'),
        )))
    else:
        blocks.append(Block('orbit', (
            ('eccentricity', f'{orbit.eccentricity:.5f} - not a closed orbit'),)))

    blocks.append(Block('velocity budget', (
        ('gravity loss', f'{budget.gravity:.1f} m/s'),
        ('aerodynamic loss', f'{budget.aerodynamic:.1f} m/s'),
        ('steering loss', f'{budget.steering:.1f} m/s'),
        ('total', f'{budget.total:.1f} m/s'),
        ('steering demand', _demand(telemetry, mission.pitch_programme.end_time)),
        ('control effort',
         f'{telemetry.control_effort[cut_off]:,.0f} m^2/s^3, integrated over the '
         f'powered flight off the demand before it is clamped'),
    )))
    return blocks


def summarise(mission: Mission, telemetry: Telemetry) -> str:
    lines = [heading(mission)]
    for block in summary_blocks(mission, telemetry):
        _block(lines, block.title, list(block.rows))
    return '\n'.join(lines)


def _site(mission: Mission, rotation: float) -> tuple[tuple[str, str], ...]:
    """Where it was launched from: the pad by name, if the file gave one.

    Two rows when it did, so that neither the name nor the geometry has to be
    read out of the middle of a long line.
    """
    geometry = (f'{mission.latitude_deg:g} deg latitude, '
                f'azimuth {mission.azimuth_deg:g} deg '
                f'({rotation:.0f} m/s from Earth rotation)')
    if not mission.site_name:
        return (('launch site', geometry),)
    return (('launch site', mission.site_name), ('pad', geometry))


def _pressure(result) -> str:
    """The peak the set found asks of the airframe, against what it is built for.

    A quicker ascent is a flatter one, and a flatter one goes faster lower
    down, so this is the first thing minimising the ascent time spends. Only a
    constraint if the caller made it one.
    """
    peak = result.best.peak_dynamic_pressure
    design = result.vehicle.design_dynamic_pressure
    line = f'{peak / 1000:.1f} kPa'
    if result.max_dynamic_pressure is not None:
        return line + f' (held under {result.max_dynamic_pressure / 1000:g} kPa)'
    if design:
        return line + f' (design {design / 1000:g} kPa' \
            + (', over it' if peak > design else '') + ')'
    return line


def _demand(telemetry: Telemetry, guided_until: float) -> str:
    """How hard the programme leaned on the guidance, and whether it could.

    The steering loss prices the deflection that holding the programme would
    take, as the sine of that deflection. Where the sine passes one there is no
    such deflection - the thrust cannot hold the programme - and the price
    saturates at the whole of the thrust. Two sets that both saturate cannot be
    told apart by their steering loss, so the share that saturates says how far
    the figure above is a measurement at all.
    """
    # only while the programme runs: there is no demand to meet once the
    # vehicle holds the attitude it reached, and rows past the handover would
    # dilute the share below with zeros that mean nothing
    guided = (telemetry.thrust > 0.0) & (telemetry.t <= guided_until)
    demand = np.abs(telemetry.steering_demand[guided])
    if not len(demand):
        return 'no powered flight under guidance'
    saturated = float(np.mean(demand >= 1.0))
    peak = f'peak {demand.max():.3f} of the 1.0 the thrust can give'
    if not saturated:
        return peak
    # a nought is what this line says when nothing saturated, so a share too
    # small to round to one percent has to be spelled out rather than rounded
    share = f'{saturated * 100:.0f}%' if saturated >= 0.005 else 'less than 1%'
    return f'{peak}, unreachable over {share} of the burn'


def _block(lines: list[str], title: str, rows: list[tuple[str, str]]) -> None:
    lines.append('')
    lines.append(f'{title.upper()}')
    for label, value in rows:
        lines.append(f'  {label:<{LABEL_WIDTH}}{value}')


def _first_time(telemetry: Telemetry, series: np.ndarray, threshold: float) -> str:
    reached = np.flatnonzero(series >= threshold)
    return f'{telemetry.t[reached[0]]:.1f} s' if len(reached) else 'not reached'


def _propellant_left(vehicle: LaunchVehicle, telemetry: Telemetry, index: int) -> str:
    """How much the stage that was burning had left when its engines stopped."""
    stage_index = int(telemetry.stage[index])
    stage = vehicle.stages[stage_index]
    if stage.propellant_mass <= 0:
        return 'none'
    stack = sum(s.dry_mass + s.propellant_mass for s in vehicle.stages[stage_index:])
    left = stage.propellant_mass - (stack - telemetry.mass[index])
    return f'{left:,.0f} kg, {100 * left / stage.propellant_mass:.1f} % of stage {stage_index + 1}'


def summarise_search(result) -> str:
    """The console summary of a parameter search: what it swept, what it found.

    `result` is a `search.SearchResult`, left untyped so that this module says
    nothing about the search: a flight summary has no business importing the
    machinery that goes looking for one.
    """
    lines = _preamble(result)
    _block(lines, 'grid', [*_axes(result), *_grid_cost(result)])
    _block(lines, 'cost', _cost(result))

    if not result.found:
        lines.append('')
        if result.over_pressure:
            lines.append(f'{result.over_pressure:,} sets reached an orbit and '
                         f'every one of them was put aside for asking more of '
                         f'the airframe than '
                         f'{result.max_dynamic_pressure / 1000:g} kPa: there is '
                         f'no set here that meets both conditions')
        else:
            lines.append('no node of the grid came out on an orbit at all')
        return '\n'.join(lines)

    lines.append('')
    lines.extend(search_table(result))
    lines.extend(_found(result))
    return '\n'.join(lines)


def summarise_plan(result) -> str:
    """The grid a search would walk, and what it would cost, before it walks it.

    The two blocks the summary of a finished search opens with, and then the
    axes and the nodes the passes come to. A grid is cheap to get wrong and
    expensive to walk, so `ascent-search --dry-run` prints this and stops.

    `result` is a `search.SearchResult` as `search.plan` returns one: everything
    settled before the first trajectory, and nothing found yet.
    """
    lines = _preamble(result)
    _block(lines, 'grid', [*_axes(result), *_grid_cost(result),
                           ('nodes planned', f'{result.planned_nodes:,}')])
    return '\n'.join(lines)


def _preamble(result) -> list[str]:
    """What the search was asked for, and what was estimated before it ran."""
    lines = [f'{result.vehicle.name} to {result.target_altitude / 1000:g} km, '
             f'{result.programme}']
    _block(lines, 'search', [
        ('launch site', f'{result.latitude_deg:g} deg latitude, '
                        f'azimuth {result.azimuth_deg:g} deg'),
        ('ranked by', 'how far the apogee and the perigee ended up from the '
                      'circle asked for, added'),
        ('reaches the orbit when', _conditions(result)),
    ])

    early, late = result.window
    _block(lines, 'estimated in advance', [
        ('energy of the orbit', f'{result.required_velocity:.1f} m/s of '
                                f'characteristic velocity'),
        ('ideal vacuum time', f'{result.vacuum_time:.1f} s'),
        ('equivalent time', f'{result.equivalent_time:.1f} s'),
        ('cut-off searched in', f'{early:.1f} to {late:.1f} s'),
    ])
    return lines


def _conditions(result) -> str:
    """The three tolerances a set has to meet, in one line."""
    return (f'the perigee, the apogee and the altitude at cut-off are all '
            f'within {result.tolerance / 1000:g} km of the target and the '
            f'speed there is within {result.speed_tolerance:g} m/s of circular')


def _axes(result) -> list[tuple[str, str]]:
    """Every parameter of the family with the range it was searched over.

    All of them, held ones included. A parameter that did not move is the thing
    a reader of this summary most needs to be told about, because it is the one
    the search did not answer.
    """
    return [(name, span.describe()) for name, span in result.ranges.items()]


def _grid_cost(result) -> list[tuple[str, str]]:
    """What the shape of the grid comes to: the passes and what they resolve."""
    refinements = max(result.passes - 1, 0)
    passes = (f'{result.passes}, the sweep and {refinements} closing in on it, '
              f'each one step wide about the best node and halving the step'
              if refinements else f'{result.passes}, the sweep alone')
    rows = [('passes', passes)]

    finest = ', '.join(f'{name} {span.step / 2 ** refinements:g}'
                       for name, span in result.searched.items())
    if finest:
        rows.append(('finest step', finest))
    return rows


def _cost(result) -> list[tuple[str, str]]:
    """What the search cost: nodes visited, and what became of each."""
    nodes = max(result.nodes, 1)
    return [
        ('nodes visited', f'{result.nodes:,}'),
        ('walked already', f'{result.revisited:,}, skipped: a pass that closes '
                           f'in shares nodes with the pass before it'),
        ('screened out', f'{result.screened:,} '
                         f'({100 * result.screened / nodes:.0f} %), unflown, '
                         f'by the altitude integral'),
        ('refused by the family', f'{result.refused:,}'),
        ('could not be flown', f'{result.failed:,}'),
        ('closed on no orbit', f'{result.no_orbit:,}'),
        ('reached an orbit', f'{result.closed:,}'),
        ('distinct sets found', f'{len(result.found):,}'),
        ('trajectories flown', f'{result.flown:,}'),
        ('divided over', f'{result.workers} '
                         f'{"process" if result.workers == 1 else "processes"}'),
    ]


# name, width, decimal places, how to read it off a candidate. The parameters
# of the family come before these and are built per search, since which of them
# were searched and how finely is part of what the search was asked for.
TERMINAL_COLUMNS = (
    ('cut-off', 10, 3, lambda c: c.cutoff_time),
    ('gamma', 8, 3, lambda c: c.flight_path_angle),
    ('h km', 9, 2, lambda c: c.altitude / 1000),
    ('h err', 8, 5, lambda c: c.altitude_error),
    ('v m/s', 9, 1, lambda c: c.speed),
    ('v err', 8, 5, lambda c: c.speed_error),
    ('per km', 9, 2, lambda c: c.orbit.perigee_altitude / 1000),
    ('apo km', 9, 2, lambda c: c.orbit.apogee_altitude / 1000),
    ('ecc', 9, 6, lambda c: c.orbit.eccentricity),
    ('orbit err', 10, 6, lambda c: c.orbit_error),
)


def search_table(result) -> list[str]:
    """The sets found, best first, with the errors each is judged by.

    A search is a map of a family before it is one answer out of it, and the
    map is what says where to look next: rows that crowd together along one
    parameter have found the orbit in that direction, and a best row sitting at
    the edge of the table has been asked to look in the wrong place.

    Only the parameters that were actually searched get a column. A held one is
    the same in every row, and it is printed above where it belongs.
    """
    rows = result.found[:max(result.top, 0)]
    if not rows:
        return []

    columns = [*_axis_columns(result), *TERMINAL_COLUMNS]
    reaching = sum(1 for candidate in result.found
                   if candidate.reaches(result.tolerance, result.speed_tolerance))
    lines = [f'TOP {len(rows):,} OF THE {len(result.found):,} SETS THAT '
             f'REACHED AN ORBIT, BEST FIRST',
             f'{reaching:,} of them meet all three tolerances, marked *'
             if reaching else 'none of them meets all three tolerances',
             '']

    header = f'{"#":>4}' + ''.join(f'{name:>{width}}'
                                   for name, width, _, _ in columns)
    lines.append(header)
    lines.append('-' * len(header))
    for index, candidate in enumerate(rows, start=1):
        marker = '*' if candidate.reaches(result.tolerance,
                                          result.speed_tolerance) else ''
        line = f'{f"{index}{marker}":>4}'
        for _, width, decimals, read in columns:
            line += f'{read(candidate):>{width}.{decimals}f}'
        lines.append(line)
    return lines


def _axis_columns(result) -> list[tuple]:
    """One column per parameter the search actually swept.

    The decimals are read off the step the search resolves that parameter to -
    the step of the sweep, halved once per refining pass - so a column says
    exactly as much as the search knows about that parameter and no more.
    """
    refinements = max(result.passes - 1, 0)
    columns = []
    for name, span in result.searched.items():
        decimals = _decimals(span.step / 2 ** refinements)
        width = max(len(name) + 2, decimals + 6)
        columns.append((name, width, decimals,
                        lambda candidate, name=name: candidate.values[name]))
    return columns


def _decimals(step: float) -> int:
    """How many decimals it takes to tell two neighbouring nodes apart."""
    if step <= 0.0:
        return 3
    return min(6, max(0, math.ceil(-math.log10(step)) + 1))


def _found(result) -> list[str]:
    """The set the search answers with, in full."""
    best = result.best
    orbit = best.orbit
    lines: list[str] = []
    _block(lines, 'found', [
        ('pitch programme', ', '.join(
            f'{key}={value:g}' if isinstance(value, float) else f'{value}'
            for key, value in best.parameters.items() if key != 'type')),
        ('cut-off', f'{best.cutoff_time:.4f} s'
                    + (f', {best.values["coast"]:g} s of it after the programme'
                       if best.values.get('coast') else '')),
        ('altitude at cut-off', f'{best.altitude / 1000:.3f} km, '
                                f'{best.altitude_miss:.0f} m out'),
        ('speed at cut-off', f'{best.speed:.1f} m/s, {best.speed_miss:.1f} m/s '
                             f'from the circular speed there'),
        ('perigee', f'{orbit.perigee_altitude / 1000:.3f} km'),
        ('apogee', f'{orbit.apogee_altitude / 1000:.3f} km'),
        ('eccentricity', f'{orbit.eccentricity:.6f}'),
        ('worst apsidal error', f'{best.miss:.0f} m'),
        ('reaches the orbit', 'yes' if result.reaches_orbit
                              else 'no: nothing on the grid met all three'),
        ('velocity budget', f'gravity {best.gravity_loss:.1f}, aerodynamic '
                            f'{best.aerodynamic_loss:.1f}, steering '
                            f'{best.steering_loss:.1f}, total '
                            f'{best.total_loss:.1f} m/s'),
        ('max dynamic pressure', _pressure(result)),
        ('peak steering demand', f'{best.peak_steering_demand:.3f} of the 1.0 '
                                 f'the thrust can give'
                                 + (' - above it the programme cannot be held'
                                    if best.peak_steering_demand > 1.0 else '')),
    ])

    if result.over_pressure:
        lines.append('')
        lines.append(f'{result.over_pressure:,} sets reached an orbit and were '
                     f'put aside for asking more of the airframe than '
                     f'{result.max_dynamic_pressure / 1000:g} kPa')

    if result.on_edge:
        lines.append('')
        lines.append(f'the set found sits on a bound of the grid in '
                     f'{", ".join(result.on_edge)}: either the family gives out '
                     f'there, or a better set lies outside the range searched')
    return lines
