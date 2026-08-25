"""Command line entry points: fly one mission, or search for a programme.

    ascent f9                             # config/mission.f9.yaml
    ascent f9 --csv out/f9.csv            # and the whole trajectory as CSV
    ascent f9 --report                    # an HTML report under out/, opened
    ascent config/mission.a62.yaml        # a mission file by path

    ascent f9 --altitude 650               # a solved set from the catalogue
    ascent f9 -a 650 -p bt                 # the same, in short
    ascent f9 --list                       # what the catalogue holds

    ascent-search f9 --altitude 500        # search for a set instead of flying one
    ascent-search f9 -a 500 -p 5f --range t1=10:30:2   # one axis of the grid, my way
    ascent-search f9 -a 500 -p 5f --dry-run            # the grid, before it is flown

A pitch programme can be named in full or by the short form beside it - `5f`,
`vs`, `bt`.
"""

import argparse
import csv
import os
import shlex
import sys
import time
import webbrowser
from pathlib import Path

import yaml

from .config import (PITCH_PROGRAMMES, PROGRAMME_ALIASES, find_in_catalogue,
                     load_catalogue, load_vehicle, mission_from_spec,
                     programme_name, read_spec, resolve)
from .search import (FAMILIES, REFINEMENTS, SPEED_TOLERANCE, TOP, axis_names,
                     default_workers, parse_ranges, plan, search)
from .summary import summarise, summarise_plan, summarise_search


def _programmes(names) -> str:
    """The programmes that can be asked for, each with its short form."""
    short = {full: alias for alias, full in PROGRAMME_ALIASES.items()}
    return ', '.join(f'{name} ({short[name]})' if name in short else name
                     for name in sorted(names))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog='ascent', description='Simulate the powered ascent of a launch vehicle.')
    parser.add_argument('mission', help='mission name (f9) or path to a mission YAML file')
    parser.add_argument('--altitude', '-a', type=float, metavar='KM',
                        help='fly the catalogue entry for this target altitude')
    parser.add_argument('--programme', '-p', metavar='NAME',
                        help=f'pitch programme to take from the catalogue: '
                             f'{_programmes(PITCH_PROGRAMMES)}')
    parser.add_argument('--list', action='store_true',
                        help='list the catalogue entries for this vehicle and stop')
    parser.add_argument('--csv', metavar='FILE', help='write the whole trajectory to a CSV file')
    parser.add_argument('--report', '-r', metavar='DIR', nargs='?', const='',
                        help='write an HTML report with plots and open it; '
                             'on its own it writes to out/ and the name of the '
                             'vehicle')
    parser.add_argument('--no-open', action='store_true',
                        help='write the report without opening it in a browser')
    parser.add_argument('--config-dir', default='config', metavar='DIR',
                        help='where short mission names and the catalogue are '
                             'looked up; a mission given by path is read where '
                             'it lies, along with the vehicle beside it')
    arguments = parser.parse_args(argv)
    if arguments.programme:
        arguments.programme = programme_name(arguments.programme)

    directory = Path(arguments.config_dir)
    mission_path = resolve(arguments.mission, directory)
    spec = read_spec(mission_path)
    # a mission names its vehicle file as a neighbour, so a mission given by
    # path brings its own vehicle with it rather than borrowing one from the
    # configuration directory
    beside = mission_path.parent

    if arguments.list:
        _list(directory, spec['vehicle'])
        return 0

    if arguments.altitude is not None or arguments.programme is not None:
        entry = find_in_catalogue(
            load_catalogue(directory / 'catalogue.yaml'),
            vehicle=spec['vehicle'],
            target_altitude=(arguments.altitude * 1000 if arguments.altitude is not None
                             else spec['target_altitude']),
            programme=arguments.programme or spec['pitch_programme']['type'])
        entry['launch_site'] = _named_site(entry.get('launch_site', {}),
                                           spec.get('launch_site', {}))
        spec = entry
        # catalogue entries name the vehicles that sit beside the catalogue
        beside = directory

    mission = mission_from_spec(spec, beside)
    telemetry = mission.run()
    print(summarise(mission, telemetry))

    if arguments.csv:
        path = Path(arguments.csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        telemetry.write_csv(path)
        print(f'\ntrajectory: {path}')

    if arguments.report is not None:
        from .report import write_report
        path = write_report(mission, telemetry,
                            _report_directory(arguments.report, spec['vehicle']),
                            command=_command_line(parser.prog, argv))
        print(f'report: {path}')
        # a report is written to be looked at, so it is opened where it can be
        if not arguments.no_open:
            webbrowser.open(path.resolve().as_uri())

    return 0


def _named_site(site: dict, named: dict) -> dict:
    """A catalogue launch site, given what the mission file calls that pad.

    The catalogue records a site as two numbers - it is written out by the
    search, which knows it as two numbers. Where the mission file the command
    named describes the same pad, its name comes along with the entry.
    """
    if 'name' in named and all(site.get(key) == named.get(key)
                               for key in ('latitude', 'azimuth')):
        return {**site, 'name': named['name']}
    return site


def _command_line(prog: str, argv: list[str] | None) -> str:
    """The command that produced the run, as it can be typed again.

    Written on to the report, so that a page found later says what made it.
    `uv run` leaves itself in the environment and this project is run through
    it, so the line is the whole of what has to be typed.
    """
    words = sys.argv[1:] if argv is None else argv
    line = ' '.join(shlex.quote(word) for word in [prog, *words])
    return f'uv run {line}' if os.environ.get('UV') else line


def _report_directory(given: str, vehicle: str) -> Path:
    """Where the report goes: what was asked for, or out/ and the vehicle.

    `--report` on its own is the common case - one run, looked at once - and
    naming a directory for it every time is a chore. `lv.f9` writes to
    `out/f9`, so a second run of the same vehicle replaces the first.
    """
    return Path(given) if given else Path('out') / vehicle.removeprefix('lv.')


def _list(directory: Path, vehicle: str) -> None:
    catalogue = [spec for spec in load_catalogue(directory / 'catalogue.yaml')
                 if spec['vehicle'] == vehicle]
    print(f'{"altitude":>10}{"programme":>18}{"cut-off":>10}'
          f'{"steering":>10}{"total loss":>12}')
    for spec in sorted(catalogue, key=lambda s: (s['target_altitude'],
                                                 s['pitch_programme']['type'])):
        reached = spec.get('reached', {})
        print(f'{spec["target_altitude"] / 1000:>8g} km'
              f'{spec["pitch_programme"]["type"]:>18}'
              f'{spec["cutoff"]["time"]:>9.1f} s'
              f'{reached.get("steering_loss", float("nan")):>9.1f} '
              f'{reached.get("total_loss", float("nan")):>11.1f} m/s')


class _Progress:
    """One line of progress, rewritten in place while the search runs.

    The passes and the nodes of each are known before the search starts, so the
    share of the nodes done is a share of the work, and the time already spent
    scales up to a time still to go. It is an estimate of an estimate: the
    first pass covers the whole range of a family and takes more trajectories
    per node than the passes that close in on an answer, so the figure starts
    pessimistic and settles as it goes.
    """

    # how often the line is rewritten, s
    INTERVAL = 0.5

    def finish(self) -> None:
        """Close the line off, wherever the search stopped.

        Not left to the last pass to do: a pass that solves nothing ends the
        search early, and the summary would then be printed on the end of the
        progress line.
        """
        if self.written:
            print(file=self.stream)
            self.written = False

    def __init__(self, stream) -> None:
        self.stream = stream
        self.start = self.last = time.monotonic()
        self.width = 0
        self.written = False

    def __call__(self, result) -> None:
        now = time.monotonic()
        done = result.pass_node == result.pass_nodes
        if now - self.last < self.INTERVAL and not done:
            return
        self.last = now

        elapsed = now - self.start
        share = result.nodes / max(result.planned_nodes, 1)
        line = (f'pass {result.pass_number}/{result.passes}  '
                f'node {result.pass_node}/{result.pass_nodes}  '
                f'{result.flown} flights  {_clock(elapsed)} gone')
        if share > 0.0:
            line += f', about {_clock(elapsed / share - elapsed)} left'
        if result.best is not None:
            line += f'  (best {result.best.cutoff_time:.2f} s, ' \
                    f'{result.best.miss:.0f} m out)'
        print(f'\r{line:<{self.width}}', end='', flush=True, file=self.stream)
        self.width = max(self.width, len(line))
        self.written = True


def _clock(seconds: float) -> str:
    return f'{int(seconds) // 60}:{int(seconds) % 60:02d}'


def search_main(argv: list[str] | None = None) -> int:
    """Entry point of `ascent-search`: solve for a programme instead of flying one.

        ascent-search f9 --altitude 500
        ascent-search f9 -a 500 -p 5f --range t1=10:30:2   # one axis, my way
        ascent-search f9 -a 500 -p 5f --dry-run            # the grid, unflown
        ascent-search f9 -a 650 -p bt --yaml
        ascent-search f9 --altitude 500 --report           # fly the set found

    The mission file supplies the vehicle and the launch site, and its own
    target altitude and programme type stand in for `--altitude` and
    `--programme` when those are not given. The pitch-programme parameters in
    it are ignored: they are what the search is for.

    Every parameter of the family is an axis of the grid, and `--range` says
    what any of them is searched over. `--dry-run` prints the grid, every axis
    with its range and its step, and stops before the first trajectory.
    """
    parser = argparse.ArgumentParser(
        prog='ascent-search',
        description='Search a grid over every parameter of a pitch programme '
                    'for the sets that reach a circular orbit.')
    parser.add_argument('mission', help='mission name (f9) or path to a mission '
                                        'YAML file: the vehicle and the launch '
                                        'site are taken from it')
    parser.add_argument('--altitude', '-a', type=float, metavar='KM',
                        help='altitude of the circular orbit to aim for')
    parser.add_argument('--programme', '-p', metavar='NAME',
                        choices=sorted(FAMILIES) + sorted(PROGRAMME_ALIASES),
                        help=f'pitch programme to search: {_programmes(FAMILIES)}')
    parser.add_argument('--range', action='append', default=[], dest='ranges',
                        metavar='NAME=LOW:HIGH:STEP',
                        help='what one parameter is searched over, repeatable. '
                             '`--range t1=10:30:2` walks the vertical rise from '
                             '10 to 30 s in steps of 2, and `--range k2=0.05` '
                             'holds a parameter at the one value. Every '
                             'parameter of the family is an axis and every one '
                             'of them takes a range; `--dry-run` lists them '
                             'with the ranges they are searched over by default')
    parser.add_argument('--top', type=int, default=TOP, metavar='N',
                        help=f'how many of the sets found to print, best first, '
                             f'with the errors each is judged by (default {TOP})')
    parser.add_argument('--tolerance', type=float, default=0.5, metavar='KM',
                        help='how close the perigee, the apogee and the altitude '
                             'at cut-off have to come to the target for the set '
                             'to count (default 0.5)')
    parser.add_argument('--speed-tolerance', type=float, default=SPEED_TOLERANCE,
                        metavar='M_S',
                        help=f'and how close the speed at cut-off has to come to '
                             f'the speed of that circular orbit (default '
                             f'{SPEED_TOLERANCE:g})')
    parser.add_argument('--refinements', type=int, default=REFINEMENTS,
                        metavar='N',
                        help=f'passes after the sweep, each one grid step wide '
                             f'about the best node and halving the step along '
                             f'every axis searched (default {REFINEMENTS}). A '
                             f'step of the sweep is worth tens of kilometres of '
                             f'apogee, and these are what close that down')
    parser.add_argument('--max-q', type=float, metavar='KPA',
                        help='put the airframe into the constraint: sets whose '
                             'dynamic pressure peaks above this are not answers, '
                             'however close. Without it the peak is reported and '
                             'nothing more')
    parser.add_argument('--coarse', type=float, default=1.0, metavar='FACTOR',
                        help='scale the nodes along every axis the family gave, '
                             'below one for a quicker and rougher sweep. An axis '
                             'given a `--range` is left alone: that step was '
                             'asked for')
    parser.add_argument('--no-screen', action='store_true',
                        help='fly every node, rather than dropping the ones the '
                             'altitude integral says cannot reach the target. '
                             'Slower by as much as the screen was saving, and '
                             'the way to check that the screen is not hiding '
                             'anything')
    parser.add_argument('--steps', type=float, default=10, metavar='PER_SECOND',
                        help='integration steps per second of every trajectory '
                             'flown (default 10). A coarser step is for a quick '
                             'look and barely moves the orbit or the budget; '
                             'the entry written out asks for ten either way')
    parser.add_argument('--workers', type=int, metavar='N',
                        help=f'processes the nodes of a pass are divided over '
                             f'(default {default_workers()}, two thirds of the '
                             f'cores on this machine); 1 to search in this one')
    parser.add_argument('--dry-run', action='store_true',
                        help='print the grid - every axis with its range, its '
                             'step and its nodes - and what the passes come to, '
                             'then stop without flying anything')
    parser.add_argument('--csv', metavar='FILE',
                        help='write every set found to a CSV file, best first, '
                             'where the summary prints the best few')
    parser.add_argument('--yaml', action='store_true',
                        help='print the set found as a catalogue entry')
    parser.add_argument('--report', '-r', metavar='DIR', nargs='?', const='',
                        help='fly the set found and write the same HTML report '
                             '`ascent` writes; on its own it goes to out/ and '
                             'the name of the vehicle')
    parser.add_argument('--no-open', action='store_true',
                        help='write the report without opening it in a browser')
    parser.add_argument('--config-dir', default='config', metavar='DIR',
                        help='where short mission names are looked up')
    arguments = parser.parse_args(argv)
    if arguments.programme:
        arguments.programme = programme_name(arguments.programme)
    for name, value in (('--tolerance', arguments.tolerance),
                        ('--speed-tolerance', arguments.speed_tolerance),
                        ('--steps', arguments.steps),
                        ('--coarse', arguments.coarse),
                        ('--top', arguments.top)):
        if value <= 0.0:
            parser.error(f'{name} has to be above zero, and is {value:g}')
    if arguments.refinements < 0:
        parser.error(f'--refinements cannot be negative, and is '
                     f'{arguments.refinements}')
    if arguments.max_q is not None and arguments.max_q <= 0.0:
        parser.error(f'--max-q has to be above zero, and is {arguments.max_q:g}')

    mission_path = resolve(arguments.mission, Path(arguments.config_dir))
    spec = read_spec(mission_path)
    site = spec.get('launch_site', {})
    vehicle_file = spec['vehicle']
    programme = arguments.programme or spec['pitch_programme']['type']

    settings = dict(
        target_altitude=(arguments.altitude * 1000 if arguments.altitude is not None
                         else spec['target_altitude']),
        programme=programme,
        latitude_deg=site.get('latitude', 0.0),
        azimuth_deg=site.get('azimuth', 90.0),
        ranges=_ranges(parser, arguments.ranges, programme),
        tolerance=arguments.tolerance * 1000,
        speed_tolerance=arguments.speed_tolerance,
        refinements=arguments.refinements,
        top=arguments.top,
        max_dynamic_pressure=(arguments.max_q * 1000
                              if arguments.max_q is not None else None),
        coarseness=arguments.coarse,
        steps_per_second=arguments.steps,
    )
    vehicle = load_vehicle(mission_path.parent / f'{vehicle_file}.yaml')

    if arguments.dry_run:
        print(summarise_plan(plan(vehicle, **settings)))
        return 0

    # with `--yaml` the entry is the whole of what this command is for, so
    # everything else goes to the error stream and a redirect of the output is
    # a file the catalogue reader can read
    told = sys.stderr if arguments.yaml else sys.stdout
    progress = _Progress(sys.stderr)
    result = search(vehicle, workers=arguments.workers,
                    screen=not arguments.no_screen, report=progress, **settings)
    progress.finish()
    print(summarise_search(result), file=told)

    if arguments.csv:
        print(f'\nsets found: {_write_search_csv(result, Path(arguments.csv))}',
              file=told)

    # only a set that reaches the orbit is written out as an entry: one that
    # misses is worth showing and is not worth filing
    if arguments.yaml and result.reaches_orbit:
        print(file=told)
        print(yaml.safe_dump({'missions': [result.specification(vehicle_file)]},
                             sort_keys=False, default_flow_style=None), end='')

    if arguments.report is not None:
        _report_the_search(result, arguments, spec, mission_path, told,
                           _command_line(parser.prog, argv))
    return 0 if result.reaches_orbit else 1


def _ranges(parser, given: list[str], programme: str) -> dict:
    """The `--range` arguments, checked against the family before anything runs.

    A parameter the family does not have is a mistake in the command and is
    answered there, with the parameters it does have - not several minutes
    later, out of the middle of a search that has already started.
    """
    try:
        ranges = parse_ranges(given)
    except ValueError as complaint:
        parser.error(str(complaint))

    names = axis_names(programme)
    unknown = [name for name in ranges if name not in names]
    if unknown:
        parser.error(
            f'{", ".join(unknown)} '
            f'{"is not a parameter" if len(unknown) == 1 else "are not parameters"} '
            f'of the {programme} turn; it is made of {", ".join(names)}')
    return ranges


# name, and how to read it off one set found. The parameters of the family come
# before these, one column each, so a row of this file is a set and everything
# that was measured of it.
SEARCH_CSV_COLUMNS = (
    ('cutoff_time_s', lambda c: c.cutoff_time),
    ('flight_path_angle_deg', lambda c: c.flight_path_angle),
    ('altitude_km', lambda c: c.altitude / 1000),
    ('altitude_error', lambda c: c.altitude_error),
    ('speed_ms', lambda c: c.speed),
    ('speed_error', lambda c: c.speed_error),
    ('perigee_km', lambda c: c.orbit.perigee_altitude / 1000),
    ('apogee_km', lambda c: c.orbit.apogee_altitude / 1000),
    ('eccentricity', lambda c: c.orbit.eccentricity),
    ('orbit_error', lambda c: c.orbit_error),
    ('gravity_loss_ms', lambda c: c.gravity_loss),
    ('aerodynamic_loss_ms', lambda c: c.aerodynamic_loss),
    ('steering_loss_ms', lambda c: c.steering_loss),
    ('total_loss_ms', lambda c: c.total_loss),
    ('max_dynamic_pressure_kpa', lambda c: c.peak_dynamic_pressure / 1000),
    ('peak_steering_demand', lambda c: c.peak_steering_demand),
)


def _write_search_csv(result, path: Path) -> Path:
    """Every set the search found, best first, one row each.

    The summary prints the best few because a console is not the place for
    several thousand rows; this is where the rest of them go, so that a coarse
    sweep can be looked at as the map it is - sorted, plotted, narrowed on -
    rather than only as the one set at the head of it.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    axes = list(result.ranges)
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow([*axes, *(name for name, _ in SEARCH_CSV_COLUMNS),
                         'reaches_orbit'])
        for candidate in result.found:
            writer.writerow([
                *(f'{candidate.values[axis]:.10g}' for axis in axes),
                *(f'{read(candidate):.10g}' for _, read in SEARCH_CSV_COLUMNS),
                'yes' if candidate.reaches(result.tolerance,
                                           result.speed_tolerance) else 'no'])
    return path


def _report_the_search(result, arguments, spec: dict, mission_path: Path,
                       told, command: str) -> None:
    """Fly the set the search found, and report it as a flight.

    The report is of a flight, not of a search: the set is turned into the
    same entry `--yaml` would write and flown by the same route a catalogue
    entry is flown, so a searched set and a filed one give the same page. A
    set that misses the orbit is not an entry and is not flown - the summary
    above has already said what was found and how far out it was.
    """
    if not result.reaches_orbit:
        print('\nno report: nothing was found that reaches the orbit', file=told)
        return

    from .report import write_report
    vehicle_file = spec['vehicle']
    entry = result.specification(vehicle_file)
    entry['launch_site'] = _named_site(entry['launch_site'],
                                       spec.get('launch_site', {}))
    mission = mission_from_spec(entry, mission_path.parent)
    path = write_report(mission, mission.run(),
                        _report_directory(arguments.report, vehicle_file),
                        command=command)
    print(f'\nreport: {path}', file=told)
    if not arguments.no_open:
        webbrowser.open(path.resolve().as_uri())


if __name__ == '__main__':
    raise SystemExit(main())
