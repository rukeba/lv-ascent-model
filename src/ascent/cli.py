"""Command line entry points: fly one mission, or search for a programme.

    ascent f9                             # config/mission.f9.yaml
    ascent f9 --csv out/f9.csv            # and the whole trajectory as CSV
    ascent f9 --report                    # an HTML report under out/, opened
    ascent config/mission.a62.yaml        # a mission file by path

    ascent f9 --altitude 650               # a solved set from the catalogue
    ascent f9 -a 650 -p bt                 # the same, in short
    ascent f9 --list                       # what the catalogue holds

    ascent-search f9 --altitude 500        # solve for a set instead of flying one

A pitch programme can be named in full or by the short form beside it - `5f`,
`vs`, `bt`.
"""

import argparse
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
from .search import FAMILIES, default_workers, search
from .summary import summarise, summarise_search


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
        spec = find_in_catalogue(
            load_catalogue(directory / 'catalogue.yaml'),
            vehicle=spec['vehicle'],
            target_altitude=(arguments.altitude * 1000 if arguments.altitude is not None
                             else spec['target_altitude']),
            programme=arguments.programme or spec['pitch_programme']['type'])
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
        ascent-search f9 -a 650 -p bt --yaml
        ascent-search a62 --altitude 700 --coarse 0.5   # a quicker, rougher look

    The mission file supplies the vehicle and the launch site, and its own
    target altitude and programme type stand in for `--altitude` and
    `--programme` when those are not given. The pitch-programme parameters in
    it are ignored: they are what the search is for.
    """
    parser = argparse.ArgumentParser(
        prog='ascent-search',
        description='Search for the pitch-programme parameters that reach a '
                    'circular orbit in the shortest time.')
    parser.add_argument('mission', help='mission name (f9) or path to a mission '
                                        'YAML file: the vehicle and the launch '
                                        'site are taken from it')
    parser.add_argument('--altitude', '-a', type=float, metavar='KM',
                        help='altitude of the circular orbit to aim for')
    parser.add_argument('--programme', '-p', metavar='NAME',
                        choices=sorted(FAMILIES) + sorted(PROGRAMME_ALIASES),
                        help=f'pitch programme to search: {_programmes(FAMILIES)}')
    parser.add_argument('--tolerance', type=float, default=0.5, metavar='KM',
                        help='how close the perigee and the apogee have to come '
                             'to the target for the set to count (default 0.5)')
    parser.add_argument('--refinements', type=int, default=10, metavar='N',
                        help='passes of the grid after the first, each one grid '
                             'step wide about the best node (default 10)')
    parser.add_argument('--max-q', type=float, metavar='KPA',
                        help='put the airframe into the constraint: sets whose '
                             'dynamic pressure peaks above this are not answers, '
                             'however quick. Without it the peak is reported and '
                             'nothing more')
    parser.add_argument('--coarse', type=float, default=1.0, metavar='FACTOR',
                        help='scale the nodes along every axis: below one for a '
                             'quicker and rougher search')
    parser.add_argument('--steps', type=float, default=10, metavar='PER_SECOND',
                        help='integration steps per second of every trajectory '
                             'flown (default 10). A coarser step is for a quick '
                             'look and barely moves the orbit or the budget; '
                             'the entry written out asks for ten either way')
    parser.add_argument('--workers', type=int, metavar='N',
                        help=f'processes the nodes of a pass are divided over '
                             f'(default {default_workers()}, two thirds of the '
                             f'cores on this machine); 1 to search in this one')
    parser.add_argument('--yaml', action='store_true',
                        help='print the set found as a catalogue entry')
    parser.add_argument('--config-dir', default='config', metavar='DIR',
                        help='where short mission names are looked up')
    arguments = parser.parse_args(argv)
    if arguments.programme:
        arguments.programme = programme_name(arguments.programme)
    for name, value in (('--tolerance', arguments.tolerance),
                        ('--steps', arguments.steps),
                        ('--coarse', arguments.coarse)):
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

    # with `--yaml` the entry is the whole of what this command is for, so
    # everything else goes to the error stream and a redirect of the output is
    # a file the catalogue reader can read
    told = sys.stderr if arguments.yaml else sys.stdout
    progress = _Progress(sys.stderr)
    result = search(
        vehicle=load_vehicle(mission_path.parent / f'{vehicle_file}.yaml'),
        target_altitude=(arguments.altitude * 1000 if arguments.altitude is not None
                         else spec['target_altitude']),
        programme=arguments.programme or spec['pitch_programme']['type'],
        latitude_deg=site.get('latitude', 0.0),
        azimuth_deg=site.get('azimuth', 90.0),
        tolerance=arguments.tolerance * 1000,
        refinements=arguments.refinements,
        max_dynamic_pressure=(arguments.max_q * 1000
                              if arguments.max_q is not None else None),
        coarseness=arguments.coarse,
        steps_per_second=arguments.steps,
        workers=arguments.workers,
        report=progress)
    progress.finish()
    print(summarise_search(result), file=told)

    # only a set that reaches the orbit is written out as an entry: one that
    # misses is worth showing and is not worth filing
    if arguments.yaml and result.reaches_orbit:
        print(file=told)
        print(yaml.safe_dump({'missions': [result.specification(vehicle_file)]},
                             sort_keys=False, default_flow_style=None), end='')
    return 0 if result.reaches_orbit else 1


if __name__ == '__main__':
    raise SystemExit(main())
