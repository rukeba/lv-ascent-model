"""Command line entry points: fly one mission, or search for a programme.

    ascent f9                             # config/mission.f9.yaml
    ascent f9 --csv out/f9.csv            # and the whole trajectory as CSV
    ascent f9 --report                    # an HTML report under out/, opened
    ascent config/mission.a62.yaml        # a mission file by path

    ascent f9 --altitude 650               # a solved set from the catalogue
    ascent f9 -a 650 -p bt                 # the same, in short
    ascent f9 --list                       # what the catalogue holds

    ascent-search f9 --altitude 500        # solve for a set instead of flying one
    ascent-search f9 --altitude 500 -r     # and a report of the set it found
    ascent-search f9 -a 500 --band 1             # a band of sets, not one
    ascent-search f9 -a 500 --free none          # the shape of the turn alone

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
from .search import (CRITERIA, FAMILIES, FLIGHTS_PER_NODE,
                     MAX_STEERING_DEMAND, Axis, default_workers, search)
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
        self.counted = False

    def __call__(self, result) -> None:
        if not self.counted:
            # what the search is about to cost, said before it has cost it.
            # The nodes are known exactly; the trajectories are a rate measured
            # off other searches, and it is the figure worth seeing beforehand
            self.counted = True
            layout = ' x '.join(f'{name} {axis.nodes}'
                                for name, axis in result.axes.items())
            print(f'{result.planned_nodes:,} nodes over {result.passes} passes '
                  f'({layout}), some '
                  f'{result.planned_nodes * FLIGHTS_PER_NODE:,} trajectories',
                  file=self.stream)
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


def _demand_limit(parser, given: str) -> float | None:
    """How hard a set may lean on the guidance and still be an answer.

    `none` lifts the limit, which is how every set on file was solved: a third
    of them ask a deflection the thrust cannot give, and they are on file
    because the figure was reported rather than imposed.
    """
    if given.lower() == 'none':
        return None
    try:
        limit = float(given)
    except ValueError:
        parser.error(f'--max-demand takes a number or `none`, and not {given!r}')
    if limit <= 0.0:
        parser.error(f'--max-demand has to be above zero, and is {limit:g}')
    return limit


def _ranges(parser, given: list[str], programme: str,
            free: tuple[str, ...]) -> dict[str, Axis]:
    """The axes the command line narrowed, as the grid reads them.

    `k2=0.04:0.08:9` is nine nodes from 0.04 to 0.08. A name that is not an
    axis of the search about to run is refused here, with the ones that are:
    the search would refuse it too, and this says it in one line rather than a
    traceback.
    """
    family = FAMILIES.get(programme)
    axes = tuple(family(free=free).axes()) if family is not None else ()
    ranges = {}
    for text in given:
        name, _, span = text.partition('=')
        parts = span.split(':')
        if not name or len(parts) != 3:
            parser.error(f'--range takes AXIS=LOW:HIGH:NODES, and not {text!r}')
        try:
            low, high, nodes = float(parts[0]), float(parts[1]), int(parts[2])
        except ValueError:
            parser.error(f'--range takes numbers, and not {text!r}')
        if high < low or nodes < 1:
            parser.error(f'--range needs low to high and at least one node, '
                         f'and not {text!r}')
        if axes and name not in axes:
            parser.error(f'--range {name}: this search runs over '
                         f'{", ".join(axes)}')
        ranges[name] = Axis(low, high, nodes)
    return ranges


def _free_axes_help() -> str:
    """What each family can search beside the shape of its turn."""
    return '; '.join(f'{name} {", ".join(family.FREE)}'
                     for name, family in sorted(FAMILIES.items()) if family.FREE)


def _free_axes(parser, programme: str, asked) -> tuple[str, ...]:
    """The axes to search, with `all` and `none` standing for the two ends.

    A name the family does not have is a typed mistake rather than a search
    worth running, so it is refused here with what the family does have. A
    programme this command does not know is left to the search to refuse,
    which it does before anything is flown.
    """
    family = FAMILIES.get(programme)
    if family is None:
        return tuple(asked)
    names = (family.names(asked[0]) if len(asked) == 1
             and asked[0] in ('all', 'none') else tuple(asked))
    unknown = [name for name in names if name not in family.FREE]
    if unknown:
        parser.error(f'--free {" ".join(unknown)}: the {programme} family can '
                     f'search all, none, or any of '
                     f'{", ".join(family.FREE) or "nothing"} beside the shape '
                     f'of its turn')
    return names


def search_main(argv: list[str] | None = None) -> int:
    """Entry point of `ascent-search`: solve for a programme instead of flying one.

        ascent-search f9 --altitude 500
        ascent-search f9 -a 650 -p bt --yaml
        ascent-search f9 --altitude 500 --report        # fly the set found, too
        ascent-search a62 --altitude 700 --coarse 0.5   # a quicker, rougher look
        ascent-search f9 -a 500 --band 1                # the band, not one set
        ascent-search f9 -a 500 --free none             # the shape alone

    The mission file supplies the vehicle and the launch site, and its own
    target altitude and programme type stand in for `--altitude` and
    `--programme` when those are not given. The pitch-programme parameters in
    it are ignored: they are what the search is for.
    """
    parser = argparse.ArgumentParser(
        prog='ascent-search',
        description='Search for the pitch-programme parameters that reach a '
                    'circular orbit, best first.')
    parser.add_argument('mission', help='mission name (f9) or path to a mission '
                                        'YAML file: the vehicle and the launch '
                                        'site are taken from it')
    parser.add_argument('--altitude', '-a', type=float, metavar='KM',
                        help='altitude of the circular orbit to aim for')
    parser.add_argument('--programme', '-p', metavar='NAME',
                        choices=sorted(FAMILIES) + sorted(PROGRAMME_ALIASES),
                        help=f'pitch programme to search: {_programmes(FAMILIES)}')
    parser.add_argument('--free', nargs='+', metavar='AXIS', default=('all',),
                        help=f'what to search beside the shape of the turn: '
                             f'`all`, the default, or `none` for the search '
                             f'that solved the catalogue, or any of '
                             f'{_free_axes_help()}')
    parser.add_argument('--band', type=float, default=0.0, metavar='WIDTH',
                        help='report not the one set but every set found that '
                             'reaches the orbit and comes within this of it, '
                             'as a range along each parameter and as a table '
                             'of the sets themselves. In the unit of the '
                             'criterion: m/s of velocity budget, or seconds of '
                             'ascent')
    parser.add_argument('--criterion', default='orbit',
                        choices=sorted(CRITERIA),
                        help='what to rank the sets found by: `orbit`, how far '
                             'the orbit each closed is from the one asked for, '
                             'or - among those that reach it - `loss`, the '
                             'whole velocity budget, or `time`, the earliest '
                             'cut-off (default orbit)')
    parser.add_argument('--top', type=int, default=10, metavar='N',
                        help='how many of the sets found to print, best first, '
                             'with the three terminal errors of each '
                             '(default 10)')
    parser.add_argument('--range', action='append', default=[],
                        metavar='AXIS=LOW:HIGH:NODES',
                        help='narrow one axis of the grid, repeatable: '
                             '`--range k2=0.04:0.08:9`. What a coarse search '
                             'found is what the next one is narrowed on to, '
                             'and the bounds hold for the refining passes too')
    parser.add_argument('--max-demand', default=str(MAX_STEERING_DEMAND),
                        metavar='SINE',
                        help='the largest thrust deflection, as its sine, a '
                             'set may ask of the guidance and still count as '
                             'an answer (default 1, the whole of the thrust); '
                             '`none` reports it without limiting it')
    parser.add_argument('--csv', metavar='FILE',
                        help='write the whole band to a CSV file - every set, '
                             'the orbit it reaches, what it costs and what it '
                             'demands - where the summary prints the first '
                             'twenty')
    parser.add_argument('--tolerance', type=float, default=0.5, metavar='KM',
                        help='how close the perigee and the apogee have to come '
                             'to the target for the set to count (default 0.5)')
    parser.add_argument('--refinements', type=int, default=10, metavar='N',
                        help='passes of the grid after the first, each one grid '
                             'step wide about the best node (default 10)')
    parser.add_argument('--max-q', type=float, metavar='KPA',
                        help='put the airframe into the constraint: sets whose '
                             'dynamic pressure peaks above this are not answers, '
                             'however close. Without it the peak is reported '
                             'and nothing more')
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
                        ('--steps', arguments.steps),
                        ('--coarse', arguments.coarse)):
        if value <= 0.0:
            parser.error(f'{name} has to be above zero, and is {value:g}')
    if arguments.refinements < 0:
        parser.error(f'--refinements cannot be negative, and is '
                     f'{arguments.refinements}')
    if arguments.max_q is not None and arguments.max_q <= 0.0:
        parser.error(f'--max-q has to be above zero, and is {arguments.max_q:g}')
    if arguments.band < 0.0:
        parser.error(f'--band cannot be negative, and is {arguments.band:g}')
    if arguments.top < 0:
        parser.error(f'--top cannot be negative, and is {arguments.top}')
    demand = _demand_limit(parser, arguments.max_demand)

    mission_path = resolve(arguments.mission, Path(arguments.config_dir))
    spec = read_spec(mission_path)
    site = spec.get('launch_site', {})
    vehicle_file = spec['vehicle']

    # with `--yaml` the entry is the whole of what this command is for, so
    # everything else goes to the error stream and a redirect of the output is
    # a file the catalogue reader can read
    told = sys.stderr if arguments.yaml else sys.stdout
    progress = _Progress(sys.stderr)
    programme = arguments.programme or spec['pitch_programme']['type']
    free = _free_axes(parser, programme, arguments.free)
    result = search(
        vehicle=load_vehicle(mission_path.parent / f'{vehicle_file}.yaml'),
        target_altitude=(arguments.altitude * 1000 if arguments.altitude is not None
                         else spec['target_altitude']),
        programme=programme,
        latitude_deg=site.get('latitude', 0.0),
        azimuth_deg=site.get('azimuth', 90.0),
        free=free,
        ranges=_ranges(parser, arguments.range, programme, free),
        criterion=arguments.criterion,
        top=arguments.top,
        max_steering_demand=demand,
        band_tolerance=arguments.band,
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

    if arguments.csv:
        path = Path(arguments.csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        written = _write_band(path, result)
        print(f'\nband: {written:,} sets to {path}', file=told)

    if arguments.report is not None:
        _report_the_search(result, arguments, spec, mission_path, told,
                           _command_line(parser.prog, argv))
    return 0 if result.reaches_orbit else 1


def _write_band(path: Path, result) -> int:
    """The whole band as a table, one set to a row. How many were written.

    The summary prints the first twenty and says how wide the band is; a study
    wants all of them, to be sorted and filtered by whatever it is looking for
    at the time. The columns are the parameters of the programme, the orbit
    each set reaches, what it misses the target by, what it costs and what it
    asks of the airframe and the guidance.
    """
    band = result.band
    if not band:
        path.write_text('')
        return 0
    keys = [key for key in band[0].parameters if key != 'type']
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow([*keys, 'cutoff_s', 'perigee_km', 'apogee_km',
                         'miss_m', 'gravity_loss', 'aerodynamic_loss',
                         'steering_loss', 'total_loss', 'max_q_kpa',
                         'steering_demand'])
        for found in band:
            writer.writerow(
                [f'{found.parameters[key]:.9g}' for key in keys]
                + [f'{found.cutoff_time:.6f}',
                   f'{found.orbit.perigee_altitude / 1000:.4f}',
                   f'{found.orbit.apogee_altitude / 1000:.4f}',
                   f'{found.miss:.1f}',
                   f'{found.gravity_loss:.2f}',
                   f'{found.aerodynamic_loss:.2f}',
                   f'{found.steering_loss:.2f}',
                   f'{found.total_loss:.2f}',
                   f'{found.peak_dynamic_pressure / 1000:.2f}',
                   f'{found.peak_steering_demand:.4f}'])
    return len(band)


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
