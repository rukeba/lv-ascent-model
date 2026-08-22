"""Command line entry point: run one mission, print its summary, save its data.

    ascent f9                             # config/mission.f9.yaml
    ascent f9 --csv out/f9.csv            # and the whole trajectory as CSV
    ascent f9 --report out/f9             # and an HTML report with plots
    ascent config/mission.a62.yaml        # a mission file by path

    ascent f9 --altitude 650               # a solved set from the catalogue
    ascent f9 --altitude 650 --programme bilinear-tangent
    ascent f9 --list                       # what the catalogue holds
"""

import argparse
from pathlib import Path

from .config import (find_in_catalogue, load_catalogue, mission_from_spec,
                     read_spec, resolve)
from .summary import summarise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog='ascent', description='Simulate the powered ascent of a launch vehicle.')
    parser.add_argument('mission', help='mission name (f9) or path to a mission YAML file')
    parser.add_argument('--altitude', type=float, metavar='KM',
                        help='fly the catalogue entry for this target altitude')
    parser.add_argument('--programme', metavar='NAME',
                        help='pitch programme to take from the catalogue')
    parser.add_argument('--list', action='store_true',
                        help='list the catalogue entries for this vehicle and stop')
    parser.add_argument('--csv', metavar='FILE', help='write the whole trajectory to a CSV file')
    parser.add_argument('--report', metavar='DIR', help='write an HTML report with plots')
    parser.add_argument('--config-dir', default='config', metavar='DIR',
                        help='where mission, vehicle and catalogue files live')
    arguments = parser.parse_args(argv)

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

    if arguments.report:
        from .report import write_report
        print(f'report: {write_report(mission, telemetry, arguments.report)}')

    return 0


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


if __name__ == '__main__':
    raise SystemExit(main())
