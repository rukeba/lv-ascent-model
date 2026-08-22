"""Command line entry point: run one mission, print its summary, save its data.

    ascent f9                       # config/mission.f9.yaml, summary only
    ascent f9 --csv out/f9.csv      # and the full trajectory as CSV
    ascent f9 --report out/f9       # and an HTML report with plots
"""

import argparse
from pathlib import Path

from .config import load_mission, resolve
from .summary import summarise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog='ascent', description='Simulate the powered ascent of a launch vehicle.')
    parser.add_argument('mission', help='mission name (f9) or path to a mission YAML file')
    parser.add_argument('--csv', metavar='FILE', help='write the whole trajectory to a CSV file')
    parser.add_argument('--report', metavar='DIR', help='write an HTML report with plots')
    parser.add_argument('--config-dir', default='config', metavar='DIR',
                        help='where to look for mission files named by short name')
    arguments = parser.parse_args(argv)

    mission = load_mission(resolve(arguments.mission, arguments.config_dir))
    telemetry = mission.run()
    print(summarise(mission, telemetry))

    if arguments.csv:
        path = Path(arguments.csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        telemetry.write_csv(path)
        print(f'\ntrajectory: {path}')

    if arguments.report:
        from .report import write_report
        path = write_report(mission, telemetry, arguments.report)
        print(f'report: {path}')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
