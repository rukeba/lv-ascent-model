"""An HTML report of a flight: the console summary, the plots and a flight log.

Not needed to run a mission - it is what makes one readable. The numbers come
from the same summary the command line prints, so the two can never disagree.
"""

from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

from .constants import EARTH_RADIUS, circular_velocity  # noqa: E402
from .mission import Mission  # noqa: E402
from .summary import summarise  # noqa: E402
from .telemetry import Telemetry  # noqa: E402

DPI = 140
# columns of the flight log, sampled every LOG_INTERVAL seconds
LOG_INTERVAL = 20.0
LOG_COLUMNS = (
    ('t, s', 't', 1.0, 1),
    ('h, km', 'altitude', 1e-3, 1),
    ('v, m/s', 'speed', 1.0, 0),
    ('v inertial, m/s', 'inertial_speed', 1.0, 0),
    ('gamma, deg', 'flight_path_angle', 1.0, 2),
    ('m, t', 'mass', 1e-3, 1),
    ('thrust, kN', 'thrust', 1e-3, 0),
    ('q, kPa', 'dynamic_pressure', 1e-3, 1),
    ('steering, deg', 'steering_angle', 1.0, 2),
    ('steering loss, m/s', 'steering_loss', 1.0, 1),
)


def write_report(mission: Mission, telemetry: Telemetry, directory: str | Path) -> Path:
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)

    figures = _plot(mission, telemetry, directory)
    images = '\n'.join(f'<figure><img src="{name}.png" alt="{title}">'
                       f'<figcaption>{title}</figcaption></figure>'
                       for name, title in figures)

    path = directory / 'index.html'
    path.write_text(_PAGE.format(
        title=f'{mission.vehicle.name} to {mission.target_altitude / 1000:g} km',
        summary=summarise(mission, telemetry),
        images=images,
        log=_log_table(telemetry),
    ), encoding='utf-8')
    return path


def _plot(mission: Mission, telemetry: Telemetry, directory: Path):
    t = telemetry.t
    target = mission.target_altitude
    figures = []

    def figure(name, title, draw):
        fig, axes = plt.subplots(figsize=(8, 4))
        axes.set_xlabel('t, s')
        draw(axes)
        axes.grid(linestyle='--', linewidth=0.7, alpha=0.5)
        # a twin axis carries its own labels; show them in one legend
        handles, labels = [], []
        for pane in fig.axes:
            pane_handles, pane_labels = pane.get_legend_handles_labels()
            handles += pane_handles
            labels += pane_labels
        if handles:
            axes.legend(handles, labels)
        fig.savefig(directory / f'{name}.png', bbox_inches='tight', dpi=DPI)
        plt.close(fig)
        figures.append((name, title))

    def altitude(axes):
        axes.plot(t, telemetry.altitude / 1000, label='altitude')
        axes.axhline(target / 1000, color='red', linestyle=':', label='target')
        axes.set_ylabel('h, km')

    def speed(axes):
        axes.plot(t, telemetry.speed, label='relative to the Earth')
        axes.plot(t, telemetry.inertial_speed, label='inertial')
        axes.axhline(circular_velocity(target), color='red', linestyle=':',
                     label='circular at target')
        axes.set_ylabel('v, m/s')

    def angle(axes):
        axes.plot(t, telemetry.flight_path_angle, label='flown')
        axes.set_ylabel('flight-path angle, deg')

    def pressure(axes):
        axes.plot(t, telemetry.dynamic_pressure / 1000, label='dynamic pressure')
        design = mission.vehicle.design_dynamic_pressure
        if design:
            axes.axhline(design / 1000, color='red', linestyle='--', label='design')
        axes.set_ylabel('q, kPa')
        twin = axes.twinx()
        twin.plot(t, telemetry.drag / 1000, color='tab:orange', label='drag')
        twin.set_ylabel('drag, kN')

    def steering(axes):
        axes.plot(t, telemetry.steering_angle, label='demanded deflection')
        axes.set_ylabel('deflection, deg')
        twin = axes.twinx()
        twin.plot(t, telemetry.steering_loss, color='tab:red', label='accumulated loss')
        twin.set_ylabel('steering loss, m/s')

    def trajectory(axes):
        arc = np.linspace(0, np.radians(max(telemetry.polar_angle)) * 1.05, 400)
        axes.plot(EARTH_RADIUS * np.sin(arc) / 1000,
                  (EARTH_RADIUS * np.cos(arc) - EARTH_RADIUS) / 1000,
                  color='0.4', label='surface')
        radius = EARTH_RADIUS + target
        axes.plot(radius * np.sin(arc) / 1000, (radius * np.cos(arc) - EARTH_RADIUS) / 1000,
                  color='red', linestyle=':', label='target orbit')
        axes.plot(telemetry.downrange_x / 1000, telemetry.downrange_y / 1000,
                  label='trajectory')
        axes.set_xlabel('downrange, km')
        axes.set_ylabel('height above the pad, km')
        axes.set_aspect('equal')

    figure('altitude', 'Altitude', altitude)
    figure('speed', 'Speed', speed)
    figure('flight-path-angle', 'Flight-path angle', angle)
    figure('dynamic-pressure', 'Dynamic pressure and drag', pressure)
    figure('steering', 'Deflection demanded by the programme, and what it cost', steering)
    figure('trajectory', 'Trajectory in the plane of the launch', trajectory)
    return figures


def _log_table(telemetry: Telemetry) -> str:
    header = ''.join(f'<th>{title}</th>' for title, _, _, _ in LOG_COLUMNS)
    rows = []
    for t in np.arange(0.0, telemetry.t[-1] + LOG_INTERVAL, LOG_INTERVAL):
        index = telemetry.at(min(t, telemetry.t[-1]))
        cells = ''.join(f'<td>{getattr(telemetry, column)[index] * scale:.{digits}f}</td>'
                        for _, column, scale, digits in LOG_COLUMNS)
        rows.append(f'<tr>{cells}</tr>')
    return f'<table><thead><tr>{header}</tr></thead><tbody>{"".join(rows)}</tbody></table>'


_PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>{title}</title>
<style>
body {{ font: 14px/1.5 -apple-system, Segoe UI, Roboto, sans-serif; margin: 2rem auto;
       max-width: 60rem; padding: 0 1rem; color: #222; }}
h1 {{ font-size: 1.4rem; }}
pre {{ background: #f6f6f6; padding: 1rem; overflow-x: auto; font-size: 13px; }}
figure {{ margin: 1.5rem 0; }}
img {{ max-width: 100%; }}
figcaption {{ color: #666; font-size: 13px; margin-top: .3rem; }}
table {{ border-collapse: collapse; font-size: 12px; width: 100%; }}
th, td {{ border: 1px solid #ddd; padding: 3px 6px; text-align: right; }}
th {{ background: #f0f0f0; }}
</style></head><body>
<h1>{title}</h1>
<pre>{summary}</pre>
{images}
<h2>Flight log</h2>
{log}
</body></html>
"""
