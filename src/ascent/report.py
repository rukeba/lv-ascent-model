"""An HTML report of a flight: the summary, the plots and a flight log.

Not needed to run a mission - it is what makes one readable. The figures come
from the same blocks the console prints, so the page and the terminal can
never disagree about a number; what the page adds is the plots, the velocity
budget drawn to scale and the trajectory tabulated every few seconds.

The markup and the styles are files of their own, under `templates/`, and are
rendered with Jinja: the page can be restyled without touching the model. The
stylesheet is inlined as the page is written, so that the report is one file
plus the images beside it.
"""

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import matplotlib
import numpy as np
from jinja2 import Environment, FileSystemLoader, select_autoescape

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

from .constants import (EARTH_RADIUS, STANDARD_GRAVITY,  # noqa: E402
                        circular_velocity)
from .losses import velocity_budget  # noqa: E402
from .mission import Mission  # noqa: E402
from .summary import heading, summary_blocks  # noqa: E402
from .telemetry import Telemetry  # noqa: E402

TEMPLATES = Path(__file__).parent / 'templates'

# plots are drawn well above screen resolution: they are read on retina
# displays, opened full size from the page, and printed
DPI = 200
# a plot of the two-column grid, and one that spans the page
PANEL = (7.4, 4.1)
SPREAD = (12.6, 4.6)
STACK = (12.6, 8.4)

# the flight log is sampled every this many seconds
LOG_INTERVAL = 5.0
# heading, unit, telemetry column, scale, decimals
LOG_COLUMNS = (
    ('t', 's', 't', 1.0, 0),
    ('h', 'km', 'altitude', 1e-3, 2),
    ('downrange', 'km', 'downrange_x', 1e-3, 1),
    ('v', 'm/s', 'speed', 1.0, 1),
    ('v inertial', 'm/s', 'inertial_speed', 1.0, 1),
    ('v horizontal', 'm/s', 'horizontal_speed', 1.0, 1),
    ('v vertical', 'm/s', 'vertical_speed', 1.0, 1),
    ('gamma', 'deg', 'flight_path_angle', 1.0, 2),
    ("gamma'", 'deg/s', 'flight_path_rate', 1.0, 4),
    ('m', 't', 'mass', 1e-3, 2),
    ('thrust', 'kN', 'thrust', 1e-3, 0),
    ('q', 'kPa', 'dynamic_pressure', 1e-3, 2),
    ('deflection', 'deg', 'steering_angle', 1.0, 2),
    ('steering loss', 'm/s', 'steering_loss', 1.0, 1),
)

BLUE, ORANGE, GREEN = '#1f4fd8', '#e07b00', '#2e7d32'
RED, PURPLE, TEAL = '#c62828', '#6a1b9a', '#00695c'
GUIDE = '#b6bec9'

STYLE = {
    'figure.facecolor': 'white',
    'savefig.facecolor': 'white',
    'axes.edgecolor': '#c3c9d2',
    'axes.labelcolor': '#3d434d',
    'axes.labelsize': 10.5,
    'axes.titlesize': 11,
    'axes.titleweight': 'semibold',
    'axes.titlecolor': '#171a1f',
    'axes.spines.top': False,
    'axes.spines.right': False,
    'font.size': 10,
    'grid.color': '#dfe3e8',
    'legend.fontsize': 9.5,
    'legend.frameon': False,
    'lines.linewidth': 1.9,
    'xtick.color': '#6b7280',
    'ytick.color': '#6b7280',
    'xtick.labelsize': 9.5,
    'ytick.labelsize': 9.5,
}


def write_report(mission: Mission, telemetry: Telemetry,
                 directory: str | Path, command: str = '') -> Path:
    """Write index.html and its plots into `directory`; return the page.

    `command` is the command line that produced the run, shown on the page so
    that it can be typed again. Left empty by a caller that has none.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)

    budget = velocity_budget(telemetry, mission.omega)
    orbit = mission.orbit
    # the style is set for the drawing alone: a caller that plots something
    # of its own afterwards gets its own matplotlib back
    with plt.rc_context(STYLE):
        figures = _plot(mission, telemetry, budget.burnout_time, directory)

    environment = Environment(loader=FileSystemLoader(TEMPLATES),
                              autoescape=select_autoescape(['html']),
                              trim_blocks=True, lstrip_blocks=True)
    page = environment.get_template('report.html').render(
        heading=heading(mission),
        command=command,
        generated=datetime.now().astimezone().strftime('%Y-%m-%d %H:%M'),
        version=_version(),
        status=_status(orbit),
        chips=[mission.pitch_programme.describe(),
               f'cut-off {mission.cutoff.describe()}',
               f'rk4 at {mission.steps_per_second:g} steps/s',
               f'{mission.latitude_deg:g} deg latitude, '
               f'azimuth {mission.azimuth_deg:g} deg'],
        tiles=_tiles(mission, telemetry, budget),
        budget=_budget(budget),
        blocks=_cards(mission, telemetry),
        figures=figures,
        plot_note='The grey vertical lines mark stage separations, the end of '
                  'the pitch programme and engine cut-off. Click a plot for '
                  'the image at full size.',
        log=_log(mission, telemetry),
    )

    path = directory / 'index.html'
    path.write_text(page, encoding='utf-8')
    return path


def _version() -> str:
    from importlib.metadata import PackageNotFoundError, version
    try:
        return version('lv-ascent-model')
    except PackageNotFoundError:  # run from a source tree that was never built
        return ''


# --- the summary ----------------------------------------------------------

# how the orbit reached colours the badge, the tile and the card
ORBIT_COLOURS = {'ok': 'green', 'sub': 'red', 'esc': 'orange'}
# a tile value longer than this is set in the smaller of the two sizes: the
# widest of them carries two numbers and a sign, and has to fit beside seven
# others across the page
VALUE_DIGITS = 8


def _status(orbit) -> dict:
    if not orbit.is_closed:
        return {'label': 'ESCAPE', 'css': 'esc'}
    if not orbit.is_orbit:
        return {'label': 'SUBORBITAL', 'css': 'sub'}
    return {'label': 'ORBIT', 'css': 'ok'}


def _tiles(mission: Mission, telemetry: Telemetry, budget) -> list[dict]:
    """The handful of figures that answer 'did it work' at a glance."""
    orbit = mission.orbit
    cut_off = telemetry.at(budget.burnout_time)
    reached = (f'{orbit.perigee_altitude / 1000:.0f} x '
               f'{orbit.apogee_altitude / 1000:.0f}' if orbit.is_closed else 'open')

    def tile(value, unit, label, css=''):
        # a value wider than the tile is set smaller rather than trimmed:
        # 'perigee x apogee' carries two numbers and a sign
        if len(value) > VALUE_DIGITS:
            css = f'{css} long'.strip()
        return {'value': value, 'unit': unit, 'label': label, 'css': css}

    return [
        tile(f'{mission.target_altitude / 1000:g}', 'km', 'target orbit', 'blue'),
        tile(reached, 'km' if orbit.is_closed else '', 'perigee x apogee',
             ORBIT_COLOURS[_status(orbit)['css']]),
        tile(f'{orbit.eccentricity:.5f}', '', 'eccentricity'),
        tile(f'{budget.burnout_time:.1f}', 's', 'engine cut-off'),
        tile(f'{telemetry.inertial_speed[cut_off]:,.0f}', 'm/s',
             'inertial speed at cut-off', 'teal'),
        tile(f'{budget.total:,.0f}', 'm/s', 'velocity lost on the way', 'amber'),
        tile(f'{telemetry.dynamic_pressure.max() / 1000:.1f}', 'kPa',
             'max dynamic pressure', 'purple'),
        tile(f'{mission.vehicle.payload_mass / 1000:.1f}', 't', 'payload'),
    ]


def _budget(budget) -> dict:
    """The three losses as shares of what they add up to."""
    parts = (('gravity', budget.gravity), ('aerodynamic', budget.aerodynamic),
             ('steering', budget.steering))
    total = budget.total or 1.0
    return {
        'total': f'{budget.total:,.1f}',
        'parts': [{'name': name, 'css': name, 'value': f'{value:,.1f}',
                   'share': f'{100 * value / total:.1f}'}
                  for name, value in parts],
    }


# a card for every block of the summary, in the order the console prints them
CARD_COLOURS = {'setup': 'blue', 'flight': 'teal', 'at cut-off': 'amber',
                'velocity budget': 'purple'}
# a value longer than this is put on a line of its own under its label
VALUE_WIDTH = 26


def _cards(mission: Mission, telemetry: Telemetry) -> list[dict]:
    cards = []
    for block in summary_blocks(mission, telemetry):
        # the block the table has no colour for is the orbit, which takes
        # the colour of the badge: green for one, red for a fall back down
        colour = CARD_COLOURS.get(block.title,
                                  ORBIT_COLOURS[_status(mission.orbit)['css']])
        cards.append({
            'title': block.title,
            'css': colour,
            'rows': [{'label': label, 'value': value,
                      'wide': len(value) > VALUE_WIDTH}
                     for label, value in block.rows],
        })
    return cards


# --- the plots ------------------------------------------------------------

@dataclass(frozen=True)
class _Figure:
    file: str
    title: str
    caption: str
    wide: bool


def _plot(mission: Mission, telemetry: Telemetry, burnout: float,
          directory: Path) -> list[_Figure]:
    t = telemetry.t
    target = mission.target_altitude
    programme = mission.pitch_programme
    figures: list[_Figure] = []
    # instants worth a line on every plot against time
    events = [stage.ignition_time for stage in mission.vehicle.stages[1:]
              if stage.ignition_time <= t[-1]]
    events += [programme.end_time, burnout]

    def draw(name, title, caption, paint, size=PANEL, wide=False, marks=True):
        figure, axes = plt.subplots(figsize=size)
        axes.set_xlabel('t, s')
        if marks:
            for instant in events:
                axes.axvline(instant, color=GUIDE, linewidth=0.9, zorder=0)
        paint(axes)
        _legend(figure, axes)
        axes.grid(linestyle='--', linewidth=0.7, alpha=0.55)
        figure.savefig(directory / f'{name}.png', bbox_inches='tight', dpi=DPI)
        plt.close(figure)
        figures.append(_Figure(f'{name}.png', title, caption, wide))

    def attitude(axes):
        """Programme against flown, in the angle and its two derivatives."""
        flown = (telemetry.flight_path_angle, telemetry.flight_path_rate,
                 np.gradient(telemetry.flight_path_rate, t))
        asked = (np.degrees(programme.angle), np.degrees(programme.rate),
                 np.degrees(programme.acceleration))
        labels = ('flight-path angle, deg', 'rate, deg/s', 'acceleration, deg/s^2')
        for pane, series, demanded, label in zip(axes, flown, asked, labels):
            for instant in events:
                pane.axvline(instant, color=GUIDE, linewidth=0.9, zorder=0)
            # the guidance holds the flown angle on the programme, so the two
            # lie on top of each other while it runs: the programme is laid
            # down as a broad band and the flight drawn over it
            pane.plot(programme.time, demanded, color=ORANGE, linewidth=5.0,
                      alpha=0.35, solid_capstyle='butt',
                      label='asked for by the programme')
            pane.plot(t, series, color=BLUE, linewidth=1.5, label='flown')
            pane.axhline(0.0, color='#c3c9d2', linewidth=0.8)
            pane.set_ylabel(label)
            pane.grid(linestyle='--', linewidth=0.7, alpha=0.55)
        axes[0].legend(loc='upper right')
        axes[-1].set_xlabel('t, s')

    def altitude(axes):
        axes.plot(t, telemetry.altitude / 1000, color=BLUE, label='altitude')
        axes.axhline(target / 1000, color=RED, linestyle=':', label='target')
        axes.set_ylabel('h, km')

    def speed(axes):
        axes.plot(t, telemetry.speed, color=BLUE, label='relative to the Earth')
        axes.plot(t, telemetry.inertial_speed, color=TEAL, label='inertial')
        axes.axhline(circular_velocity(target), color=RED, linestyle=':',
                     label='circular at target')
        axes.set_ylabel('v, m/s')

    def components(axes):
        axes.plot(t, telemetry.speed, color=GUIDE, label='total')
        axes.plot(t, telemetry.horizontal_speed, color=BLUE, label='horizontal')
        axes.plot(t, telemetry.vertical_speed, color=ORANGE, label='vertical')
        axes.axhline(0.0, color='#c3c9d2', linewidth=0.8)
        axes.set_ylabel('v, m/s')

    def propulsion(axes):
        axes.plot(t, telemetry.thrust / 1000, color=BLUE, label='thrust')
        axes.set_ylabel('thrust, kN')
        twin = _twin(axes)
        twin.plot(t, telemetry.mass / 1000, color=PURPLE, label='mass')
        twin.set_ylabel('m, t')

    def acceleration(axes):
        axial = (telemetry.thrust - telemetry.drag) / telemetry.mass
        axes.plot(t, axial / STANDARD_GRAVITY, color=GREEN, label='axial')
        axes.set_ylabel('a, g')

    def pressure(axes):
        axes.plot(t, telemetry.dynamic_pressure / 1000, color=BLUE,
                  label='dynamic pressure')
        design = mission.vehicle.design_dynamic_pressure
        if design:
            axes.axhline(design / 1000, color=RED, linestyle='--', label='design')
        axes.set_ylabel('q, kPa')
        twin = _twin(axes)
        twin.plot(t, telemetry.drag / 1000, color=ORANGE, label='drag')
        twin.set_ylabel('drag, kN')

    def steering(axes):
        axes.plot(t, telemetry.steering_angle, color=BLUE,
                  label='demanded deflection')
        axes.set_ylabel('deflection, deg')
        twin = _twin(axes)
        twin.plot(t, telemetry.steering_loss, color=RED,
                  label='accumulated loss')
        twin.set_ylabel('steering loss, m/s')

    def profile(axes):
        """The ascent in the plane it is flown through, rather than in time."""
        cut_off = telemetry.at(burnout)
        axes.plot(telemetry.inertial_speed, telemetry.altitude / 1000,
                  color=BLUE, label='ascent')
        # the two markers land on each other when the ascent arrives where it
        # was aimed, so the target is a ring and the cut-off sits inside it
        axes.plot(circular_velocity(target), target / 1000, marker='o',
                  markersize=13, markerfacecolor='none', markeredgewidth=2.2,
                  color=RED, label='circular orbit at target')
        axes.plot(telemetry.inertial_speed[cut_off],
                  telemetry.altitude[cut_off] / 1000, marker='o', color=GREEN,
                  label='cut-off')
        axes.set_xlabel('inertial speed, m/s')
        axes.set_ylabel('h, km')

    def trajectory(axes):
        arc = np.linspace(0, np.radians(max(telemetry.polar_angle)) * 1.05, 400)
        axes.plot(EARTH_RADIUS * np.sin(arc) / 1000,
                  (EARTH_RADIUS * np.cos(arc) - EARTH_RADIUS) / 1000,
                  color='0.4', label='surface')
        radius = EARTH_RADIUS + target
        axes.plot(radius * np.sin(arc) / 1000,
                  (radius * np.cos(arc) - EARTH_RADIUS) / 1000,
                  color=RED, linestyle=':', label='target orbit')
        axes.plot(telemetry.downrange_x / 1000, telemetry.downrange_y / 1000,
                  color=BLUE, label='trajectory')
        axes.set_xlabel('downrange, km')
        axes.set_ylabel('height above the pad, km')
        axes.set_aspect('equal')

    # the angle and its two derivatives come first: the programme is what the
    # model exists to compare, and this is the plot of it
    figure, panes = plt.subplots(3, 1, figsize=STACK, sharex=True)
    attitude(panes)
    figure.align_ylabels(panes)
    figure.savefig(directory / 'attitude.png', bbox_inches='tight', dpi=DPI)
    plt.close(figure)
    figures.append(_Figure(
        'attitude.png', 'Angle, angular rate and angular acceleration',
        'What the pitch programme asks for and what the vehicle flew. The '
        'guidance holds the flown angle on the programme until it ends; after '
        'that the vehicle keeps the attitude it reached, and the two part.',
        wide=True))

    draw('altitude', 'Altitude',
         'Height above the pad against time; the dotted line is the target orbit.',
         altitude)
    draw('speed', 'Speed',
         'Relative to the rotating Earth, which is what the air and the '
         'programme see, and inertial, which is what the orbit is built from.',
         speed)
    draw('velocity-components', 'Vertical and horizontal velocity',
         'The vertical component lifts the vehicle out of the atmosphere and '
         'is spent again by gravity; the horizontal one is what stays in orbit.',
         components)
    draw('propulsion', 'Thrust and mass',
         'Thrust steps at separation and rises with altitude as the ambient '
         'pressure falls; the mass falls with what is burned and what is dropped.',
         propulsion)
    draw('acceleration', 'Axial acceleration',
         'Thrust less drag over the mass. It climbs through a burn as the '
         'tanks empty, and drops back at every separation.',
         acceleration)
    draw('dynamic-pressure', 'Dynamic pressure and drag',
         'The load the airframe carries through the lower atmosphere, against '
         'what it is designed for.',
         pressure)
    draw('steering', 'Steering the thrust, and what it cost',
         'The deflection that holding the programme demands, and the velocity '
         'it takes to hold it - the figure a pitch programme is judged by.',
         steering)
    draw('speed-altitude', 'Speed against altitude',
         'The ascent as a path rather than as a history: it has to arrive at '
         'the circular speed exactly at the target altitude.',
         profile, marks=False)
    draw('trajectory', 'Trajectory in the plane of the launch',
         'Drawn to scale over the curve of the Earth, from the pad to the '
         'end of the flight.',
         trajectory, size=SPREAD, wide=True, marks=False)
    return figures


def _twin(axes):
    """A second y axis on the right, with the spine the style takes away."""
    twin = axes.twinx()
    twin.spines['right'].set_visible(True)
    twin.spines['top'].set_visible(False)
    return twin


def _legend(figure, axes) -> None:
    """One legend for the axes and any twin of it."""
    handles, labels = [], []
    for pane in figure.axes:
        pane_handles, pane_labels = pane.get_legend_handles_labels()
        handles += pane_handles
        labels += pane_labels
    if handles:
        axes.legend(handles, labels)


# --- the flight log -------------------------------------------------------

def _log(mission: Mission, telemetry: Telemetry) -> dict:
    stages = mission.vehicle.stages
    rows, previous = [], None
    for instant in np.arange(0.0, telemetry.t[-1] + LOG_INTERVAL, LOG_INTERVAL):
        index = telemetry.at(min(instant, telemetry.t[-1]))
        stage = int(telemetry.stage[index])
        rows.append({
            'cells': [f'{getattr(telemetry, column)[index] * scale:.{digits}f}'
                      for _, _, column, scale, digits in LOG_COLUMNS],
            'mark': (f'now on the {stages[stage].name}'
                     if previous is not None and stage != previous else ''),
        })
        previous = stage
    return {
        'interval': f'{LOG_INTERVAL:g}',
        'columns': [{'name': name, 'unit': unit}
                    for name, unit, _, _, _ in LOG_COLUMNS],
        'rows': rows,
    }
