"""The console summary of a flight: the same figures the HTML report shows.

Everything here is read back from the recorded telemetry, so the summary
describes the run that was actually flown rather than what was asked for.
"""

import numpy as np

from .constants import STANDARD_GRAVITY, circular_velocity
from .losses import velocity_budget
from .mission import Mission
from .telemetry import Telemetry
from .vehicle import LaunchVehicle

LABEL_WIDTH = 24


def summarise(mission: Mission, telemetry: Telemetry) -> str:
    budget = velocity_budget(telemetry, mission.omega)
    cut_off = telemetry.at(budget.burnout_time)
    vehicle = mission.vehicle
    first_stage = vehicle.stages[0]
    rotation = mission.omega * telemetry.radius[0]

    lines = [f'{vehicle.name} to {mission.target_altitude / 1000:g} km']

    _block(lines, 'setup', [
        ('launch site', f'{mission.latitude_deg:g} deg latitude, '
                        f'azimuth {mission.azimuth_deg:g} deg '
                        f'({rotation:.0f} m/s from Earth rotation)'),
        ('pitch programme', mission.pitch_programme.describe()),
        ('engine cut-off', mission.cutoff.describe()),
        ('integration', f'rk4, {mission.steps_per_second:g} steps/s, '
                        f'{mission.duration:g} s of flight'),
        ('lift-off mass', f'{vehicle.lift_off_mass:,.0f} kg'),
        ('payload', f'{vehicle.payload_mass:,.0f} kg'),
        ('thrust to weight', f'{first_stage.thrust_sea_level / (vehicle.lift_off_mass * STANDARD_GRAVITY):.2f}'),
    ])

    acceleration = (telemetry.thrust - telemetry.drag) / telemetry.mass
    peak_q, peak_a = int(np.argmax(telemetry.dynamic_pressure)), int(np.argmax(acceleration))
    design_q = vehicle.design_dynamic_pressure
    _block(lines, 'flight', [
        ('max dynamic pressure',
         f'{telemetry.dynamic_pressure[peak_q] / 1000:.1f} kPa at t = {telemetry.t[peak_q]:.1f} s, '
         f'h = {telemetry.altitude[peak_q] / 1000:.1f} km'
         + (f' (design {design_q / 1000:g} kPa)' if design_q else '')),
        ('max acceleration',
         f'{acceleration[peak_a] / STANDARD_GRAVITY:.2f} g at t = {telemetry.t[peak_a]:.1f} s'),
        ('stage separation', ', '.join(f'{s.ignition_time:g} s' for s in vehicle.stages[1:]) or 'none'),
        ('engine cut-off', f'{budget.burnout_time:.1f} s'),
        ('target altitude reached', _first_time(telemetry, telemetry.altitude, mission.target_altitude)),
        ('orbital speed reached',
         _first_time(telemetry, telemetry.inertial_speed,
                     circular_velocity(mission.target_altitude))),
    ])

    remaining = _propellant_left(vehicle, telemetry, cut_off)
    _block(lines, 'at cut-off', [
        ('altitude', f'{telemetry.altitude[cut_off] / 1000:.2f} km'),
        ('speed, relative', f'{telemetry.speed[cut_off]:.1f} m/s'),
        ('speed, inertial', f'{telemetry.inertial_speed[cut_off]:.1f} m/s'),
        ('flight-path angle', f'{telemetry.flight_path_angle[cut_off]:.3f} deg'),
        ('downrange angle', f'{telemetry.polar_angle[cut_off]:.2f} deg'),
        ('mass', f'{telemetry.mass[cut_off]:,.0f} kg'),
        ('propellant left', remaining),
    ])

    orbit = mission.orbit
    if orbit.is_closed:
        _block(lines, 'orbit', [
            ('perigee', f'{orbit.perigee_altitude / 1000:.2f} km'),
            ('apogee', f'{orbit.apogee_altitude / 1000:.2f} km'),
            ('eccentricity', f'{orbit.eccentricity:.5f}'),
            ('period', f'{orbit.period / 60:.1f} min'),
            ('circularisation dv', f'{orbit.circularisation_dv:.1f} m/s'),
            ('closes an orbit', 'yes' if orbit.is_orbit else 'no, perigee below the surface'),
        ])
    else:
        _block(lines, 'orbit', [('eccentricity', f'{orbit.eccentricity:.5f} - not a closed orbit')])

    _block(lines, 'velocity budget', [
        ('gravity loss', f'{budget.gravity:.1f} m/s'),
        ('aerodynamic loss', f'{budget.aerodynamic:.1f} m/s'),
        ('steering loss', f'{budget.steering:.1f} m/s'),
        ('total', f'{budget.total:.1f} m/s'),
        ('steering demand', _demand(telemetry, mission.pitch_programme.end_time)),
    ])
    return '\n'.join(lines)


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
