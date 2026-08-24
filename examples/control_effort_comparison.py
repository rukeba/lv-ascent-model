"""Compare the three pitch programmes by how hard they work the guidance.

Falcon 9 into a 500 km circular orbit from Cape Canaveral, due east, with the
same three parameter sets as `steering_loss_comparison.py`. That script prices
the programmes in propellant: the steering loss is the share of the thrust that
holding the programme points away from the velocity. This one prices them in
control: the functional

    J = integral of a_control^2 dt over the powered flight,  m^2/s^3

where `a_control` is the normal acceleration the guidance has to produce. The
square charges an abrupt stretch more than an even one, so two programmes that
cost the same propellant are still told apart by it.

The two accumulations are drawn side by side, because where they part is the
point: the loss grows wherever the thrust is deflected at all, the effort grows
sharply wherever the programme swings the flight-path angle.

    uv run python examples/control_effort_comparison.py [figure.png]
"""

import sys

import matplotlib.pyplot as plt

from ascent import (BilinearTangentProgramme, CutoffAtTime, FivePhaseProgramme,
                    Mission, VelocityShareProgramme, load_vehicle, velocity_budget)
from ascent.report import BLUE, DPI, GREEN, GUIDE, ORANGE, STYLE

TARGET_ALTITUDE = 500_000
LATITUDE, AZIMUTH = 28.5, 90.0
FIGURE = 'out/control-effort.png'

# programme, cut-off time, colour
CASES = (
    (FivePhaseProgramme(t1=20.0, t4=502.8, k2=0.056178, k3=0.522859), 502.8, BLUE),
    (VelocityShareProgramme(t1=20.0, tf=491.691775, te=502.1492, s=0.995106),
     502.1492, ORANGE),
    (BilinearTangentProgramme(t1=20.0, a=-1.097246, b=527.99193, c=1.927467, te=501.2),
     501.2, GREEN),
)


def run(programme, cutoff_time):
    mission = Mission(
        vehicle=load_vehicle('config/lv.f9.yaml'),
        pitch_programme=programme,
        cutoff=CutoffAtTime(cutoff_time),
        target_altitude=TARGET_ALTITUDE,
        duration=600.0,
        steps_per_second=10,
        latitude_deg=LATITUDE,
        azimuth_deg=AZIMUTH,
    )
    return mission, mission.run()


def draw(flights, path):
    """The two accumulations against time, one panel each."""
    with plt.rc_context(STYLE):
        figure, (left, right) = plt.subplots(1, 2, figsize=(12.6, 4.6))
        for name, colour, telemetry, separation, handover in flights:
            for panel, series in ((left, telemetry.steering_loss),
                                  (right, telemetry.control_effort)):
                panel.plot(telemetry.t, series, color=colour, label=name)
                # the separation and the end of the programme, which is where
                # the curves are read against each other
                for instant in (separation, handover):
                    panel.axvline(instant, color=GUIDE, linewidth=0.9, zorder=0)

        left.set_title('Steering loss, what it costs')
        left.set_ylabel('velocity lost to steering, m/s')
        right.set_title('Control effort, how hard it is worked')
        right.set_ylabel('J, m^2/s^3')
        for panel in (left, right):
            panel.set_xlabel('t, s')
            panel.grid(linestyle='--', linewidth=0.7, alpha=0.55)
        left.legend(loc='upper left')

        figure.savefig(path, bbox_inches='tight', dpi=DPI)
        plt.close(figure)


def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else FIGURE
    print(f'Falcon 9 into a {TARGET_ALTITUDE / 1000:g} km circular orbit, '
          f'{LATITUDE} deg latitude, azimuth {AZIMUTH:g} deg\n')
    print(f'{"programme":<22}{"steering":>10}{"effort":>12}{"peak demand":>13}')

    flights = []
    for programme, cutoff_time, colour in CASES:
        mission, telemetry = run(programme, cutoff_time)
        budget = velocity_budget(telemetry, mission.omega)
        name = programme.describe().split('(')[0]
        effort = telemetry.control_effort[-1]
        peak = abs(telemetry.steering_demand).max()
        print(f'{name:<22}{budget.steering:>10.1f}{effort:>12.0f}{peak:>13.3f}')
        flights.append((name, colour, telemetry,
                        mission.vehicle.stages[1].ignition_time,
                        programme.end_time))

    print(f'\nsteering in m/s, effort in m^2/s^3, demand in units of the 1.0 '
          f'the thrust can give')
    draw(flights, path)
    print(f'figure written to {path}')


if __name__ == '__main__':
    main()
