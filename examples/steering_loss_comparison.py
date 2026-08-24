"""Compare the three pitch programmes by what they cost in velocity.

Falcon 9 into a 500 km circular orbit from Cape Canaveral, due east. Each
programme is flown with the parameters that minimise its steering loss subject
to reaching that orbit, and the resulting velocity budget is printed next to
the published figures, so the agreement can be read off directly.

    uv run python examples/steering_loss_comparison.py
"""

from ascent import (BilinearTangentProgramme, CutoffAtTime, FivePhaseProgramme,
                    Mission, VelocityShareProgramme, load_vehicle, velocity_budget)

TARGET_ALTITUDE = 500_000
LATITUDE, AZIMUTH = 28.5, 90.0

# programme, cut-off time, published (gravity, aerodynamic, steering) loss, m/s
CASES = (
    (FivePhaseProgramme(t1=20.0, t4=502.8, k2=0.056178, k3=0.522859),
     502.8, (2568.8, 29.3, 526.4)),
    (VelocityShareProgramme(t1=20.0, tf=491.691775, te=502.1492, s=0.995106),
     502.1492, (2538.0, 29.7, 411.0)),
    (BilinearTangentProgramme(t1=20.0, a=-1.097246, b=527.99193, c=1.927467, te=501.2),
     501.2, (2500.0, 29.6, 433.0)),
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
    return mission, velocity_budget(mission.run(), mission.omega)


def main() -> None:
    print(f'Falcon 9 into a {TARGET_ALTITUDE / 1000:g} km circular orbit, '
          f'{LATITUDE} deg latitude, azimuth {AZIMUTH:g} deg\n')
    print(f'{"programme":<22}{"gravity":>10}{"aerodynamic":>13}{"steering":>10}'
          f'{"total":>10}{"perigee":>10}{"apogee":>9}')

    worst = 0.0
    for programme, cutoff_time, published in CASES:
        mission, budget = run(programme, cutoff_time)
        name = programme.describe().split('(')[0]
        orbit = mission.orbit
        print(f'{name:<22}{budget.gravity:>10.1f}{budget.aerodynamic:>13.1f}'
              f'{budget.steering:>10.1f}{budget.total:>10.1f}'
              f'{orbit.perigee_altitude / 1000:>10.1f}{orbit.apogee_altitude / 1000:>9.1f}')
        print(f'{"  published":<22}{published[0]:>10.1f}{published[1]:>13.1f}'
              f'{published[2]:>10.1f}{sum(published):>10.1f}')
        worst = max(worst, *(abs(a - b) for a, b in zip(
            (budget.gravity, budget.aerodynamic, budget.steering), published)))

    print(f'\nlargest deviation from the published figures: {worst:.2f} m/s')


if __name__ == '__main__':
    main()
