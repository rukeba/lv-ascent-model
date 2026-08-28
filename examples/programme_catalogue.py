"""Print the catalogue of solved pitch-programme parameters as a table.

One row per vehicle, pitch programme and target altitude: the parameters that
insert that vehicle into that circular orbit, and what the ascent costs in
velocity.

    uv run python examples/programme_catalogue.py

See docs/catalogue.md
"""

from ascent.config import load_catalogue

SHOWN = ('t1', 't4', 'k2', 'k3', 'tf', 'te', 's', 'a', 'b', 'c')


def parameters(programme: dict) -> str:
    return ', '.join(f'{key}={programme[key]:g}' for key in SHOWN if key in programme)


def main() -> None:
    catalogue = load_catalogue('config')

    for vehicle in dict.fromkeys(spec['vehicle'] for spec in catalogue):
        entries = [spec for spec in catalogue if spec['vehicle'] == vehicle]
        print(f'\n{vehicle}\n')
        print(f'{"orbit":>9}  {"programme":<17}{"cut-off":>9}'
              f'{"gravity":>9}{"aero":>7}{"steering":>10}{"total":>9}   parameters')
        for spec in sorted(entries, key=lambda s: (s['target_altitude'],
                                                   s['pitch_programme']['type'])):
            reached = spec['reached']
            print(f'{spec["target_altitude"] / 1000:>6g} km  '
                  f'{spec["pitch_programme"]["type"]:<17}'
                  f'{spec["cutoff"]["time"]:>8.1f}s'
                  f'{reached["gravity_loss"]:>9.1f}{reached["aerodynamic_loss"]:>7.1f}'
                  f'{reached["steering_loss"]:>10.1f}{reached["total_loss"]:>9.1f}   '
                  f'{parameters(spec["pitch_programme"])}')
    print('\nlosses in m/s; every entry inserts into a circular orbit '
          'at its target altitude')


if __name__ == '__main__':
    main()
