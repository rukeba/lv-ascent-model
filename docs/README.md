# Documentation

The reasoning behind the model, kept out of the source so that the source
stays short enough to be read on paper. Each page is small and stands on its
own; the code points at the page that explains it.

## The model

- [The model](model.md) - frame, forces, guidance, what is and is not in it
- [Constants of the Earth model](constants.md) - and why `MU` is measured
- [Atmosphere and drag](atmosphere.md) - ICAO layers, the drag ceiling
- [Integration](integration.md) - Runge-Kutta, and why events matter more
- [The velocity budget](velocity-budget.md) - gravity, aerodynamic, steering
- [Control effort](control-effort.md) - the second measure, `J`

## The pitch programmes

- [Pitch programmes](pitch-programmes.md) - what they prescribe, and how
- [Five-phase turn](pitch-five-phase.md)
- [Velocity share](pitch-velocity-share.md)
- [Bilinear tangent](pitch-bilinear-tangent.md)
- [What the three cost](programme-comparison.md)

## Configuration and data

- [Configuration files](configuration.md) - mission and vehicle YAML
- [The vehicles](vehicles.md) - Falcon 9, Ariane 62, H3-22S
- [The catalogue](catalogue.md) - what a solved parameter set records
- [What the catalogue is missing](catalogue-gaps.md)

## The search

- [Searching for a parameter set](search.md) - and what a set is judged by
- [The grid](search-grid.md) - axes, ranges, what each count buys
- [The two estimates](search-estimates.md) - what keeps a search affordable
- [Closing in](search-refinement.md) - passes, valleys, the step ramp
- [What a search costs](search-cost.md) - measurements, and how to spend less

## Running and checking

- [The commands](cli.md) - `ascent` and `ascent-search`
- [Summary and report](report.md) - the console page and the HTML one
- [Tests](tests.md) - what is checked, and against what
- [Implementation notes](performance.md) - why some code is written oddly
