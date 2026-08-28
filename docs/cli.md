# The commands

Two entry points, in [`cli.py`](../src/ascent/cli.py).

`f9`, `a62` and `h3` are short names for `config/mission.<name>.yaml`, and the
three pitch programmes answer to `5f`, `vs` and `bt` as well as to their full
names. `--altitude` is `-a` and `--programme` is `-p` on both commands.

Both let Ctrl+C end them without a stack trace, and report the exit status a
shell reports for a program stopped by an interrupt.

## `ascent` - fly one mission

```sh
uv run ascent f9                            # summary of the flight on the console
uv run ascent f9 --csv out/f9.csv           # and the whole trajectory as CSV
uv run ascent f9 --report                   # an HTML report in out/f9, opened
uv run ascent f9 --report out/run-12        # or wherever it is wanted
uv run ascent config/mission.a62.yaml       # a mission file by path

uv run ascent f9 --list                     # solved parameter sets on file
uv run ascent f9 --altitude 600             # fly one of them
uv run ascent f9 --altitude 600 --programme bilinear-tangent
uv run ascent f9 -a 600 -p bt               # the same, in short
```

| option | |
|---|---|
| `--altitude`, `-a` | fly the catalogue entry for this target altitude, in km |
| `--programme`, `-p` | which pitch programme to take from the catalogue |
| `--list` | list the catalogue entries for this vehicle and stop |
| `--csv FILE` | write the whole trajectory to a CSV file |
| `--report [DIR]` | write an HTML report with plots and open it |
| `--no-open` | write the report without opening it |
| `--config-dir DIR` | where short mission names and the catalogue are looked up |

`--list` prints the tolerance each entry was accepted at beside it. Not every
entry was asked for the same thing, and a listing that left it out would show
the two alike - see [the catalogue](catalogue.md).

## `ascent-search` - search for a parameter set

```sh
uv run ascent-search f9 --altitude 500      # search for a set instead of flying one
uv run ascent-search f9 -a 500 -p 5f --dry-run           # the grid, before it is flown
uv run ascent-search f9 -a 500 -p 5f --range t1=10:25:10 # one parameter, my way
uv run ascent-search f9 -a 500 -p 5f --range k2=0.05     # or held at one value
uv run ascent-search a62 --altitude 700 --yaml           # as a catalogue entry
uv run ascent-search f9 --altitude 500 --report          # fly it and report it
uv run ascent-search f9 --altitude 500 --csv out/sets.csv  # every set found
uv run ascent-search h3 --altitude 1100 --coarse 0.5     # a quicker, rougher sweep
uv run ascent-search f9 --altitude 500 --workers 1       # in this process alone
```

The mission file supplies the vehicle and the launch site, and its own target
altitude and programme type stand in for `--altitude` and `--programme` when
those are not given. The pitch-programme parameters in it are ignored: they are
what the search is for.

| option | |
|---|---|
| `--range NAME=LOW:HIGH:VALUES` | what one parameter is searched over, repeatable. See [the grid](search-grid.md) |
| `--tolerance KM` | how close the perigee, the apogee and the altitude at cut-off have to come |
| `--speed-tolerance M_S` | and how close the speed at cut-off has to |
| `--refinements N` | passes after the sweep. See [closing in](search-refinement.md) |
| `--basins N` | places on the grid the passes close in on at once |
| `--max-q KPA` | put the airframe into the constraint |
| `--coarse FACTOR` | scale the nodes along every axis the family gave |
| `--no-screen` | fly every node rather than screening on the altitude integral |
| `--steps PER_SECOND` | integration steps per second of every trajectory flown |
| `--workers N` | processes the nodes of a pass are divided over |
| `--top N` | how many of the sets found to print |
| `--dry-run` | print the grid and what the passes come to, then stop |
| `--csv FILE` | write every set found, best first |
| `--yaml` | print the set found as a catalogue entry |
| `--report [DIR]`, `--no-open` | fly the set found and write the report |

`--yaml` prints the set found as a catalogue entry, ready to paste; `--report`
turns it into that same entry, flies it and writes the page `ascent --report`
writes, so a set that was searched for and one that was filed give the same
report. A set that misses the orbit is not an entry: it is printed, and it is
not flown.

With `--yaml` the entry is the whole of what the command is for, so everything
else goes to the error stream and a redirect of the output is a file the
catalogue reader can read.

## What a search prints, and when

**The grid comes first**, before anything is flown: the orbit asked for, what
the two estimates said, and every parameter of the family with the range and the
number of values it is searched over. That is minutes of integration described
in fifteen lines, and it is worth reading while there is still time to stop and
narrow it. `--dry-run` prints exactly that and stops there of its own accord.

**Then the progress line**, rewritten in place: which pass it is on, how many
trajectories it has integrated and roughly how much longer it will take.

**Then** what became of every node, the table, and the set found.
