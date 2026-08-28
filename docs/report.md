# Summary and report

## The console summary

[`summary.py`](../src/ascent/summary.py). Lists the set-up, the notable instants
of the flight, the state at engine cut-off, the orbit reached and [the velocity
budget](velocity-budget.md).

The steering demand is summarised only over the stretch where the programme
runs: there is no demand to meet once the vehicle holds the attitude it reached,
and rows past the handover would dilute the share with zeros that mean nothing.
Where the demand saturated - see [control effort](control-effort.md) - the share
of the burn that it did is spelled out rather than rounded, since a nought is
what the line says when nothing saturated at all.

The search summary prints the same kind of thing about a grid: every axis with
its range and its step, the two estimates, what became of every node, and the
table of sets found. Two details it takes from the result rather than working
out again:

- **the valleys followed** is what the ranking actually offered, not what was
  asked for. A search that only ever found one valley says so rather than
  claiming to have looked around five.
- **the integration step of each pass**, where that is not one figure
  throughout. A reader who sees a search cost less than the last one is owed the
  reason, and a search that stopped after its coarse sweep would otherwise be
  read as having run at the step it never got to.

## The HTML report

[`report.py`](../src/ascent/report.py) and
[`templates/`](../src/ascent/templates), rendered with Jinja.

`--report` writes the console figures as a page - laid out as cards, with the
velocity budget drawn to scale - and adds ten plots and the trajectory tabulated
every five seconds.

The plots are PNG files beside the page, drawn well above screen resolution so
that they stay sharp when opened full size or printed. The styles are inlined,
so the page can be sent on with the images next to it.

It is opened in a browser as soon as it is written, which `--no-open`
suppresses. Given no directory it writes to `out/` and the name of the vehicle
file, so `ascent f9 --report` lands in `out/f9`. The page carries the command
that produced it, so a report found months later says how to make it again.

The plot of the pitch programme comes first: the programme is what the model
exists to compare. The flown angle lies on top of the commanded one while the
guidance runs, so the programme is laid down as a broad band and the flight
drawn over it.

The trajectory plot is drawn over the Earth itself, to scale, with the surface
and the target orbit as circles about the centre - taking them as a height over
the downrange, rather than as an arc over an angle, is what makes each of them
span the axis exactly.

![Falcon 9 into a 500 km circular orbit, drawn to scale over the curve of the Earth](trajectory.png)
