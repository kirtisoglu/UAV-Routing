# Animation data from the pre-24-September-2026 algorithm

Route and move traces, packed viewer data and built pages for all fourteen
instances, recorded from ten-minute runs of the ILS as it stood before the
24 September 2026 change round.

Superseded because every run here predates:

* the **evaluated-route store fix** — a candidate whose route had been evaluated
  anywhere in the run was dropped from the move set, so the search could never
  move to it even when it would improve. About 14% of draws were discarded this
  way, so these trajectories are not the ones the current code produces;
* the **false-infeasible fix** — routes the taut-string energy screen certifies
  feasible were recorded as infeasible when the solver failed on them;
* the **new move weight** — information change read from the realized windows
  over the whole route, all reorderings kept in their sets, and roulette weights
  scaled onto a positive range.

The move traces here also carry the old column set, without `w_chosen` and
`w_sum`, so the candidate panel of a page built from them shows raw weights and
cannot form selection probabilities. `animation/build_viewer_data.py` now
expects the new columns.

Kept for comparison against the new runs, not for the paper.
