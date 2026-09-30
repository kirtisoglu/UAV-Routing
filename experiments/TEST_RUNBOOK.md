# Test runbook: sign tiers instead of the exchange value for Swap and 2-opt

For the local agent. This is an experiment, not a change of the design: nothing in
the paper is edited, the reference results stay, and the outcome is written to
`experiments/DESIGN.md` under a dated heading. Rules 1, 2, 5 and 6 of
`experiments/RERUN_PLAN.md` apply (root of the repository, one run at a time, no
`ILS_LICENSE_GUARD`, report rather than fix).

## What is being tested

The paper's reordering weight is the exchange value x(p, q) = (γ_u − γ_v)(a_q − a_p),
the L_r = 20 largest entering the set and the draw being a roulette over the shifted
values. The test replaces this by a rule that needs no arrival times:

* Swap ⟨p, q⟩, q > p: tier 2 if the target at p has a positive slope and the target at
  q a negative one, tier 0 if the signs are the other way round, tier 1 otherwise
  (same signs, or a zero slope). The draw is uniform among the moves of the highest
  tier still in the set; when they are exhausted the next tier follows. Feasibility
  checks are unchanged.
* 2-opt ⟨p, q⟩, two candidate rules. `signs`: the tier of the outermost pair (p, q),
  which is the pair with the largest separation of arrival times and so the dominant
  term of the paper's sum S(p, q). `signsm`: majority vote over the nested pairs
  (p, q), (p+1, q−1), ...: tier 2 if more pairs have the (+, −) signs than the (−, +)
  signs, tier 0 if fewer, tier 1 on a tie. Swap is identical under both.
* The list length L_r still applies: when a set holds more than L_r moves, the L_r
  kept are the highest tiers first, in random order within a tier. A run with
  `DESIGN_RCL=0` removes the cap.

A third variant keeps the exchange value's form but changes what it reads:

* `mid`: the paper's x(p, q) reads both targets at the earliest arrivals a^min of
  their old positions and holds those times fixed. `mid` reads each of the two
  exchanged targets at the midpoint of its realized window, before the move (its old
  position in the current route) and after the move (its new position in the
  reordered route, with the new legs), and takes the change of the two rewards. The
  new windows come from the segment summaries of Section 4.3 in O(1), so the cost per
  pair is unchanged. For a 2-opt only the outer pair (r_p, r_q) is read this way; the
  nested pairs keep the paper's value. The draw (roulette over the shifted values)
  and the list length L_r are unchanged, so this isolates the effect of the reading.
* `midall`: the reading of `mid` extended to every target of the reordered part. For
  a 2-opt that is every target of the reversed segment (r_q, ..., r_p); for a Swap it
  is r_q, r_p and the interior r_{p+1}, ..., r_{q-1}, whose arrivals the new legs
  shift. The score is the sum over those targets of the slope times the change of the
  midpoint of the realized window. The windows after the move come from tables of
  nested segment summaries (every forward interior segment for Swap, every reversed
  segment for 2-opt, filled once per route), so a target costs O(1) and a feasible
  pair O(q - p) instead of O(1). Targets before and after the reordered part are still
  not read; reading them too would be the route-wide estimate the paper rejected for
  reorderings. Checked against a full recomputation of the arrival chain on every
  feasible Swap and 2-opt of the best routes of nine instances (2352 moves, largest
  difference 4e-12), with feasible sets identical to the default builders'. Building
  the Swap and 2-opt sets of those best routes takes C104 (100) 4.9 against 11.4 ms, PR15 (240) 6.8 against 7.2 ms, PR10 (288) 3.3 against 3.4 ms (default against `midall`).
* `route`: the insertion's weight, applied to reorderings exactly as to Insert and
  Replace. The information part is the route-wide change of the estimate of Section
  4.3 (labels `eq:info-mid` and `eq:info-change`) on the reordered route: every target
  of the new route, each read at the midpoint of its realized window, which is how the
  insertion weight reads it. The weight is that change divided by the length change
  when the move lengthens the route by more than 1e-9 m, and the change itself
  otherwise, clamped at zero and drawn by the plain roulette, uniform when no move has
  a positive weight: the insertion code's rule, fallback included. The fallback matters
  here: 7 percent of the feasible reorderings on the best routes of the reference run
  do not lengthen the route, and one that also gains information gets its raw gain as
  weight, which on the meter scale dwarfs every quotient, so such moves are drawn
  first. L_r still trims the Swap and 2-opt sets, now by this weight. Checked against a
  full recomputation of the arrival chain on every feasible Swap and 2-opt of those
  routes (2352 moves, largest relative difference 3e-13). Building the sets takes
  C104 (100) 4.7 against 20.1 ms, PR15 (240) 6.5 against 7.5 ms, PR10 (288) 3.0 against 3.7 ms (default against `route`).
* `routebest`: `route` with each target read at the end of its realized window that
  gives the most information, the upper end I^+ of Section 4.3 (label
  `eq:info-bounds`): the earliest arrival when its slope is negative, the latest
  otherwise, in place of the midpoint. The route-wide change of that value is taken for
  all four operators, so unlike `route` the Insert and Replace weights change as well;
  the weight is the one of `route` (the change over the length change when the move
  lengthens the route, the change itself otherwise, clamped at zero, plain roulette).
  Checked against a full recomputation of the arrival chain on every feasible move of
  the four operators on the best routes of nine instances (3633 moves, largest
  relative difference 6e-12).

The driver knob is `DESIGN_REORDER_W` (default `exch`, the paper); the value is
recorded in the `extra` column of every row as `reorder=signs`, `reorder=signsm`,
`reorder=mid`, `reorder=midall`
or `reorder=route`.

## Runs, about 3.5 hours in total

The six instances where reorderings matter (the L_r grid set of Section 5.8):

    G="PR11 (48);R104 (100);RC104 (100);C104 (100);PR15 (240);PR10 (288)"

    DESIGN_INSTANCES="$G" DESIGN_REORDER_W=signs                 DESIGN_OUT=_signs      python3 paper_runs/run_design.py new
    DESIGN_INSTANCES="$G" DESIGN_REORDER_W=signsm                DESIGN_OUT=_signsm     python3 paper_runs/run_design.py new
    DESIGN_INSTANCES="$G" DESIGN_REORDER_W=signs  DESIGN_RCL=0   DESIGN_OUT=_signs_Lall python3 paper_runs/run_design.py new
    DESIGN_INSTANCES="$G" DESIGN_REORDER_W=mid                   DESIGN_OUT=_mid        python3 paper_runs/run_design.py new
    DESIGN_INSTANCES="$G" DESIGN_REORDER_W=midall                DESIGN_OUT=_midall     python3 paper_runs/run_design.py new
    DESIGN_INSTANCES="$G" DESIGN_REORDER_W=route                 DESIGN_OUT=_route      python3 paper_runs/run_design.py new

The reference is replication 1, `paper_runs/results/design_new.csv` (and
`design_new_rep2.csv` if Step 9 of the runbook has run, which says how large the
seed-to-seed noise is on these instances). Do not rerun the reference.

Sanity check before the six-instance runs (seconds):

    DESIGN_INSTANCES="R101 (50)" DESIGN_REORDER_W=signs DESIGN_OUT=_signs_check python3 paper_runs/run_design.py new

Expected: `stop=max_idle_shakes`, `extra` reads `reorder=signs`, an objective near
11 902.02 (R101 (50) has almost no feasible reorderings, so the rule barely acts).

## Added 30 September: `routebest`, and `route` completed on this machine

The `route` runs on this machine covered C104, PR11 and PR15; its numbers for R102,
RC104 and R1_2_1 came from the cloud container, whose license caps routes at 32
targets. Run both variants on the same six instances here:

    R="R102 (100);RC104 (100);R1_2_1 (200);C104 (100);PR11 (48);PR15 (240)"
    DESIGN_INSTANCES="$R" DESIGN_REORDER_W=route     DESIGN_OUT=_route     python3 paper_runs/run_design.py new
    DESIGN_INSTANCES="$R" DESIGN_REORDER_W=routebest DESIGN_OUT=_routebest python3 paper_runs/run_design.py new
    python3 paper_runs/compare_runs.py paper_runs/results/design_new_route.csv     paper_runs/results/design_new.csv
    python3 paper_runs/compare_runs.py paper_runs/results/design_new_routebest.csv paper_runs/results/design_new.csv
    python3 paper_runs/compare_runs.py paper_runs/results/design_new_routebest.csv paper_runs/results/design_new_route.csv

The first command resumes `design_new_route.csv` and runs only the three instances it
lacks. About 25 minutes in total at the reference run times; C104 and PR15 dominate.
Report the three comparisons and the `sets_pct` column of both CSVs.

## What to report

    python3 paper_runs/compare_runs.py paper_runs/results/design_new_signs.csv      paper_runs/results/design_new.csv
    python3 paper_runs/compare_runs.py paper_runs/results/design_new_signsm.csv     paper_runs/results/design_new.csv
    python3 paper_runs/compare_runs.py paper_runs/results/design_new_signs_Lall.csv paper_runs/results/design_new.csv
    python3 paper_runs/compare_runs.py paper_runs/results/design_new_mid.csv        paper_runs/results/design_new.csv
    python3 paper_runs/compare_runs.py paper_runs/results/design_new_midall.csv     paper_runs/results/design_new.csv
    python3 paper_runs/compare_runs.py paper_runs/results/design_new_route.csv      paper_runs/results/design_new.csv

Write into `experiments/DESIGN.md`, under a heading with the date and the commit, one
table with a row per instance and variant: objective, change in percent against
replication 1, t_best, Run,
shakes and `sets_pct`, the share of the run spent building the move sets (all from
the CSVs; pull before running, since a CSV started earlier lacks that column), and, if `design_new_rep2.csv`
exists, the replication-1 to replication-2 difference on the same instance as the
noise reference. Then three sentences: whether the sign rule is within noise of the
exchange value on the objective, what it does to the run time and the shake count,
whether `signs` or `signsm` is the better 2-opt rule, whether `mid` moves the
objective against the paper's reading in either direction, and whether `midall` adds
anything to `mid` beyond its cost in run time, and whether `route`, the insertion's rule
as a whole, does better than the paper's exchange value. No recommendation beyond
that; the author decides.

## What must not happen

* No change to the paper, to `design_new.csv`, or to any file the tables read.
* No tuning of L_r, S or D inside this test; the three commands above are the test.
* If a run fails or prints `[warn]`, keep the console output, note it in `DESIGN.md`,
  and stop.
