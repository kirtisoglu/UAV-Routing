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

The driver knob is `DESIGN_REORDER_W` (default `exch`, the paper); the value is
recorded in the `extra` column of every row as `reorder=signs`, `reorder=signsm`,
`reorder=mid` or `reorder=midall`.

## Runs, about 3 hours in total

The six instances where reorderings matter (the L_r grid set of Section 5.8):

    G="PR11 (48);R104 (100);RC104 (100);C104 (100);PR15 (240);PR10 (288)"

    DESIGN_INSTANCES="$G" DESIGN_REORDER_W=signs                 DESIGN_OUT=_signs      python3 paper_runs/run_design.py new
    DESIGN_INSTANCES="$G" DESIGN_REORDER_W=signsm                DESIGN_OUT=_signsm     python3 paper_runs/run_design.py new
    DESIGN_INSTANCES="$G" DESIGN_REORDER_W=signs  DESIGN_RCL=0   DESIGN_OUT=_signs_Lall python3 paper_runs/run_design.py new
    DESIGN_INSTANCES="$G" DESIGN_REORDER_W=mid                   DESIGN_OUT=_mid        python3 paper_runs/run_design.py new
    DESIGN_INSTANCES="$G" DESIGN_REORDER_W=midall                DESIGN_OUT=_midall     python3 paper_runs/run_design.py new

The reference is replication 1, `paper_runs/results/design_new.csv` (and
`design_new_rep2.csv` if Step 9 of the runbook has run, which says how large the
seed-to-seed noise is on these instances). Do not rerun the reference.

Sanity check before the six-instance runs (seconds):

    DESIGN_INSTANCES="R101 (50)" DESIGN_REORDER_W=signs DESIGN_OUT=_signs_check python3 paper_runs/run_design.py new

Expected: `stop=max_idle_shakes`, `extra` reads `reorder=signs`, an objective near
11 902.02 (R101 (50) has almost no feasible reorderings, so the rule barely acts).

## What to report

    python3 paper_runs/compare_runs.py paper_runs/results/design_new_signs.csv      paper_runs/results/design_new.csv
    python3 paper_runs/compare_runs.py paper_runs/results/design_new_signsm.csv     paper_runs/results/design_new.csv
    python3 paper_runs/compare_runs.py paper_runs/results/design_new_signs_Lall.csv paper_runs/results/design_new.csv
    python3 paper_runs/compare_runs.py paper_runs/results/design_new_mid.csv        paper_runs/results/design_new.csv
    python3 paper_runs/compare_runs.py paper_runs/results/design_new_midall.csv     paper_runs/results/design_new.csv

Write into `experiments/DESIGN.md`, under a heading with the date and the commit, one
table with a row per instance and variant: objective, change in percent against
replication 1, t_best, Run, shakes (all from the CSVs), and, if `design_new_rep2.csv`
exists, the replication-1 to replication-2 difference on the same instance as the
noise reference. Then three sentences: whether the sign rule is within noise of the
exchange value on the objective, what it does to the run time and the shake count,
whether `signs` or `signsm` is the better 2-opt rule, whether `mid` moves the
objective against the paper's reading in either direction, and whether `midall` adds
anything to `mid` beyond its cost in run time. No recommendation beyond
that; the author decides.

## What must not happen

* No change to the paper, to `design_new.csv`, or to any file the tables read.
* No tuning of L_r, S or D inside this test; the three commands above are the test.
* If a run fails or prints `[warn]`, keep the console output, note it in `DESIGN.md`,
  and stop.
