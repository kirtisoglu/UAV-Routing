# Runbook: fill the matheuristic tables of Section 5

For the local agent. Read this file to the end before running anything; it replaces
every earlier plan. The design of Section 4 is final and is the default of
`paper_runs/run_design.py`. Nothing is tuned, nothing is compared, nothing is
redesigned here: the job is to run the listed commands, let the scripts write the
tables and figures, and report.

## What is where

| Thing | Location |
|---|---|
| The paper (single source) | `paper/ArXiv-version.tex`, figures in `paper/fig/`, bibliography `paper/ref.bib` |
| The matheuristic | `experiments/run_ils_time_matched.py` with `experiments/fast_sets.py`, `run_ils_final.py`, `run_ils_final_scored.py`, `run_ils_fb_cascade_demo.py` and the `uav_routing` package |
| The driver (every ILS number) | `paper_runs/run_design.py`; outputs `paper_runs/results/design_new<SUFFIX>.csv` and traces under `paper_runs/results/details/design_traces/` |
| The table writer | `paper_runs/fill_tables.py` (writes the table bodies into the tex from the CSVs) |
| The figures | `paper_runs/make_convergence_figure.py`, `paper_runs/make_capdiv_figure.py` (write into `paper/fig/`) |
| Comparing two runs | `paper_runs/compare_runs.py a.csv b.csv` |
| What each result file is | `paper_runs/results/README.md` |
| MISOCP tables (5.3 to 5.6) | done; `misocp_s1.csv`, `misocp_seeds.csv`, `slope_regimes.csv`, `eta_sweep.csv`, `loitering.csv`. Do not rerun. |

## Rules

1. Work in the repository root, on the branch the author names, after `git pull`.
   Every CSV row records the commit it was run with; all tables must come from one commit.
2. Never set `ILS_LICENSE_GUARD`. Never set `DESIGN_WORKERS` above 1. Never run two
   campaigns at once, nor a campaign next to a MISOCP solve: `t_best` and `Run` are reported.
3. Do not change any parameter or any code. D = 3, L_r = 20, S = 100 are the driver's
   defaults; a step below says explicitly when one of them is varied.
4. Do not edit table bodies by hand; run `fill_tables.py`. Do not delete files.
5. If a run fails or a step prints `[warn]`, keep the console output, write what
   happened under a dated heading in `experiments/DESIGN.md`, and stop. Do not retry
   with other settings.
6. After each step: the printed line `DONE -> ...csv`, then check that the CSV has one
   row per instance asked (14 unless `DESIGN_INSTANCES` was set) and that every `stop`
   is `max_idle_shakes`. Anything else is reported, not fixed.
7. The driver resumes: an interrupted campaign is restarted with the same command and
   continues from the rows already on disk.

## Step 0: check the installation (1 minute)

    DESIGN_INSTANCES="R101 (50)" DESIGN_OUT=_check python3 paper_runs/run_design.py new

Expected: `obj=11902.02 tour=10 ... stop=max_idle_shakes` within a few seconds, and
`paper_runs/results/design_new_check.csv` with the columns
`Instance, design, init, init_seed, D, L_r, S, extra, commit, init_obj, init_size, Objective,
Tour, flown_km, energy_pct, time_s, t_best (s), Wall (s), stop, ...`.

## Step 1: the reference run (Table `tab:matheuristic-vs-exact`, Figure `fig:ils-convergence`), about 30 minutes

    python3 paper_runs/run_design.py new

The existing `design_new.csv` (27 Sept, fewer columns) is moved aside automatically
as `design_new.v1.csv` and all fourteen instances run again. The search is
deterministic given the seed, so the objectives must reproduce; check with

    python3 paper_runs/compare_runs.py paper_runs/results/design_new.csv paper_runs/results/design_new.v1.csv

Expected: `change %` is 0.00 on every row (the only code change since 27 Sept is a
tie-break that fires only on exactly tied weights). A difference is reported in
`DESIGN.md`, not fixed. Run times per instance from the 27 Sept run, for planning:
R101 (50) 2 s, R101 (100) 4, R1_2_1 18, C101 (50) 33, C101 (100) 147, C1_2_1 139,
RC1_2_1 90, R102 59, R104 58, C104 304, RC104 28, PR11 17, PR15 705, PR10 197
(total 1 801 s).

Then write the table and the figure:

    python3 paper_runs/fill_tables.py --only tab:matheuristic-vs-exact
    python3 paper_runs/make_convergence_figure.py

The `f(R_4)` column changes against the earlier table (R101 (50): 4 643.08 becomes
5 863.95). Expected: the earlier column was the R4 tour evaluated at fixed speed; the
new one is the subproblem value of the tour the run actually starts from, with the
speed free. Leave it.

## Step 2: value of speed and value of loitering (Tables `tab:fixed-speed`, `tab:coverage`), about 1 hour

    DESIGN_EXTRA="--fixed-speed" DESIGN_OUT=_fixed    python3 paper_runs/run_design.py new
    DESIGN_EXTRA="--no-loiter"   DESIGN_OUT=_noloiter python3 paper_runs/run_design.py new
    python3 paper_runs/fill_tables.py --only tab:fixed-speed tab:coverage

Both variants build their own R4 start under their own subproblem (fixed speed, or
flight length pinned to the straight line), so `init_obj` and `Tour` differ from the
reference run; that is the protocol. Checks: under `--no-loiter`, `flown_km` is the
straight-line length of the route; under `--fixed-speed`, `energy_pct` is below the
reference run's on most instances.

## Step 3: the cap divisor grid (Table `tab:theta`), 2 to 3 hours

    DESIGN_CAPDIV=6  DESIGN_OUT=_D6  python3 paper_runs/run_design.py new
    DESIGN_CAPDIV=12 DESIGN_OUT=_D12 python3 paper_runs/run_design.py new
    python3 paper_runs/fill_tables.py --only tab:theta

The D = 3 block is the reference run of Step 1; nothing is rerun for it.

## Step 4: the perturbation figure (Figure `fig:ils-capdiv`), about 1 hour

    for D in 3 6 12; do
      DESIGN_INSTANCES="PR15 (240)" DESIGN_DYNAMICS=1 DESIGN_CAPDIV=$D DESIGN_OUT=_D${D}dyn \
          python3 paper_runs/run_design.py new
    done
    python3 paper_runs/make_capdiv_figure.py

Determinism check: the objective of `design_new_D3dyn.csv` equals PR15's in
`design_new.csv`, and `_D6dyn` and `_D12dyn` equal PR15's rows of `design_new_D6.csv`
and `design_new_D12.csv`. The per-iteration traces land in
`paper_runs/results/details/dynamics/` and are ignored by git; the figure is committed.

## Step 5: the initial-tour table (Table `tab:initial-tour`), about 2.5 hours

    DESIGN_INIT=R1 DESIGN_OUT=_R1 python3 paper_runs/run_design.py new
    DESIGN_INIT=R2 DESIGN_OUT=_R2 python3 paper_runs/run_design.py new
    for s in 1 2 3; do
      DESIGN_INIT=R3 DESIGN_INIT_SEED=$s DESIGN_OUT=_R3s$s python3 paper_runs/run_design.py new
    done
    python3 paper_runs/fill_tables.py --only tab:initial-tour

The R4 column is the reference run of Step 1.

## Step 8: the two levers by energy budget (Table `tab:levers-eta`), about 3 hours

Added 28 September, after Steps 1 to 7 were done. The reference run and the two
variants of Step 2 are repeated at a tighter and a looser energy budget; the driver's
`DESIGN_ETA` scales the budget and nothing else, and the value is recorded in the
`extra` column of every row.

    for e in 0.75 1.25; do t=$(echo $e | tr -d .);
      DESIGN_ETA=$e                              DESIGN_OUT=_eta$t          python3 paper_runs/run_design.py new
      DESIGN_ETA=$e DESIGN_EXTRA="--fixed-speed" DESIGN_OUT=_fixed_eta$t    python3 paper_runs/run_design.py new
      DESIGN_ETA=$e DESIGN_EXTRA="--no-loiter"   DESIGN_OUT=_noloiter_eta$t python3 paper_runs/run_design.py new
    done
    python3 paper_runs/fill_tables.py --only tab:levers-eta

Six campaigns of fourteen runs; the `eta = 1` columns of the table are already filled
from Step 1 and Step 2. Expected from the exact solver's Table `tab:loitering`: the
loitering gain rises as the budget tightens (R101 (50): 12.5 percent at 0.75 against
0.3 at 1). The speed gain may move either way: a tighter budget leaves less energy for
speed above `v_mr`, but makes the cheap waiting speed `v_mp` more valuable. A container
preview on R101 (50) at 0.75 gave a loitering gain of 4.7 percent (0.17 at 1) and a
speed gain of 15.8 percent (12.9 at 1); the reference run there reached 10 021 against
the exact optimum 10 989, so report the exact-solver gains of Table `tab:loitering`
next to these where both exist. Checks: every `stop` is `max_idle_shakes`; at `eta = 0.75`
the objectives and `Tour` are below the reference run's, at `1.25` above. Then
compile (Step 6) and commit (Step 7), adding `paper_runs/results/design_new_*eta*.csv`.

## Step 9: replication 2 of the reference run (Table `tab:matheuristic-vs-exact`), about 30 minutes

Added 29 September. Table `tab:matheuristic-vs-exact` now reports two replications of
the single-start ILS side by side. Replication 1 is `design_new.csv` (Step 1).
Replication 2 is the same start R4 with the ILS seed shifted (`--seed-offset 1`, which
the driver records in the `extra` column):

    DESIGN_EXTRA="--seed-offset 1" DESIGN_OUT=_rep2 python3 paper_runs/run_design.py new
    python3 paper_runs/fill_tables.py --only tab:matheuristic-vs-exact

Output `paper_runs/results/design_new_rep2.csv` and the traces
`details/design_traces/<stem>_new_rep2.csv`. Checks: 14 rows, every `stop` is
`max_idle_shakes`, the `init_obj` column equals that of `design_new.csv` on every
instance (same start), and the objectives differ on at least the wide-window
instances (on R101 (50) both replications reach 11 902.02). Then compile (Step 6) and
commit (Step 7), adding the new CSV and traces. Every other table stays on
replication 1, as the caption of Table `tab:matheuristic-vs-exact` states.

## Step 6: compile

    cd paper && pdflatex -interaction=nonstopmode ArXiv-version && bibtex ArXiv-version \
      && pdflatex -interaction=nonstopmode ArXiv-version && pdflatex -interaction=nonstopmode ArXiv-version

Expected: no line starting with `!` in `ArXiv-version.log`, no `??` in the PDF, the
five tables filled, both ILS figures replaced (check the file dates in `paper/fig/`).

## Step 7: commit

    git add paper_runs/results/design_new*.csv paper_runs/results/details/design_traces \
            paper/ArXiv-version.tex paper/fig/ils_capdiv.png paper/fig/meta_convergence_timematched.png \
            experiments/DESIGN.md
    git commit -m "Section 5 matheuristic tables and figures from the campaign of <date> at <commit>"
    git push

`experiments/tm_ils_*`, the dynamics traces and the LaTeX build products are ignored
by git and must stay out of the commit.

## Then the text, and only then

Only after every table above is written, and without touching Section 4. Sections 5.7,
5.9 and 5.10 currently have no prose at all (their text was removed pending this
campaign; the tex marks the places with `% TEXT REMOVED` and `% TEXT PENDING`
comments). Write one paragraph per item from the numbers in the tables, in the style of
Sections 5.3 to 5.6, and delete the comment it replaces.

1. Section 5.7, before Table `tab:initial-tour`: which start wins where, how far the
   three R3 draws spread, whether the start decides the outcome on any instance, and the
   conclusion that a single start from R4 is the protocol of the tables that follow.
2. Section 5.8, the first paragraph says "Table `tab:theta` reports the choice of D":
   add two sentences reading the table (where D = 3 wins, where it loses, shakes and run
   times). The S and L_r paragraphs already carry numbers; they stand unless Step 1
   changed an objective, which it should not.
3. Section 5.9, before Table `tab:matheuristic-vs-exact`: where the ILS reaches the
   proven optimum, where it beats the one-hour incumbent and by how much, `t_best`
   against the MISOCP time, and the two figures (`fig:ils-convergence` after the table).
   State whether the wall-clock safeguard ever ended a run before the shake limit did
   (Section 4.4 promises this; read the `stop` column of every design CSV). Once Step 9
   is done, read the two replications against each other: on how many instances they
   agree to the cent, the largest difference and where it is, and whether both beat the
   incumbent on the same instances. Replace the `% TEXT REMOVED` comment; update the
   comment block above the table (date, commit).
3b. Section 5.1, last paragraph before Section 5.2: the shares of window-feasible Swap
   and 2-opt pairs on the best routes are quoted there. Run
   `python3 paper_runs/reorder_share.py` after Step 1 and update the sentence only if
   the shares moved (they do not if Step 1 reproduced the routes).
4. Table `tab:fixed-speed`: one paragraph (loss from fixing the speed, change in the
   number of scheduled targets, where the loss is largest).
5. Section 5.10, Table `tab:coverage`: the `% TEXT PENDING` comment holds the drafted
   reading and the sentences moved out of the caption; write the paragraph from the
   numbers, then a paragraph on Table `tab:levers-eta` once Step 8 has filled it (how
   each gain moves with the budget, and why: loitering is the cheapest way to wait,
   so it pays when energy is scarce; speed above `v_mr` costs energy, so it pays when
   energy is plentiful).
5b. Every `% Moved out of the caption` comment in the tex holds material for the prose
   of its subsection; work it in and delete the comment.
6. Recompile (Step 6) and commit (Step 7).

## Optional, only if the author asks

* Multi-seed statistics: `DESIGN_EXTRA="--seed-offset k" DESIGN_OUT=_seed$k` for
  k = 1..4 gives four more runs per instance from the same R4 start; report mean,
  best and spread per instance next to Table `tab:matheuristic-vs-exact`.
* L_r grid for the 5.8 prose: `DESIGN_RCL=0|10|40` with `DESIGN_OUT=_Lall|_L10|_L40` on
  `DESIGN_INSTANCES="PR11 (48);R104 (100);RC104 (100);C104 (100);PR15 (240);PR10 (288)"`.
  The current prose quotes the 27 Sept `_Lall` numbers, which stand.
* Acceptance rate by rank (evidence for L_r): one run per instance with
  `DESIGN_EXTRA="--route-trace <f>_route.csv --move-trace <f>_moves.csv"` and
  `MOVE_TRACE_TOP=20` in the environment, then `python3 paper_runs/analyze_traces.py ranks <dir>`.
* The static-reward check against best-known OPTW values needs a slope-regime option
  in the ILS runner, which does not exist yet; ask the cloud session for it.

## Machine

Apple M3 Pro, Python 3.12, Gurobi 13 with the full license. The ILS is single-threaded.
The whole sequence above is about eight hours of machine time; Steps 1 to 5 can run
back to back in one shell script as long as nothing else runs on the machine.
