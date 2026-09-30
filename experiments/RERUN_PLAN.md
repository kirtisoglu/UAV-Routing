# Runbook: every ILS table and figure of the paper, design of 30 September 2026

For the local agent. Read this file to the end before running anything; it replaces
every earlier plan. On 30 September the design of Section 4 changed in two places:

* every move of the four operators is weighted by the four tiers of its information
  change and length change (Section 4.3, paragraphs "Weight of a move" and "Candidate
  selection"), and
* the shake applies the next removal of the enumeration, with no look-ahead (Section 4.4).

This is the `tiers` test with `knap=0` of `experiments/TEST_RUNBOOK.md`, and it is now
the default of `paper_runs/run_design.py`. Every ILS table and both ILS figures of
Section 5 are regenerated under it; no number of the previous design stays in the paper.
Table `tab:matheuristic-vs-exact` (Table 10) reports two replications.

The ILS cells of the six tables are empty in the tex (Table 10 keeps its Greedy and
MISOCP columns), and every number of the prose that came from the previous runs is a
`\tbd{}` marker, printed as **[TBD]**: Section 4.3 (one), Section 5.1 (six) and the
Parameter analysis (twelve). The steps below fill the tables; the prose items at the end
replace the markers.

## What is where

| Thing | Location |
|---|---|
| The paper (single source) | `paper/ArXiv-version.tex`, figures in `paper/fig/`, bibliography `paper/ref.bib` |
| The matheuristic | `experiments/run_ils_time_matched.py` with `experiments/fast_sets.py`, `run_ils_final.py`, `run_ils_final_scored.py`, `run_ils_fb_cascade_demo.py` and the `uav_routing` package |
| The driver (every ILS number) | `paper_runs/run_design.py`; outputs `paper_runs/results/design_new<SUFFIX>.csv` and traces under `paper_runs/results/details/design_traces/` |
| The table writer | `paper_runs/fill_tables.py` (writes the table bodies into the tex from the CSVs; a table whose inputs are missing is skipped, not emptied) |
| The figures | `paper_runs/make_convergence_figure.py`, `paper_runs/make_capdiv_figure.py` (write into `paper/fig/`) |
| Comparing two runs | `paper_runs/compare_runs.py a.csv b.csv` (last column `same` = same best route, iterations and shakes) |
| Shake gaps and the stopping rule | `paper_runs/analyze_traces.py shakes|stop <dir> '<pattern>'` |
| What each result file is | `paper_runs/results/README.md` |
| The previous design's results | `paper_runs/results/previous_design/` (records; no table reads them) |
| MISOCP tables (Sections 5.4 to 5.7) | done; `misocp_s1.csv`, `misocp_seeds.csv`, `slope_regimes.csv`, `eta_sweep.csv`, `loitering.csv`. Do not rerun. |

## Rules

1. Work in the repository root, on the branch `claude/exciting-brahmagupta-ocwy98`, after
   `git pull`. Every CSV row records the commit it was run with; all tables must come
   from one commit.
2. Never set `ILS_LICENSE_GUARD`. Never set `DESIGN_WORKERS` above 1. Never run two
   campaigns at once, nor a campaign next to a MISOCP solve: `t_best` and `Run` are reported.
3. Do not change any parameter or any code, and do not set `DESIGN_REORDER_W` or
   `DESIGN_SHAKE_KNAP`: their defaults are the design. D = 3, L_r = 20, S = 100 are the
   driver's defaults; a step below says explicitly when one of them is varied.
4. Do not edit table bodies by hand; run `fill_tables.py`. Do not delete files.
5. If a run fails or a step prints `[warn]`, keep the console output, write what
   happened under a dated heading in `experiments/DESIGN.md`, and stop. Do not retry
   with other settings.
6. After each step: the printed line `DONE -> ...csv`, then check that the CSV has one
   row per instance asked (14 unless `DESIGN_INSTANCES` was set), that every `stop` is
   `max_idle_shakes`, and that `extra` holds only what the step sets (never `reorder=`
   or `knap=`). Anything else is reported, not fixed.
7. The driver resumes: an interrupted campaign is restarted with the same command and
   continues from the rows already on disk.

## Step 0: set the previous results aside, check the installation (2 minutes)

Start only when no campaign is running: the `tiers` test runs of `TEST_RUNBOOK.md` must
have finished (the `knap=0` one is the comparison of Step 1), since the commands below
move their CSVs.

The driver resumes from any CSV of the same name, so a `design_new*.csv` of the
previous design left in `paper_runs/results/` would make the step that writes that file
skip every instance and keep the old numbers. The files tracked by git were moved to
`paper_runs/results/previous_design/` in the commit that brought this runbook; move the
ones that exist only on this machine too (for instance `design_new_rep2.csv`, or the
test runs `design_new_tiers*.csv`, `design_new_route*.csv`):

    git pull
    mkdir -p paper_runs/results/previous_design
    find paper_runs/results -maxdepth 1 -name 'design_new*.csv' -exec mv {} paper_runs/results/previous_design/ \;
    find paper_runs/results -maxdepth 1 -name 'design_new*.csv'      # must print nothing
    DESIGN_INSTANCES="R101 (50)" DESIGN_OUT=_check python3 paper_runs/run_design.py new

Expected: the `[plan]` line reads `reorder=tiers knap=0`, then
`obj=11921.13 tour=10 ... stop=max_idle_shakes` within seconds (the cloud container gave
11 921.13 with 228 shakes), and the `extra` column of `design_new_check.csv` is empty.

## Step 1: replication 1, the reference run (Table `tab:matheuristic-vs-exact`, Figure `fig:ils-convergence`)

    python3 paper_runs/run_design.py new
    python3 paper_runs/fill_tables.py --only tab:matheuristic-vs-exact
    python3 paper_runs/make_convergence_figure.py

If `paper_runs/results/previous_design/design_new_tiers_knap0.csv` exists (the test run
of this design), compare:

    python3 paper_runs/compare_runs.py paper_runs/results/design_new.csv paper_runs/results/previous_design/design_new_tiers_knap0.csv

Expected: `+0.00` and `same` on every row, since the test run is this design at the same
seed and the search code has not changed since commit 20889f3; the run should take about
as long as that test run did. A `differs` row is reported in `DESIGN.md`, not fixed.

## Step 2: replication 2 (Table `tab:matheuristic-vs-exact`)

The same start R4 with the ILS seed shifted (`--seed-offset 1`, recorded in `extra`):

    DESIGN_EXTRA="--seed-offset 1" DESIGN_OUT=_rep2 python3 paper_runs/run_design.py new
    python3 paper_runs/fill_tables.py --only tab:matheuristic-vs-exact

Output `paper_runs/results/design_new_rep2.csv` and the traces
`details/design_traces/<stem>_new_rep2.csv`. Checks: 14 rows, every `stop` is
`max_idle_shakes`, `extra` reads `--seed-offset 1`, the `init_obj` column equals that of
`design_new.csv` on every instance (same start), and the objectives differ on at least
the wide-window instances. Every other table stays on replication 1, as the caption of
Table `tab:matheuristic-vs-exact` states.

## Step 3: value of speed and value of loitering (Tables `tab:fixed-speed`, `tab:coverage`)

    DESIGN_EXTRA="--fixed-speed" DESIGN_OUT=_fixed    python3 paper_runs/run_design.py new
    DESIGN_EXTRA="--no-loiter"   DESIGN_OUT=_noloiter python3 paper_runs/run_design.py new
    python3 paper_runs/fill_tables.py --only tab:fixed-speed tab:coverage

Both variants build their own R4 start under their own subproblem (fixed speed, or
flight length pinned to the straight line), so `init_obj` and `Tour` differ from the
reference run; that is the protocol. Checks: under `--no-loiter`, `flown_km` is the
straight-line length of the route; under `--fixed-speed`, `energy_pct` is below the
reference run's on most instances.

## Step 4: the cap divisor grid (Table `tab:theta`)

    DESIGN_CAPDIV=6  DESIGN_OUT=_D6  python3 paper_runs/run_design.py new
    DESIGN_CAPDIV=12 DESIGN_OUT=_D12 python3 paper_runs/run_design.py new
    python3 paper_runs/fill_tables.py --only tab:theta

The D = 3 block is replication 1 (Step 1); nothing is rerun for it.

## Step 5: the perturbation figure (Figure `fig:ils-capdiv`)

    for D in 3 6 12; do
      DESIGN_INSTANCES="PR15 (240)" DESIGN_DYNAMICS=1 DESIGN_CAPDIV=$D DESIGN_OUT=_D${D}dyn \
          python3 paper_runs/run_design.py new
    done
    python3 paper_runs/make_capdiv_figure.py

Determinism check: the objective of `design_new_D3dyn.csv` equals PR15's in
`design_new.csv`, and `_D6dyn` and `_D12dyn` equal PR15's rows of `design_new_D6.csv`
and `design_new_D12.csv`. The per-iteration traces land in
`paper_runs/results/details/dynamics/` (ignored by git; the previous design's files of
the same names are overwritten); the figure is committed.

## Step 6: the initial-tour table (Table `tab:initial-tour`)

    DESIGN_INIT=R1 DESIGN_OUT=_R1 python3 paper_runs/run_design.py new
    DESIGN_INIT=R2 DESIGN_OUT=_R2 python3 paper_runs/run_design.py new
    for s in 1 2 3; do
      DESIGN_INIT=R3 DESIGN_INIT_SEED=$s DESIGN_OUT=_R3s$s python3 paper_runs/run_design.py new
    done
    python3 paper_runs/fill_tables.py --only tab:initial-tour

The R4 column is replication 1 (Step 1).

## Step 7: the two levers by energy budget (Table `tab:levers-eta`)

The reference run and the two variants of Step 3 at a tighter and a looser energy
budget; `DESIGN_ETA` scales the budget and nothing else, and is recorded in `extra`.

    for e in 0.75 1.25; do t=$(echo $e | tr -d .);
      DESIGN_ETA=$e                              DESIGN_OUT=_eta$t          python3 paper_runs/run_design.py new
      DESIGN_ETA=$e DESIGN_EXTRA="--fixed-speed" DESIGN_OUT=_fixed_eta$t    python3 paper_runs/run_design.py new
      DESIGN_ETA=$e DESIGN_EXTRA="--no-loiter"   DESIGN_OUT=_noloiter_eta$t python3 paper_runs/run_design.py new
    done
    python3 paper_runs/fill_tables.py --only tab:levers-eta

Six campaigns of fourteen runs; the `eta = 1` columns come from Steps 1 and 3. Checks:
every `stop` is `max_idle_shakes`; at `eta = 0.75` the objectives and `Tour` are below the
reference run's, at `1.25` above.

## Step 8: the numbers of Section 5.9 (Parameter analysis) on L_r, S and the solve time

The second paragraph of Section 5.9 quotes the restricted list against the unrestricted
reordering sets and the shake gaps behind S = 100, and the third the time of a solve in
the scaled units against physical units; all are measured again:

    G="PR11 (48);R104 (100);RC104 (100);C104 (100);PR15 (240);PR10 (288)"
    DESIGN_INSTANCES="$G" DESIGN_RCL=0 DESIGN_OUT=_Lall python3 paper_runs/run_design.py new
    python3 paper_runs/compare_runs.py paper_runs/results/design_new.csv paper_runs/results/design_new_Lall.csv
    python3 paper_runs/analyze_traces.py shakes paper_runs/results/details/design_traces '*_new.csv'
    python3 paper_runs/analyze_traces.py stop   paper_runs/results/details/design_traces '*_new.csv'
    DESIGN_SCALED=0 DESIGN_OUT=_physical python3 paper_runs/run_design.py new

The last command repeats replication 1 with the subproblem in physical units
(`physical-units` in `extra`); only its `socp_ms` column is used, since its search
diverges from the scaled run's at the first differing verdict. Solves are about five
times slower there (R101 (50): 12.4 against 2.3 ms in the container), so allow several
hours.

The first run is the six instances where reorderings matter with no list at all
(`L_r=0` in the CSV); the unrestricted runs on C104, PR15 and PR10 are long, and the
wall-clock safeguard of four hours per run bounds them (report it if it binds). The
comparison gives the objective change of L_r = 20 against no list, and the shakes and
run times of both. `shakes` gives, for each run of replication 1, the longest gap
between two improvements of the best (`max gap`); `stop` gives the objective and the
stopping shake the run would have had with S = 25, 50, 100, .... Write the outputs and
the `socp_ms` range (smallest and largest over the fourteen rows) of `design_new.csv` and
of `design_new_physical.csv` into `DESIGN.md`; prose item 3 below turns them into the text.

## Step 9: compile

    cd paper && pdflatex -interaction=nonstopmode ArXiv-version && bibtex ArXiv-version \
      && pdflatex -interaction=nonstopmode ArXiv-version && pdflatex -interaction=nonstopmode ArXiv-version

Expected: no line starting with `!` in `ArXiv-version.log`, no `??` in the PDF (the
**[TBD]** markers stay until the prose items are done), the seven tables filled (Tables `tab:matheuristic-vs-exact` with both replications,
`tab:fixed-speed`, `tab:coverage`, `tab:theta`, `tab:initial-tour`, `tab:levers-eta`),
both ILS figures replaced (check the file dates in `paper/fig/`).

## Step 10: commit

    git add paper_runs/results/design_new*.csv paper_runs/results/previous_design \
            paper_runs/results/details/design_traces \
            paper/ArXiv-version.tex paper/fig/ils_capdiv.png paper/fig/meta_convergence_timematched.png \
            experiments/DESIGN.md
    git commit -m "Section 5 ILS tables and figures under the tiers design, campaign of <date> at <commit>"
    git push

`experiments/tm_ils_*`, the dynamics traces and the LaTeX build products are ignored
by git and must stay out of the commit.

## Then the text, and only then

Only after every table above is written. Write one paragraph per item from the numbers
in the tables, in the style of Sections 5.4 to 5.7, and delete the `% TEXT PENDING` or
`% TEXT REMOVED` comment it replaces. Every `\tbd{}` gets its number; where a sentence
around a marker no longer fits the numbers, rewrite the sentence rather than force the
number into it. Subsections are named by title, since the numbers moved.

1. Section 4.3, the paragraph after Algorithm 1: "at most \tbd{} routes per run" is the
   largest `socp_numeric_infeasible` of `design_new.csv` and `design_new_rep2.csv`; then
   delete the `% TEXT PENDING` comment. Nothing else in Section 4 is touched.
2. "Initial tour selection for the matheuristic", before Table `tab:initial-tour`: which
   start wins where, how far the three R3 draws spread, whether the start decides the
   outcome on any instance, and the conclusion that a single start from R4 is the
   protocol of the tables that follow.
3. "Parameter analysis": in the first paragraph, after "Table `tab:theta` reports the
   choice of D", two sentences reading the table (where D = 3 wins, where it loses,
   shakes and run times). The second paragraph has nine markers, filled from Step 8: the
   longest gap and where, the most elsewhere, the objective change at S = 50 and at
   S = 25, the objective change of L_r = 20 against no list over the six instances
   (smallest and largest) and what the list does to the shakes a run affords; say
   whether S = 100 cuts any improvement and whether the list gains or loses, and delete
   the comment. The third paragraph (solver settings) has four markers: the `socp_ms`
   range of `design_new.csv` (scaled units) and of `design_new_physical.csv` (physical
   units), Step 8.
4. "Matheuristic vs. exact solver", before Table `tab:matheuristic-vs-exact`: where the
   ILS reaches the proven optimum, where it beats the one-hour incumbent and by how much,
   `t_best` against the MISOCP time, and Figure `fig:ils-convergence` after the table.
   State whether the wall-clock safeguard ever ended a run before the shake limit did
   (Section 4.4 promises this; read the `stop` column of every design CSV). Read the two
   replications against each other: on how many instances they agree to the cent, the
   largest difference and where it is, and whether both beat the incumbent on the same
   instances. Replace the `% TEXT REMOVED` comment, and update the comment block above
   the table (date, commit, design: tiers weights, no look-ahead).
5. Section 5.1, last paragraph before Section 5.2: six markers, the shares of
   window-feasible Swap and 2-opt pairs on the best routes (smallest and largest on the
   co-monotone and on the inverted block), the largest number of scheduled targets on the
   Cordeau block and the share of its pairs left feasible. Run
   `python3 paper_runs/reorder_share.py` after Step 1.
6. Table `tab:fixed-speed`: one paragraph (loss from fixing the speed, change in the
   number of scheduled targets, where the loss is largest).
7. "Value of loitering for the matheuristic", Table `tab:coverage`: the `% TEXT PENDING`
   comment holds the drafted reading and the sentences moved out of the caption; write
   the paragraph from the numbers, then a paragraph on Table `tab:levers-eta` (how each
   gain moves with the budget, and why: loitering is the cheapest way to wait, so it
   pays when energy is scarce; speed above `v_mr` costs energy, so it pays when energy
   is plentiful).
8. Every `% Moved out of the caption` comment in the tex holds material for the prose
   of its subsection; work it in and delete the comment.
9. When no `\tbd` is left in the tex, delete the line `\newcommand{\tbd}...` in the
   preamble; then recompile (Step 9) and commit (Step 10).

## Optional, only if the author asks

* Multi-seed statistics: `DESIGN_EXTRA="--seed-offset k" DESIGN_OUT=_seed$k` for
  k = 2..4 gives more runs per instance from the same R4 start; report mean, best and
  spread per instance next to Table `tab:matheuristic-vs-exact`.
* The static-reward check against best-known OPTW values needs a slope-regime option
  in the ILS runner, which does not exist yet; ask the cloud session for it.

## Machine

Apple M3 Pro, Python 3.12, Gurobi 13 with the full license. The ILS is single-threaded.
The previous design's campaign took about eight hours of machine time for Steps 1 to 7;
Step 8 adds the unrestricted runs. Steps 1 to 8 can run back to back in one shell script
as long as nothing else runs on the machine.
