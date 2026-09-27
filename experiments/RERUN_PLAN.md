# Note for the local agent (written 2026-09-27 by the cloud session; read this first)

Context. The cloud session rewrote Section 4 of the paper for a revised matheuristic
design, implemented the design as opt-in flags of `experiments/run_ils_time_matched.py`,
and merged everything to `main` (PR #1, commit 9b0a4cb; this note and small driver fixes
came after). The cloud machine had only the pip Gurobi license (routes of at most 32
targets), so the design was validated on the mechanics and on four short-route instances
only. **Every number that Section 5 needs has to be produced on this machine.** The
evidence behind the design is in `experiments/DESIGN.md` (section "Change round of
27 September 2026"); the paper files are `paper/Section4-matheuristic.tex` (the section),
`paper/ArXiv-version.tex` (full manuscript, compiles) and `paper/ref.bib` (adds
`savelsbergh1992vehicle`). Section 5 has not been touched and is now inconsistent with
Section 4 in the places listed under "Then edit Section 5" below.

The design, in one line: same operators, roulette, acceptance and reference-route shake
as before, plus `--fast-sets` (O(1) move tests, same sets and weights), `--scaled-socp`
(subproblem in nondimensional units, Presolve 0, homogeneous barrier), `--reorder-rcl 20`
(only the 20 best Swap/2-opt moves enter the set), `--sweep enum` (explicit interleaved
enumeration of a reference's removals) and `--max-idle-shakes 100` (stop after 100
consecutive shakes without an improvement); no stall, no maxIter. Parameters of the
paper: D = 3 (cap c = ceil(k/D)), L_r = 20, S = 100, look-ahead m = 6 fixed.

## Rules

* Never set `ILS_LICENSE_GUARD` here (it was a cloud-only workaround).
* Protocol of `paper_runs/README.md` stays: single start from R4, seed 42, eta = 1,
  slope seed 1, one ILS run at a time and never concurrently with a MISOCP solve,
  so that t_best and Run are comparable with the MISOCP column.
* `paper_runs/run_design.py` is the only driver for matheuristic numbers. It appends
  each finished run to its CSV and resumes, so an interrupted campaign is simply
  restarted. Its knobs: `DESIGN_INSTANCES`, `DESIGN_RCL`, `DESIGN_IDLE`, `DESIGN_CAPDIV`,
  `DESIGN_OUT` (suffix so a grid does not overwrite the main campaign), `DESIGN_EXTRA`
  (flags appended to every run), `DESIGN_BUDGET`, `DESIGN_WORKERS` (keep 1).
* Do not change the design while producing the tables. If something looks wrong,
  record it in `experiments/DESIGN.md` and stop.

## Step 0: sanity checks (15 minutes)

1. `python3 experiments/validate_fast_sets.py` must print "sets identical on 120/120"
   for every instance (it needs no solver; the long-route instances are the point).
2. Regression of the O(1) construction inside the loop: with the base environment
   `ILS_INSERT_RATIO=1 ILS_SHAKE_KNAP=6 ILS_NO_RETURN=2 ILS_REORDER_W=exch
   ILS_SHAKE_STALL=1000` run
   `python3 experiments/run_ils_time_matched.py --instance "R101 (100)" --max-iter 13000
   --budget 600 --init R4 --shake-return --shake-backtrack --fast-sets --tag reg`
   and compare with `paper_runs/results/newdesign/final_r101_100.log`: it must report
   best 22 884.32 found at iteration 351 after 13 351 iterations (identical trajectory).
3. `python3 experiments/test_scaled_socp2.py`: the "P0+BH" row must agree with the
   physical model on 40/40 routes for R102, RC104 and R101. Then add PR15 (240) and
   C104 (100) to the dictionary at the bottom of the script and run it again: these are
   the long routes the cloud could not test. Keep the printed iteration counts and
   milliseconds for Section 5.8.
4. Smoke test of the full configuration: `python3 paper_runs/run_design.py new` with
   `DESIGN_INSTANCES="R101 (50)"`; expect 11 902.02 or the optimum 11 921.13 within
   seconds, and `stop = idle_shakes` in `paper_runs/results/design_new.csv`. Delete that
   CSV afterwards so the main campaign starts clean, or keep it (the driver resumes).

## Step 1: the main campaign (feeds Table 5.9, Figure 5, the S calibration)

    python3 paper_runs/run_design.py new

14 runs, one at a time. Output `paper_runs/results/design_new.csv` (Objective, Tour,
t_best, Wall, stop, iterations, shakes, accepted, socp_calls, socp_ms, Route) and the
best-seen traces `paper_runs/results/details/design_traces/<stem>_new.csv` with a
`shakes` column. Expected order of magnitude: the old design took 62 minutes for all
fourteen on this machine; the new one should be shorter on the short-route instances
and longer only where 100 idle shakes take long (PR15, PR10, C104). If a run hits the
14 400 s safeguard, note it in the table (Section 4.4 promises to report how often it
binds).

Then, without any run:

    python3 paper_runs/analyze_traces.py stop paper_runs/results/details/design_traces

prints, per instance, the objective and stopping shake for S in {25, 50, 100, 150,
200, 300}: this is the S calibration of Section 5.8 (the runs used S = 100, so values
above 100 are not observable and read as the S = 100 result).

## Step 2: same-machine baseline (feeds the design comparison in Section 5.8)

    python3 paper_runs/run_design.py old

Reruns the 2026-09-26 base (physical-unit SOCP, full reorder sets, arithmetic sweep,
maxIter = 13 000) on the same machine and day, so the comparison with `design_new.csv`
is clean. About one hour. Report per instance: Objective, t_best, Run, shakes, socp_ms
for both designs; the cloud numbers in DESIGN.md are for reference only.

## Step 3: parameter grids (Section 5.8), on the five instances where they can matter

    G="R104 (100);RC104 (100);C104 (100);PR15 (240);PR10 (288)"
    DESIGN_INSTANCES="$G" DESIGN_RCL=10 DESIGN_OUT=_L10 python3 paper_runs/run_design.py new
    DESIGN_INSTANCES="$G" DESIGN_RCL=40 DESIGN_OUT=_L40 python3 paper_runs/run_design.py new
    DESIGN_INSTANCES="$G" DESIGN_RCL=0  DESIGN_OUT=_Lall python3 paper_runs/run_design.py new
    DESIGN_INSTANCES="$G" DESIGN_CAPDIV=6 DESIGN_OUT=_D6 python3 paper_runs/run_design.py new

L_r = 20 and D = 3 are the main campaign. The `_Lall` grid (no restriction) is the
ablation that justifies L_r; expect it to be slower with a similar objective on the
first four and much slower on C104. The co-monotone instances are not needed here:
their reorder sets hold a handful of moves and the list is not binding.

## Step 4: acceptance rate by rank (the evidence for L_r, Section 5.8)

    python3 paper_runs/analyze_traces.py ranks animation/traces

uses the move traces of the 2026-09-26 base already on disk (the statistic is a
property of the operators and weights, which the new design keeps). Report one row per
operator class for two or three instances (e.g. R104, PR15, C104): accepted/drawn per
rank bucket. To have the same evidence from the new design, run one instance with
`MOVE_TRACE_TOP=20` and the flags `--route-trace <f>_route.csv --move-trace <f>_moves.csv`
added to the new configuration, then point the script at that directory.

## Step 5: value of speed and of loitering (Tables 10 and 5.10)

    DESIGN_EXTRA="--fixed-speed" DESIGN_OUT=_fixed   python3 paper_runs/run_design.py new
    DESIGN_EXTRA="--no-loiter"   DESIGN_OUT=_noloiter python3 paper_runs/run_design.py new

Both use the new design. The scaled SOCP takes the speed envelope from the drone at
solve time, so `--fixed-speed` pins v_min = v_max = v_mr correctly (fixed 2026-09-27;
check one instance: the Tour column must drop against `design_new.csv`).

## Step 6: initial-tour table (Table 8), only if it is to be refreshed

The driver has no start option; run `experiments/run_ils_time_matched.py` directly
with the new flags, `--init R1|R2|R3 --init-seed s`, the env of Step 0.2 with
`ILS_SHAKE_STALL=0`, and collect the "best obj ... found at" lines as
`paper_runs/run_initial_tours.py` did.

## Then edit Section 5 (text only; Section 4 is final)

1. 5.8 parameter analysis: remove the sentences on `stall = 1000` and `maxIter = 13 000`;
   the parameters are D, L_r and S. Add: the S calibration (Step 1), the L_r grid and the
   no-restriction ablation (Step 3), the acceptance-by-rank table (Step 4), the SOCP
   iterations and time physical vs scaled (Step 0.3), and the same-machine comparison
   old vs new (Step 2: objective, t_best, run time, shakes per instance). Section 4
   refers to 5.8 for all of these by `\ref{subsec:parameter-analysis}`.
2. Table 9 (cap divisor) was produced with the old design; either replace it by the
   D grid of Step 3 or state in its caption which design it belongs to.
3. Table 5.9 (matheuristic vs exact): ILS columns from `design_new.csv`; caption: the
   run ends after S = 100 shakes without an improvement, t_best and Run as before.
4. Figure 5: regenerate from `design_traces/*_new.csv` (`paper_runs/make_convergence_figure.py`
   reads the old trace location; point it at the new files).
5. Table 10 and the loitering table 5.10 from Step 5; the pending caption text of 5.10 is
   already drafted in the tex as a comment.
6. Section 5.1 (datasets) still says "Section 4.3 quantifies how often they can be
   applied", which is unchanged and fine.
7. Recompile `paper/ArXiv-version.tex` (pdflatex, bibtex, pdflatex x2). The three figure
   files under `fig/` are not in the repository; copy them from the paper folder.

## Known caveats to keep in mind (not to fix now)

* The energy tie-break coefficient 1e-5 is below the barrier tolerance on objectives
  of order 2e4, so the returned energy among information-equivalent schedules is
  solver noise on degenerate routes; the capacity tie-break of the reorder acceptance
  reads that noise. Same in the old design.
* In scaled mode the energy post-check accepts an excess of up to 1e-5 of the budget
  (measured at most 1.7e-6, cone slack on very short legs).
* Single runs are noisy on the wide-window instances (4 to 20 percent spread across
  configurations in the September logs). If a design comparison is close, that is
  within noise; do not over-read it.
* The enumeration order matters: a level-by-level order lost 3.4 percent on R1_2_1 in
  the cloud tests before the levels were interleaved. The committed order is the
  interleaved one.

---

# Rerun campaign plan (September 2026)

Everything after Section 5.3 of the paper is rewritten from the results of
this campaign, because two things changed since the current tables were made:
the benchmark grew from 6 to 14 instances (Table 2 of the paper), and the
matheuristic changed (four operators drawn uniformly, sweep shake,
five pre-SOCP checks). Old numbers stay visible in the paper tables until
each is replaced. The prose that was removed is kept in
`Brain/40-Papers/uav-routing/Section5-removed-text.tex` for reference.

> **2026-09-27.** Section 4 was rewritten for the design of `paper_runs/run_design.py new`
> (`--fast-sets --scaled-socp --reorder-rcl 20 --sweep enum --max-idle-shakes 100`).
> Every matheuristic row below is to be produced with that driver; the parameters of
> Section 5.8 are D, L_r and S (no stall, no maxIter). Add to 5.8: acceptance rate by
> weight rank per operator (from `--move-trace`), SOCP iterations/time physical vs
> scaled, and the S calibration read from the traces. See experiments/DESIGN.md.

## 0. Ground rules (apply to every experiment)

| Item | Setting | Notes |
|---|---|---|
| Instances | the 14 of Table 2 | not every experiment uses all 14; see per-experiment lists |
| Slope realization | seed 1 | seeds 2-4 only where the plan says so |
| eta | 1.0 | except the eta sweep (5.10, kept as is) |
| Exact method | MISOCP with tightened per-leg big-M (Desrochers-Laporte), no cuts | `paper_runs/run_misocp_s1.py`, Threads=0, 3600 s, Gurobi seed 42. Cuts dropped 2026-09-04 together with Section 5.9; earlier cut-campaign results discarded |
| Matheuristic | **single start**, one initial tour, 3600 s wall-clock | matches Section 4 as written; restarts OFF, no best-of-K, no elite ruin |
| ILS config | 4 operators drawn uniformly (adaptive layer dropped 2026-09-04: weights sat at the floor on every regime, objective neutral; Turkes et al. 2021 meta-analysis +0.14%), sweep shake, shake after theta = 1000 proposals without an accepted move (counter fixed 2026-09-09; every non-accepted proposal counts), cap ceil(k/6) on the tour length | E2 grid 2026-09-12 (14 instances x theta in {100, 300, 1000, 3000} x 3 seeds, 600 s): theta = 1000 has the smallest mean deviation from the per-instance best (0.61%) and a max deviation of 3.14%; the per-target rule 20 n lost on both large instances in the 09-11 grids |
| Initial tour | R4 greedy unless the experiment is about the initial tour | |
| Machine | M3 Pro 11 cores; ILS is single-threaded, MISOCP uses all cores | never run an ILS timing and a MISOCP concurrently |
| Bookkeeping | one CSV per experiment under `experiments/`, appended per run (resume-safe) | Gurobi logs and dynamics traces to scratchpad |

Runner changes needed before E1: a `--single-start` flag in
`run_ils_time_matched.py` (restart_stall = inf, no elite restarts) and a
`--cap` shortcut; `run_bestof_ils.py` is no longer part of the protocol.

## 1. Experiments, in execution order

### E0. Escalation cap decision (before everything else)
- Run: single-start ILS, 3600 s, cap in {4, ceil(n/6), ceil(n/3)} on C101 (100),
  C1_2_1 (200), RC104 (100), PR15 (240). 12 runs, 12 h.
- Decide: one cap for the whole paper (state it in 4.6, replace the `% TODO`).
- Analyze: objective and time-to-best per cap; whether the answer flips
  between co-monotone and inverted instances.

### E1. Matheuristic vs exact (Table 8, the core table)
- Run: single-start ILS 3600 s on all 14; MISOCP 3600 s on all 14
  (PR15 done in `paper_runs/results/misocp_s1.csv`; 13 to run, ~13 h).
- Record: greedy value f(R4), ILS best, time-to-best, MISOCP obj/bound/gap, Delta.
- Existing one-hour ILS results (PR15 15,000.21; PR10 14,057.75) were multi-start,
  so they are superseded; keep as sanity bounds only.
- Analyze: where ILS beats/ties/loses the exact incumbent; crossing times;
  per-operator acceptance rates per instance vs Table 2's forced-pair share
  (the falsifiable claim of Section 5.1: reordering moves are accepted at a
  fraction of the insertion rate where forced pairs dominate, at the same
  rate where they are rare). Record with `--weights-out`.
- Also feeds: E7 figures, E13 arrival distributions, the greedy
  baseline column (fixes the old 3,426.79 vs 2,966.62 mismatch on C1_2_1).
- Cost: 14 h ILS + 13 h MISOCP.

### E2. Exact-solver scalability (Table 5)
- Formulation of record is the MISOCP with tightened big-M (the old
  numbers stay in Table 5 until replaced; caption must say which model).
- Run: s = 1 all 14 (shared with E1); s = 2,3,4 on a subset first: R101 (100),
  C101 (100), R104 (100), RC104 (100), PR11 (48). Full 4-seed grid on all 14
  only if the subset shows seed sensitivity worth reporting.
- Analyze: closure pattern vs window regime (not vs geometry); seed spread.
- Cost: 0 h (s=1 from E1) + 15 h (subset seeds 2-4); +27 h if full grid.

### E3. Slope regime (Table 6, Fig. 4)
- Run: MISOCP under mixed / growth / decay / static on
  R104 (100), C104 (100), RC104 (100), PR15 (240). 16 h.
- Keep old rows for the 6 original instances as they are (old model, note
  in caption) or rerun them if E2 is rerun anyway.
- Analyze: does the regime effect (growth pre-commits late arrival, decay
  early) survive when windows do not order the targets; static regime as
  OPTW reference.

### E4. Initial tour selection (Table 7)
- Purpose gained weight: the method is single-start, so this table is the
  evidence that the starting tour does not matter.
- Run: single-start ILS from R1, R2, R3 (3 draws), R4 on all 14, 600 s each
  (sensitivity to start, not final quality). 14 x 6 x 600 s = 14 h.
- Analyze: spread across starts vs spread across R3 draws; any instance
  where the start decides the basin.

### E5. Robustness variants (Table 9)
- Run on C1_2_1 (200), RC104 (100), PR15 (240), single-start 3600 s each:
  fixed operator set (no swap/2-opt), segment kick instead of sweep,
  uniform candidate sampling, tilt acceptance.
  4 variants x 3 instances = 12 h.
- Analyze: each variant vs the final configuration; the sweep row is the
  one that justifies Section 4.5.

### E6. Value of continuous speed, heuristic side (Table 10)
- Run: single-start ILS 3600 s with v_min = v_max = v_mr on all 14. 14 h.
- Analyze: loss vs variable speed (Table 8 column); tour-size drop; does the
  loss grow with instance size and with window overlap.

### E7. Figures from E1 traces (Figs. 5 and 6)
- Fig. 5 (convergence): best-seen vs wall-clock, all 14, from E1 traces.
- Fig. 6 (perturbation dynamics): regenerate with `run_fig_pr15_dynamics.py`
  style for two contrasting instances, e.g. C101 (100) and PR15 (240), from
  E1 `--dynamics-out` traces; replaces the old k/tau design-study figure,
  whose parameters no longer exist.
- No extra runs; record `--dynamics-out` in E1.

### E8. (removed) Warm start
- Dropped 2026-09-04: Section 5.8 was removed from both papers, so the
  warm-start table has no home. Frees ~11 machine hours.

### E9. (removed) Cuts subsection
- Dropped 2026-09-04: the paper no longer uses valid inequalities; Section 5.9
  was removed and the tightened big-M is part of the formulation (Section 3.3).

### E10. Energy sensitivity (5.10) - untouched for now
- Kept as is for the co-author. Later: fold into E2/E6 or move.

### E11. Loitering (Table 15, exact vs exact)
- Exact-vs-exact needs closure; only the R-class closes, so the old table
  stands. New: heuristic-side loitering value on all 14 (fixed-tour SOCP with
  L = d on the E1 tours vs free L), no extra ILS runs. Decide whether it
  becomes a column of Table 10 or its own small table.

### E12. (removed) Fixed-speed exact
- Dropped 2026-09-05: the exact-side speed table duplicated the
  matheuristic-side one and was removed from both papers. Frees ~3-6 h.

### E13. Arrival-time distributions (Fig. 14)
- From E1 tours (matheuristic side) and E2 incumbents; regenerate grouped by
  window regime rather than R/C class.

## 2. Budget and ordering

| Priority | Experiments | Machine hours |
|---|---|---|
| P1 | E0, E1 | ~37 |
| P2 | E2 (seed subset), E5 | ~27 |
| P3 | E3, E4, E6, E12 | ~48 |
| P4 | E2 full seed grid, E11 heuristic loitering | ~30 |

Run P1 first; E1 unlocks E7, E13 and the greedy-baseline fix at no cost.

## 3. Open decisions (need the author)
1. Escalation cap c (E0 decides; then fix in Section 4.6).
2. Table 5 model of record: tightened big-M everywhere, or keep the old
   numbers for the six original instances with a caption note.
3. Seeds: 4 on a subset (plan) or on all 14.
4. Whether Table 7 (initial tours) runs at 600 s or 3600 s.
5. Fate of 5.10 after the co-author review.

## 4. Analysis notes to carry into the rewrite
- Difficulty tracks window structure and geometry, not size (RC104 (100) 81%
  vs C1_2_1 (200) 34%; R101 closes, R102/R104 do not).
- Tightened big-M moved the PR15 incumbent +78% and the bound 0.02% (primal
  side only); the cuts that moved the bound were dropped from the paper.
- Energy binds on every Solomon-derived instance at eta = 1, not on Cordeau.
- Operator acceptance rates (300 s probe): C101 insert 7.9%, swap 2.7%;
  R102, RC104, PR15 all four within a factor of 1.5 of each other.
- Exact solver's incumbent failure mode on large overlapping instances is
  branch-and-bound throughput (477 nodes/h on PR15) plus weak primal
  heuristics, not model error (ILS tours are accepted by the MISOCP).
