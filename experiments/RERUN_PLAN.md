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
