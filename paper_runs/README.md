# Paper runs

Every number of Section 5 is produced here, one driver per table, with the results
kept next to the driver that made them. `results/README.md` says what each result
file is and which table it feeds; `experiments/RERUN_PLAN.md` is the runbook that
produces the matheuristic tables from scratch.

## Fixed protocol

All drivers build instances through `common.py` (exact solver) or
`experiments/run_ils_fb_cascade_demo.make_instance` (matheuristic), so no run can
silently use a different setup than the one documented here.

| Item | Value |
|---|---|
| Slope realization | `s = 1` (graph seed 1) |
| Slope sampling | sign ~ U{-1,+1}, magnitude ~ U(0.5, 1.0) x I_e/Delta |
| Sortie time | 3 h, calibration method `campaign` |
| Energy scaling | `eta` scales the energy budget only; the mission horizon is unchanged |
| Energy tie-break | 1e-5 x normalized energy, subtracted from the reward |
| Exact model | MISOCP with tightened per-leg big-M constants; no valid inequalities |
| Gurobi | seed 42, `Threads = 0` (all cores), 3600 s per solve |
| Matheuristic | single start from R4; operators Insert, Replace, Swap, 2-opt drawn uniformly; feasible sets from the leg sets, the exact slot test (Insert, Replace) and the O(1) segment-concatenation test (Swap, 2-opt), energy floor while the set is built and the taut-string test on the drawn move; weights: route-wide midpoint estimate over the detour (Insert, Replace), exchange value (Swap, 2-opt); only the L_r = 20 reorderings of largest weight enter the set, ties to the larger distance saving; roulette over the set; accept Insert/Replace on f(R') > f(R), reorderings on an improvement or a tie with smaller cap; shake at every exhausted route on the reference local optimum, removals enumerated per size (one sweep of the route per cons = 1..ceil(k/3)), knapsack look-ahead over the next 6; stop after S = 100 consecutive shakes without an improvement of R_best; fixed-tour SOCP in nondimensional units (Presolve 0, homogeneous barrier). ILS seed 42. `paper_runs/run_design.py new` |
| Machine | Apple M3 Pro, 11 cores; one ILS run at a time, never next to a MISOCP solve |
| Versions | python 3.12.2, gurobi 13 |

The energy tie-break makes the continuous part of the solution well defined:
among schedules collecting the same reward it selects the one burning least
energy. It was calibrated on R101 (50), where removing it leaves energy at
100% and any value from 1e-5 upward settles at 96.49%, so 1e-5 is already in
the saturated regime and distorts the reported objective by about 3e-6.

## Drivers and outputs

| Driver | Paper | Output |
|---|---|---|
| `run_misocp_s1.py` | `tab:scalability` (s = 1), `tab:tour-r101-100`, MISOCP columns of `tab:matheuristic-vs-exact` | `results/misocp_s1.csv`, `results/details/s1_*.json` |
| `run_misocp_seeds.py` | `tab:scalability`, s = 2, 3, 4 (`SEEDS_INSTANCES`, `SEEDS_LIST`) | `results/misocp_seeds.csv`, `results/details/seeds_*.json` |
| `run_slope_regimes.py` | `tab:slope-effect`, growth / decay / static (mixed = the s = 1 campaign) | `results/slope_regimes.csv`, `results/details/regime_*.json` |
| `run_eta_sweep.py` | `tab:eta-sweep`, the cells not already solved elsewhere | `results/eta_sweep.csv`, `results/details/eta_*.json` |
| `run_loitering.py` | `tab:loitering`, loitering allowed vs `L_ij = d_ij` | `results/loitering.csv`, `results/details/loiter_*.json` |
| `make_distribution_figures.py` | arrival and speed distributions of the s = 1 records (not in the current manuscript) | `results/figures/` |
| `run_design.py new` | every matheuristic table: `tab:matheuristic-vs-exact`, `tab:fixed-speed`, `tab:coverage`, `tab:theta`, `tab:initial-tour`, and the traces of both ILS figures; knobs in its docstring | `results/design_new<SUFFIX>.csv`, `results/details/design_traces/`, `results/details/dynamics/` |
| `run_design.py old` | the 2026-09-26 design on the same machine, for the comparison in `experiments/DESIGN.md` only | `results/design_old.csv` |
| `fill_tables.py` | writes the bodies of the five matheuristic tables into `paper/ArXiv-version.tex` from the CSVs | the tex |
| `make_convergence_figure.py` | `fig:ils-convergence` | `paper/fig/meta_convergence_timematched.png` |
| `make_capdiv_figure.py` | `fig:ils-capdiv` | `paper/fig/ils_capdiv.png` |
| `analyze_traces.py` | Section 5.8 evidence from traces: acceptance rate by rank, shake gaps, S calibration | console |
| `compare_runs.py` | two campaign CSVs side by side (reproduction check, variant against reference) | console |
| `reorder_share.py` | Section 5.1: share of position pairs of the best routes whose Swap or 2-opt keeps every window reachable | console |

The matheuristic itself is `experiments/run_ils_time_matched.py`, which depends on
`experiments/fast_sets.py` (feasible sets and weights in O(1) per move),
`experiments/run_ils_final.py` (instance list), `experiments/run_ils_final_scored.py`
(taut-string energy test) and `experiments/run_ils_fb_cascade_demo.py` (instance
construction, leg sets), on top of the `uav_routing` package.

## Self-validation

The drivers assert, on the returned solution rather than on the model:

* flown length is never below the straight-line distance;
* under the no-loiter model, flown length equals the straight-line distance
  on every leg, so `L_ij = d_ij` really holds;
* total energy never exceeds the eta-scaled budget;
* `eta` scales the energy budget and nothing else.

A violated assertion stops the campaign at the first solve rather than after
hours of unusable runs.

## Reproducing

    python3 paper_runs/run_loitering.py                       # one exact-solver table
    python3 paper_runs/run_design.py new                      # the matheuristic reference run
    python3 paper_runs/fill_tables.py                         # tables into the tex

Environment overrides of the exact drivers: `LOITER_TIME_LIMIT` (seconds per solve),
`LOITER_LOGDIR` (Gurobi logs, scratch by default). The matheuristic driver's knobs are
listed in its docstring and used step by step in `experiments/RERUN_PLAN.md`.

## Paper-to-code map, Section 4

| Paper | Code |
|---|---|
| 4.1 nondimensional subproblem, solver settings | `uav_routing/solver/socp.py`, `Solver(scaled=True)` (set through `instance.socp_scaled`; runner flag `--scaled-socp`) |
| 4.2 construction heuristics R1 to R4 | `uav_routing/local_search/initial_solution.py`, `build_R1` to `build_R4`; each candidate is evaluated by the subproblem of the run (variable or fixed speed, loitering or `L_ij = d_ij`) |
| 4.3 leg sets, arrival bounds, slot test | `experiments/fast_sets.py`: `chain_bounds`, `chained_sets`, `build_add`, `build_replace` |
| 4.3 reorderings in O(1) (segment summaries D, W, L) | `fast_sets.build_two_opt` (prepend), `fast_sets.build_swap` (append) |
| 4.3 propagated information estimate | `fast_sets.delta_insert`, `fast_sets.delta_replace` |
| 4.3 2-opt weight by the recursion S(p,q) = x(p,q) + S(p+1,q-1) | `fast_sets.build_two_opt` |
| 4.3 restricted candidate list L_r for Swap and 2-opt, ties to the distance saving | `run_ils_time_matched.py`, `--reorder-rcl` (`heapq.nlargest` on `(exchange value, -dd)` before the roulette) |
| 4.3 roulette, zero-weight fallback | `run_ils_time_matched.py`, the draw loop of `run_start_paper` |
| 4.4 acceptance (strict for Insert/Replace; improvement or tie with smaller cap for reorderings) | `run_start_paper`, the `# ---- acceptance criterion ----` block (`ILS_REORDER_ACCEPT=tie`) |
| 4.4 enumeration E(R^ref), block removal, look-ahead | `TimedILS._enumeration`, `TimedILS._remove_block`, `--sweep enum`, `ILS_SHAKE_KNAP=6` |
| 4.4 termination after S idle shakes | `--max-idle-shakes`; `idle_shakes` reset in `_record_best` |
| identity check of the O(1) construction | `experiments/validate_fast_sets.py` (same sets and weights as the previous construction on trace routes) |
| SOCP settings check | `experiments/test_scaled_socp2.py` (feasibility verdicts against the physical model and the taut string, barrier iterations, time) |
| campaign | `paper_runs/run_design.py new` |
