# Paper runs

Every experiment whose numbers appear in the paper is produced here, one
driver per table, with results kept next to the driver that made them.
Nothing in this folder is intermediate: if a file is here, it is either a
driver, a result used in the paper, or the provenance of one.

## Fixed protocol

All drivers build instances through `common.py`, so no run can silently use
a different setup than the one documented here.

| Item | Value |
|---|---|
| Slope realization | `s = 1` (graph seed 1) |
| Slope sampling | sign ~ U{-1,+1}, magnitude ~ U(0.5, 1.0) x I_e/Delta |
| Sortie time | 3 h, calibration method `campaign` |
| Energy scaling | `eta` scales the energy budget only; the mission horizon is unchanged |
| Energy tie-break | 1e-5 x normalized energy, subtracted from the reward |
| Exact model | MISOCP with tightened per-leg big-M constants; no valid inequalities |
| Gurobi | seed 42, `Threads = 0` (all cores), 3600 s per solve |
| Matheuristic | single start from R4; operators Insert, Replace, Swap, 2-opt drawn uniformly; phased local search (Insert, Replace, Swap/2-opt on shortening moves, each to exhaustion), feasible set from the labels and the energy floor, exact energy test by the taut string, roulette over the f = 5 highest weights rho^2 / c, shake at every local optimum with cap c = max(2, ceil(k/6)); accept if f(R') > f(R), reorderings if f does not decrease |
| Machine | Apple M3 Pro, 11 cores |
| Versions | python 3.12.2 | gurobi (13, 0, 2) |

The energy tie-break makes the continuous part of the solution well defined:
among schedules collecting the same reward it selects the one burning least
energy. It was calibrated on R101 (50), where removing it leaves energy at
100% and any value from 1e-5 upward settles at 96.49%, so 1e-5 is already in
the saturated regime and distorts the reported objective by about 3e-6.

## Drivers and outputs

| Driver | Paper | Output |
|---|---|---|
| `run_misocp_s1.py` | Table 3, s = 1 column (all 14 instances); Table 4; Figure 2 | `results/misocp_s1.csv`, `results/details/s1_*.json` |
| `run_misocp_seeds.py` | Table 3, s = 2, 3, 4 (`SEEDS_INSTANCES`, `SEEDS_LIST`) | `results/misocp_seeds.csv`, `results/details/seeds_*.json` |
| `run_slope_regimes.py` | Table 5, growth / decay / static (mixed = s = 1 campaign) | `results/slope_regimes.csv`, `results/details/regime_*.json` |
| `run_eta_sweep.py` | Table 6, the cells not already solved elsewhere | `results/eta_sweep.csv`, `results/details/eta_*.json` |
| `run_loitering.py` | Table 7, loitering allowed vs `L_ij = d_ij` | `results/loitering.csv`, `results/details/loiter_*.json` |
| `make_distribution_figures.py` | Figure 2 from the s = 1 records, no solver | `results/figures/distributions.png` |
| `run_tuning.py <grid>` | Section 5 parameter choices for the matheuristic | `results/tuning/<grid>.csv` |
| `run_initial_tours.py` | Table 8 (not yet run) | `results/initial_tours.csv` |

The matheuristic itself is `experiments/run_ils_time_matched.py`, which
depends on `experiments/run_ils_final.py` (instance list, kick helpers),
`experiments/run_ils_final_scored.py` (Insert and Replace with roulette-wheel
candidate order) and `experiments/run_ils_fb_cascade_demo.py` (instance
construction, F and B sets), on top of the `uav_routing` package.

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

    python3 paper_runs/run_loitering.py

Environment overrides: `LOITER_TIME_LIMIT` (seconds per solve),
`LOITER_LOGDIR` (Gurobi logs, scratch by default).

## Paper-to-code map (for checking that the paper describes the code)

| Paper | Code |
|---|---|
| 3.3 tightened time links, `M = l_i` (lower) and `l_j` (upper) | `uav_routing/solver/exact.py`, the MTZ block (`M_lo`, `M_up`) |
| 3.3 energy tie-break | `exact.py`, `energy_tiebreak` (default 1e-5) in the objective |
| 4.2 R1 to R4 | `uav_routing/local_search/initial_solution.py`, `build_R1` to `build_R4`; ratio (R4) in `build_R4` |
| 4.3 operators | `experiments/run_ils_time_matched.py`, `run_start_phased()` (Insert, Replace, Swap, 2-opt phases) |
| 4.4 labels, feasible set | `run_ils_fb_cascade_demo.compute_epsilon`, `run_ils_final_scored.compute_beta`, `feasible_insert_pairs`, `feasible_replace_pairs` |
| 4.4 exact energy test (taut string) | `run_ils_final_scored.taut_min_energy`, applied in `TimedILS.evaluate()` before every solve |
| 4.4 weight rho^2 / c, f = 5, roulette | `run_ils_final_scored._slot_reward`, `_slot_cost`, `_weight`, `draw_pair` (`RCL = 5`) |
| 4.5 phases, acceptance, local optimum | `run_start_phased()`: `insert_phase`, `replace_phase`, `reorder_phase` (`shortening_reorders`) |
| 4.6 shake, cap c = ceil(k/D) | `sweep_remove` (`--sweep-cap-div`, D = 6); Gunawan escalation with restart: `--shake-schedule gunawan --restart-threshold 10` (under evaluation) |
| 4.7 ILS loop | `run_start_phased()` main loop: local search to a local optimum, shake, best kept |
| previous design (random operator draw, theta, kappa_max) | `--local-search random`: `run_start()`, `propose()`, `add_fb_cascade_scored`, `swap_fb_cascade` |

To verify a component, read the paper paragraph, open the code location, and
run one instance with `python3 experiments/run_ils_time_matched.py --instance
"R101 (50)" --budget 60 --single-start --init R4 --ruin-mode sweep
--sweep-cap-div 6`; the run prints the per-operator counters and the shake
count, and reaches the proven optimum 11921.13 on this instance within a
minute.
