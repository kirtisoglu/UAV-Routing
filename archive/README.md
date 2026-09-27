# Archive

Scripts and results that belong to designs the paper no longer describes. Kept
for provenance, not on the working path, so they cannot be run or read by
mistake.

## scripts

- `fill_paper.py` writes tables and figures into the papers using the labels and
  column layout of the pre-2026-09-18 design. Running it would overwrite the
  current tables with stale structure.
- `run_theta.py` swept the shake threshold. The threshold no longer exists: the
  shake fires when the four feasible sets of the route are exhausted.
  `paper_runs/run_capdiv.py` is its replacement and sweeps the removal cap.
- `make_perturbation_figure.py` drew the old threshold-by-divisor figure.
  `paper_runs/make_capdiv_figure.py` replaces it.
- `run_cap.py`, `run_tuning.py`, `run_ils_section2.py` are earlier campaign
  drivers with no current caller.

## results

- `superseded/prefix_solver/` predates the 2026-09-14 solver fix and is
  pessimistic throughout.
- `superseded/phased_design/` and `superseded/old_design/` come from designs
  before the weight became information over distance added.
- `initial_tours.csv` and `capdiv.csv` here are the runs of Tables 8 and 9 made
  before that weight change.
