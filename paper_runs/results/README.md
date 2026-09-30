# paper_runs/results

What each file is and which table it feeds. Files not listed under "current" are
records of earlier designs; no script reads them and no table is built from them.

## Current: exact solver (Section 5.3 to 5.6, done, do not rerun)

| File | Table |
|---|---|
| `misocp_s1.csv`, `details/s1_*.json` | Table `tab:scalability` (s = 1), Table `tab:tour-r101-100`, the MISOCP columns of Table `tab:matheuristic-vs-exact`, the dashed lines of both ILS figures |
| `misocp_seeds.csv`, `details/seeds_*.json` | Table `tab:scalability`, s = 2, 3, 4 |
| `slope_regimes.csv`, `details/regime_*.json` | Table `tab:slope-effect` |
| `eta_sweep.csv`, `details/eta_*.json` | Table `tab:eta-sweep` |
| `loitering.csv`, `details/loiter_*.json` | Table `tab:loitering` |

## Current: matheuristic (Section 5.8 to 5.11), written by `paper_runs/run_design.py`

One CSV per configuration; the suffix is `DESIGN_OUT`. Every row carries the git
commit of the run. `paper_runs/fill_tables.py` reads these and writes the tables.
Since 30 September 2026 the design is the tiers weighting with the shake applying the
next removal (`experiments/RERUN_PLAN.md`); the files below are written by that
campaign, and until a step has run its file is absent and its table keeps the
previous numbers.

| File | Configuration | Table or figure |
|---|---|---|
| `design_new.csv`, `details/design_traces/<stem>_new.csv` | reference run: R4 start, D = 3, L_r = 20, S = 100 | `tab:matheuristic-vs-exact`, `fig:ils-convergence`, D = 3 block of `tab:theta`, R4 column of `tab:initial-tour`, right blocks of `tab:fixed-speed` and `tab:coverage` |
| `design_new_rep2.csv`, `details/design_traces/<stem>_new_rep2.csv` | replication 2: same start, ILS seed shifted (`--seed-offset 1`) | `tab:matheuristic-vs-exact`, replication 2 block |
| `design_new_fixed.csv` | `--fixed-speed` | `tab:fixed-speed`, left block |
| `design_new_noloiter.csv` | `--no-loiter` | `tab:coverage`, left block |
| `design_new_D6.csv`, `design_new_D12.csv` | D = 6, D = 12 | `tab:theta` |
| `design_new_R1.csv`, `design_new_R2.csv`, `design_new_R3s1.csv`, `design_new_R3s2.csv`, `design_new_R3s3.csv` | starts R1, R2, R3 with seeds 1, 2, 3 | `tab:initial-tour` |
| `details/dynamics/pr15_240_new_D{3,6,12}dyn.csv` (ignored by git, regenerated) | PR15 with the per-iteration trace | `fig:ils-capdiv` |
| `design_new_eta075.csv`, `design_new_fixed_eta075.csv`, `design_new_noloiter_eta075.csv`, and the same with `_eta125` | the reference run and both variants at `eta = 0.75` and `1.25` (runbook Step 7) | `tab:levers-eta` |
| `design_new_Lall.csv` | no restricted list (`DESIGN_RCL=0`) on the six instances where reorderings matter (runbook Step 8) | the L_r numbers of the Parameter analysis prose |
| `design_new_physical.csv` | replication 1 with the subproblem in physical units (`DESIGN_SCALED=0`, runbook Step 8) | the solve times of the Parameter analysis prose (`socp_ms` only) |

## Records (not inputs to any table)

| File | What |
|---|---|
| `design_old.csv`, `details/design_traces/<stem>_old.csv` | the 2026-09-26 design on the same machine; the comparison in `experiments/DESIGN.md` |
| `previous_design/` | every `design_new*.csv` of the previous design (exchange value for Swap and 2-opt, six-removal look-ahead in the shake), campaigns of 27 to 29 Sept 2026, and the test runs of `experiments/TEST_RUNBOOK.md` |
| `newdesign/design_*.log` | console logs of the campaigns above |
| `newdesign/anim_*.log` | logs of the viewer recordings (`animation/pack_all.py` reads them) |
| everything else (`anim_exch.csv`, `capdiv*.csv`, `verify_d3.csv`, `stall_ten.csv`, `iratio_*.csv`, `initial_tours*.csv`, `ils_*.csv`, `final.csv`, `tuning/`, `figures/`, `details/aba`, `details/capdiv_traces`, `details/theta_traces`, `details/ils_traces`, `details/gain1h_traces`, the old `details/dynamics/*.csv`, `details/*.log`, the remaining `newdesign/*.log`) | previous designs; listed for deletion by the author |
