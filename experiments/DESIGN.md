# ILS matheuristic — operator design state

State as of the most recent session. Captures (a) the algorithm we have landed
on, (b) what each remaining experiment script does, (c) what was tested and
rejected, and (d) open design choices that still need user decisions.

## Change round of 24 September 2026 — paper first, then code

Four changes requested, to be written into `ArXiv-version.tex` Section 4.3 first
and implemented in `run_ils_time_matched.py` afterwards.

| # | Change | Paper | Code |
|---|--------|-------|------|
| 1 | Replace = remove + insert. Drop the admission filter `Ibar[u] > r_{r_p}(a_{r_p})`; for each removal `p`, build the insertion move set of `R (-) r_p` and draw it by the insertion rule. | done | done |
| 2 | Realized window `Wbar_i = W_i ∩ [a_min_i, a_max_i]`. Empty anywhere rejects the move (unchanged test, new name). New: the slope gives `I^-_i, I^+_i` over `Wbar_i`; the weight's information change is the change of the route-wide sum of their midpoints. | done | done |
| 3 | Report every cache mechanism in the paper: leg sets `F_i, B_i`; per-route feasible sets; per-route quantities (arrival bounds, realized windows, info estimates, `f, T_R, E_R, cap`); evaluated-route store; shared Gurobi environment. | done | n/a |
| 4 | Swap and 2-opt keep every leg-feasible reordering: no `Δd < 0` filter. Operator choice stays uniform-random; the shake still fires when the four sets are simultaneously built and empty (already the paper's rule). | done | done |
| 5 | Termination by `maxIter` consecutive iterations without an improvement of `R*`, not by a wall-clock budget. The budget becomes a safeguard and the stopping time is reported. The limit ends the **run**, not just the current start (`run()` no longer begins a fresh initial tour). | done | done |

Consequence of 2 and 4 that needed a decision (flag for review): with a
route-wide, signed `ΔI_m` and a `Δd_m` of either sign, `ΔI/Δd` is no longer a
valid weight. Adopted in the paper: cost `c(m) = d_0 + max(Δd_m, 0)` (a move
that frees distance is charged nothing, `d_0` = 1% of the mean leg length),
score `s(m) = ΔI_m / c(m)`, and roulette weights
`w(m) = s(m) − min s + eps (max s − min s)` over the set being drawn from
(Goldberg linear scaling; keeps every move's probability positive).

Implementation (all in `experiments/run_ils_time_matched.py`, `run_start_paper`):
* `realized(cr)` walks the forward `a^min` and backward `a^max` recursions of a
  candidate route and returns `(feasible, sum of realized-window midpoint
  rewards)`. It replaces `label_feasible_add`, `label_feasible_replace` and
  `cascade_feasible_route` in the paper path: the exact window test now comes
  free with the weight, so `--paper-label` no longer changes anything.
* `score_of(dI, dd) = dI / (d0 + max(dd, 0))`, `self.d_floor = 0.01 * mean leg
  length`, `SCALE_EPS = 0.01`. The roulette recomputes the shift over the
  remaining set at every draw, since the set shrinks as moves are drawn.
* Move trace gained `w_chosen` and `w_sum` columns, so a reader can turn the
  listed weights into probabilities (the animation panel showed raw weights
  that did not sum to one).
* `--max-iter` / `ILS_MAX_ITER`; `self.idle` is reset in `_record_best`.

Calibrating `maxIter`: a run whose limit never binds records every improvement
in `*_trace.csv` as `(wall_s, iter, best_obj)`, so the objective and stopping
time for **any** candidate `maxIter` can be reconstructed exactly from one
run. No need to guess the value before the experiment.

**Bug found 24 Sep 2026 in the evaluated-route store, present in every result
reported before this date.** At draw time the code discarded any candidate
whose route was already in `self.cache`, whichever value was stored. But
`f(X)` is a property of the route `X` alone: a route refused as worse than
`f(A)` may well improve a *different* current route `B` with `f(B) < f(X)`.
The search could therefore never move to it. ~14% of draws were silently
discarded this way (1 323 of ~9 257 iterations in a 90 s probe).

Fixed: only a route stored as **infeasible** is dropped from the set. A route
stored with a value goes through `evaluate()`, which answers from the store
with no solve; the acceptance test is decided on the stored value and a solve
is paid only when the move is accepted, since the accepted state must carry
the schedule (`_resolve`). New counters `cache_worse` (refused with no solve)
and `sets_resumed`.

Also added, per the same discussion: the feasible sets and route quantities of
a route now survive leaving it (`self._mstore`, an LRU of `ILS_MSTORE_MAX`=64
routes), so returning to a route resumes its sets with the drawn moves still
removed instead of rebuilding them and redrawing.

Effect on R104 (100), 90 s, everything else equal: **32 917.30 -> 37 858.30**
(39 targets), i.e. a 90-second run now beats the old design's full 3 600 s
result of 37 802.1.

Code notes found while taking the cache inventory:
* `evaluate()` in `run_ils_time_matched.py` has the `socp_numeric_infeasible`
  counter block **duplicated four times** (lines ~682–691), so that counter
  reads 4x. Diagnostic only, no effect on results. Fix with the change round.
* `--set-cache` (`self._set_best`, target-set -> best objective) is off by
  default and unused in every reported run; not described in the paper.
* `self._lo_seen` only feeds the `lo_repeat` counter; not a search mechanism.

## 0. Time-matched protocol and headline results (July 2026 session)

The paper's matheuristic-vs-exact table now uses a **time-matched protocol**:
K independent single-threaded ILS workers per instance, each with the same
3,600 s wall-clock cap as the MISOCP (which uses all 11 cores; K <= 6, so the
ILS never consumes more core-time). Worker 0 seeds from R4, workers >= 1 from
R3 with worker-unique seeds; best across workers is reported. The inner ILS is
UNCHANGED from Section 1 below (scored sampling, add/replace, segment kicks
{2,3}, hill-climbing, t_improve=10, tabu 10).

Final numbers (seed 1, eta = 1, vs MISOCP from Table 4):

| instance | ILS best | t_best | MISOCP | Δ |
|---|---|---|---|---|
| R101 (50) | 11,902.02 | 4 s | 11,921.13 (opt) | +0.16% |
| R101 (100) | 22,884.39 | 135 s | 22,886.93 (opt) | +0.01% |
| R1_2_1 (200) | 16,949.96 | 1,130 s | 16,955.66 (opt) | +0.03% |
| C101 (50) | 8,504.33 | 1,168 s | 8,594.17 (3600 s inc.) | +1.05% |
| C101 (100) | 11,102.74 | 782 s | 11,259.65 (3600 s inc.) | +1.39% |
| **C1_2_1 (200)** | **11,000.35** | 2,153 s | 10,723.91 (3600 s inc.) | **−2.58% (beats)** |

On C1_2_1 (200) five of six workers crossed the incumbent (first at 2,080 s);
the value 11,000.35 is saturated across every alternative mechanism tested
(Section 5b), i.e. near the practically attainable optimum. The huge C-class
MIP gaps are dual-bound artifacts: the LP bound (~19,466 on C1_2_1 (200))
matches the top-60 nodes perfectly timed, which is energy-infeasible; timing
on the ILS tour is ~90% of its own perfect-timing ceiling and energy is 100%
utilized. A warm-started MISOCP experiment (run_warmstart_misocp.py)
quantifies this for the paper.

### New keeper scripts (this session)

- `run_ils_time_matched.py` — wall-clock-budgeted ILS runner (TimedILS):
  multi-start, elite restarts, eval cache, energy-LB prefilter,
  `--seed-offset` for parallel workers. Optional (off by default, all tested
  worse or neutral): `--escalate-kicks`, `--relocate`, `--tilt`,
  `--ruin-mode reward`.
- `run_bestof_ils.py` — best-of-K parallel driver; aggregates worker
  summaries into `bestof_<instance>_<tag>.csv`.
- `run_warmstart_misocp.py` — injects the ILS tour + its oracle solution as
  a complete MIP start into the exact model (certification + bound-artifact
  experiment). Uses `exact.py`'s `warm_tour`/`warm_arc_solution` params.
- `run_fig_timematched.py` — paper figure: worker trajectories vs wall-clock
  with the MISOCP incumbent line (fig/ils_timematched_c1_2_1.png).

Kept result files: `bestof_*_final*.csv` (5 instances),
`bestof_c1_2_1_200_prod*.csv` (headline), `tm_ils_*_final/prod_*` worker
summaries/traces, `warmstart_misocp_results.csv` + `warmstart_*.gurobi.log`.

### Warm-start + bound-artifact experiments (paper subsec:warmstart)

Warm-started MISOCP (ILS tour + oracle solution injected as a complete MIP
start via exact.py `warm_tour`/`warm_arc_solution`; 3,600 s, seed 42):

| instance | ILS start (certified) | warm obj | warm bound | warm gap | cold obj | cold gap |
|---|---|---|---|---|---|---|
| C101 (50) | 8,504.33 | 8,595.41 | 8,899.25 | 3.53% | 8,594.17 | 2.53% |
| C101 (100) | 11,102.74 | 11,331.02 | 15,041.31 | 32.74% | 11,259.65 | 33.03% |
| C1_2_1 (200) | 11,000.35 | 11,162.49 | 21,433.42 | 92.01% | 10,723.91 | 81.52% |

Gurobi loaded every start within 5e-2 of the oracle value (certification).
Warm incumbents are the best known solutions on all three instances; the gap
barely moves -> the C-class gap measures certification failure.

No-energy probes (eta = 1000, run_noenergy_probe.py): ILS 900 s single climb
reached 11,751 (C101 (100)) / 11,380 (C1_2_1 (200)); exact 3,600 s reached
12,129 (gap 54.8%, bound 18,777) / 10,466 (gap 184%, bound 29,725). Best
known no-energy rewards (12,136 / 12,188, from the eta=1.5 sweep) sit 24% /
60-76% BELOW the with-energy dual bounds -> bound looseness is structural
(big-M time linking + fractional selection), not energy-blindness.

### Value-of-speed-optimization ablation (paper tab:fixed-speed)

Same time-matched protocol with v_min = v_max = v_mr (loitering allowed),
via `run_ils_time_matched.py --fixed-speed` / `run_bestof_ils.py --fixed-speed`:

| instance | fixed-speed | variable-speed | gain |
|---|---|---|---|
| R101 (50) | 10,540.63 (9) | 11,902.02 (10) | +12.9% |
| R101 (100) | 19,453.83 (14) | 22,884.39 (16) | +17.6% |
| R1_2_1 (200) | 12,066.59 (20) | 16,949.96 (32) | +40.5% |
| C101 (50) | 7,463.05 (36) | 8,504.33 (41) | +14.0% |
| C101 (100) | 9,875.76 (37) | 11,102.74 (45) | +12.4% |
| C1_2_1 (200) | 8,950.68 (40) | 11,000.35 (51) | +22.9% |

Speed optimization is worth 12-40% of objective (grows with instance size);
dwarfs loitering value (<= 12.5%). Data: bestof_*_fs.csv + tm_ils_*_fs_*.

Exact-side companion (paper tab:fixed-speed-exact, run_loitering_fixedspeed.py):
fixing v = v_mr COLLAPSES the MISOCP to a MILP (cones vanish; energy becomes a
second time budget sum t <= E_max/P(v_mr)). Solving the MISOCP with
v_min = v_max instead fails with Gurobi numeric errors (degenerate cones, no
interior) -- do NOT do that. MILP optima on the loitering grid (all certified,
gap 0.00%): speed-opt gain 11.7-26.0% across R-class x eta in {0.75,1,1.25};
fixed-speed optima saturate at eta = 1 (energy budget = flight-time budget at
v_mr by calibration). Oracle validation: exact MILP re-timing of the fs-ILS
tours reproduces the SOCP oracle values to +0.0000 (R101(50): ILS attained the
proven fixed-speed optimum 10,540.63; R1_2_1(200): ILS 12,066.59 vs proven
optimum 14,174.66, i.e. the 40.5% heuristic-side gap is ~half model value,
~half search difficulty). Data: loitering_fixedspeed.csv.

Known code quirk (fix at the instance-expansion re-run, NOT before): the
experiment scripts pass the RAW Solomon graph to the cascade/F/B filters while
T_max is physical, so the cascade filtered more weakly than designed; results
are unaffected (the SOCP oracle on instance.graph is the feasibility gate;
cross-checks above confirm), but fixing it perturbs RNG trajectories, so all
heuristic tables should be regenerated together in the expansion sweep.
Figure scripts: run_fig_timematched.py (headline C1_2_1 panel),
run_fig_convergence_tm.py (6-panel wall-clock convergence, replaces the old
iteration-indexed meta_convergence in the paper).

### Tested and rejected in the time-matched setting (evidence)

- **Escalating kick tiers** ({2,3}→{4,6}→{8,12}): wrecks large instances —
  C1_2_1 (200) single climb 9,546 vs 10,594 with fixed {2,3}. An 8–12-node
  removal from a ~30-node tour is a near-restart hill-climbing cannot repair.
- **Frequent restarts** (restart_stall 15k–25k proposals): fragments the
  budget; every climb was cut at ~27% of its plateau. Long climbs win.
- **Best-insertion proposals** (Solomon I1-style, top-K): 9,614 vs 10,030
  for scored sampling at 10k iters on C101 (100) — over-greedy, locks in.
- **Tilt (worse-move acceptance p≈0.1) + relocate**: best-of-4 long climbs
  reached only 10,456 on C1_2_1 (200) vs 11,000 pure — dilutes the climb.
- **Reward-guided ruin** (drop lowest-contribution nodes): 10,941 best-of-8
  — beats the incumbent but below pure segment kicks.
- **Or-opt-1 relocation as a pool operator**: SOCP-feasibility ~3% on
  C-class (measured; swap/2-opt ~0–2%, confirming the paper's exclusion),
  but adding it did not raise the ceiling.
- **Energy lower-bound prefilter** (distance × min P(v)/v): valid bound,
  never fires on these instances (energy binds via timing, not distance);
  kept because it is free.

## 1. Algorithm we have landed on

**Outer structure**: Iterated Local Search (Vansteenwegen 2009 OPTW recipe):
hill-climbing inner search interleaved with stagnation-triggered perturbation.

**Operator filter pipeline** (the work we built):

For inserting candidate `u` between predecessor `i` and successor `j`:

1. **F/B prefilter** — precomputed once per instance:
   - `F[i] = {u : ell_u > e_i}`  (u can come after i)
   - `B[j] = {u : e_u < ell_j}`  (u can come before j)
   - Candidate set per edge `(i, j)`: `F[i] & B[j] & complement`.
2. **v_max forward cascade** — O(K) per candidate, computed against
   precomputed earliest-arrival labels `epsilon`:
   - `epsilon_u = max(e_u, epsilon[i] + d(i,u)/v_max)`. Reject if `> ell_u`.
   - If `j != depot`: cascade `epsilon` through `j, j+1, ..., depot` and check
     each `ell` along the way.
   - Final horizon check: cascade arrival at depot `<= T_max`.

This pipeline is **necessary-and-sufficient** for time-window feasibility.
Remaining SOCP infeasibility (~95% on R101 (50)) is **energy-side**, not
TW-side.

**Sampling** (the structural improvement over u-first random):

- Pick an edge `(i, j)` uniformly from the route's edges (including the
  return arc).
- Pick candidates from `F[i] & B[j] & complement` in random order.
- For each, run the cascade. Take the first that passes; insert; one SOCP
  evaluates the move.
- If no edge admits any cascade-passing candidate, declare **saturated**
  (this state was empirically never reached on R-class — the chain is
  never structurally stuck under the F/B + cascade filter).

**Operators in the local-move pool**:

- `add_random_node` — edge-first F/B + cascade.
- `replace_random_node` — position-first F/B + cascade. Predecessor and
  successor neighbors define `(i, j)`.
- ~~swap_two_nodes~~, ~~swap_two_opt~~ — confirmed ~0% SOCP-feasible on
  TW-tight routes (reversing or swapping inverts the time order). Dropped.
- `remove_segment` — consecutive `k in {2, 3}` nodes removed. Open design
  question: is this the perturbation, or an additional local-move operator?

**Acceptance**: hill-climbing (strict improvement). Optional tilted run
(p = 0.3 of accepting a worsening move) — small effect, harmless to keep.

**Perturbation**: open design — see Section 4.

**Stagnation**: original semantics restored (per user guidance). `worse_rej`
(SOCP-feasible but no improvement) increments the counter; infeasibility
does NOT. Infeasibility means "the operator picked a dead candidate", not
"we are at a local optimum".

## 2. Package-side changes (these landed in `uav_routing/`)

These are persistent edits to the codebase, not just experiment scripts:

- `local_search/state.py`: added `"last_operator"` to `State.__slots__`,
  default `None` in both init paths.
- `local_search/proposal.py`: `universal_proposal`, `random_flip_with_tabu`,
  and `perturb_state` now set `last_operator` on the returned state.
- `local_search/optimization.py`: added `Tally` class. `Optimizer.run_ils`
  now takes a `tally: bool = False` parameter; when `True`, `self.tally`
  is populated with per-operator attempt and outcome counters.

These changes are independent of the experiment scripts and are required
by any script that uses `tally=True`.

## 3. Keeper scripts in `experiments/`

### Operator-design experiments (the current algorithm work)

- `run_ils_fb_cascade_demo.py` — F/B + cascade, hill-climbing. The
  clean baseline of the working design. Two instances (R101 (50) and
  R101 (100)). Reports per-operator tally with SOCP-feasibility column.
- `run_ils_fb_cascade_sa_demo.py` — same, plus tilted-run acceptance
  (p = 0.3). Single instance R101 (50). Misnamed "_sa_" historically;
  it is a tilted run, not SA.
- `run_ils_fb_cascade_seg_demo.py` — same, plus `remove_segment` as a
  fifth local-move operator. Open question: should this REPLACE the
  existing perturbation harness instead?

### Paper-table generators (unrelated to operator work; do not touch)

- `run_loitering_clustered.py` — C-class loitering value (paper Table 10).
- `run_arrival_distr_clustered.py` — C-class arrival distribution (Fig 8).
- `run_slope_regime_clustered.py` — C-class slope-regime sensitivity (Table 6).
- `run_scalability_clustered.py` — single-seed C-class scalability.
- `run_scalability_seeds.py` — multi-seed scalability (Table 4, seeds 1-4).
- `run_initial_tour_selection.py` — initial tour selection (Table 7).
- `run_alpha_scalability.py`, `run_alpha_loitering.py` — alpha = 2, 3
  sensitivity for the "stronger slopes" future-work pitch.
- `run_alpha_loitering_r101_100_seeds.py` — multi-seed alpha sanity check.

### Pre-existing scripts

- `run_ils_section2.py` — older paper-side ILS script. Not part of the
  current operator work.
- `run_scalability_move_a.py` — Move A scalability test, pre-existing.
- `test_move_a.py` — Move A unit test.

## 4. Design decisions (resolved end of session)

1. **Perturbation**: REPLACE existing `perturb_state` random-k removal
   with consecutive segment removal. User specified "we should be able
   to turn back" — implementation keeps both perturbation modes
   available via a `perturbation_mode` parameter, default = "segment".
   To revert: pass `perturbation_mode="random_k"`.

2. **Tilted run**: OFF. Pure hill-climbing acceptance.

3. **`t_improve`**: lowered to **10**. With pure hill-climbing and no
   infeasibility-counts-as-stagnation, this is high enough to give
   inner local search a chance to find improvements, low enough that
   perturbation actually fires.

4. **Local-search style**: random per-iteration. Classical
   first-improvement-to-exhaustion is structurally cleaner but at
   SOCP budget = 1000 only completes 1 cycle; deferred until SOCP
   budget is increased to >= 5000. Decision was delegated to me.

## 5. What was tested and rejected (briefly)

- **Stagnation increment on SOCP infeasibility**: misread of the spec;
  reverted. Infeasibility does not signal local optimum.
- **v_mr energy proxy**: lower bound on tour energy at minimum-energy-
  per-meter constant flight. Too loose for R101-style instances —
  threshold of 478,548 m is well above typical tour distances; never
  fires. Would need cascade-aware per-arc energy estimate to be useful.
- **Tighter geometric TW filter** (vs simple `ell_u > e_i`): cascade
  catches 100% of TW failures; geometric filter alone is sufficient on
  R101 because TWs are tight, but no improvement over cascade.
- **Tree-lifted MCMC** (FalcomChain-style): legitimate, deferred to
  follow-up paper. The directed precedence tree where parent(v) is the
  latest-finishing feasible predecessor is the recommended construction.
- **Classical first-improvement until exhaustion** at SOCP budget 1000:
  burns budget on negative-confirmation sweeps; needs ~5000 SOCPs for
  multiple cycles, or capped-round variant.

## 6. Open empirical questions

- SOCP feasibility on the F/B + cascade pipeline plateaus at ~5% across
  all R-class operators. The remaining 95% rejection is energy-side.
- v_mr energy lower bound is too loose to catch this. Tighter proxy
  would use the cascade-derived arrival times to estimate per-arc
  required speed, then `E(d_ij, v_required)` summed across arcs.
- The diagnostic showed at a 7-node R101 (50) steady-state:
  76 cascade-passing candidates, 9 SOCP-feasible (11.8%), concentrated
  on just 2 of the 8 edges. Most edges are "dead zones" no node can
  fit into. This motivates segment-removal perturbation (to create
  fresh edges) and tree-structured insertion (to learn dead-zone
  patterns).

## 7. Suggested next steps

1. **User to decide** the four open design choices in Section 4.
2. **Apply the decision** to a single experiment script (probably
   `run_ils_fb_cascade_seg_demo.py` or a new `run_ils_final.py`).
3. **Run on R101 (50) and R101 (100)** at SOCP budget 1000, report the
   final tally and objective.
4. **Energy proxy v2** (cascade-aware): build a per-arc minimum-energy
   estimate using the cascade's earliest-arrival times to pin down
   v_required per arc, then sum. This is the next algorithmic
   improvement after the structural decisions above are settled.
5. **Tree-lifted MCMC** is the follow-up paper. Do not start until the
   current paper is shipped.

## 8. Rerun campaign (September 2026)

The benchmark grew to 14 instances and the algorithm changed (four operators drawn uniformly, sweep shake, single-start protocol). The full experiment plan, ordering, budget and open decisions are in `experiments/RERUN_PLAN.md`.

## Change round of 27 September 2026 — Section 4 design (paper first, code second)

Diagnosis from the logs and traces of the 2026-09-26 base (paper_runs/results/newdesign,
animation/traces):

* Where the time goes. On short routes the SOCP dominates; on the long routes the
  Python construction of the feasible sets did: O(k) `realized()` per reordering pair
  (O(k^3) per route) and per surviving Insert/Replace candidate. Measured on trace
  routes: PR15 3.36 s -> 0.39 s, C104 1.42 s -> 0.12 s per route with the O(1)
  construction (`validate_fast_sets.py`, identical sets and weights on 840/840 sets).
* The SOCP in physical units needs NumericFocus 3 and takes 56-87 barrier iterations;
  in the nondimensional units of Section 5.3 with Presolve 0 + BarHomogeneous it takes
  11-18 and agrees with the physical model and the taut string on every tested route
  (`test_scaled_socp2.py`). Presolve, not the units, causes the false infeasibility.
* Acceptance rate by rank of the drawn move (traces, 14 instances): for Insert and
  Replace it does NOT decay with the weight rank (R104 Replace: 8% at rank 1, 24% at
  ranks 21-50), so a shortlist of insertions loses improving moves; for Swap and 2-opt
  it does decay (PR15: 17% at rank 1, 3-5% beyond rank 20). Hence L_r on reorderings
  only. A full RCL/VND descent (`run_ils_rcl.py`, deterministic first-improvement over
  top-L lists) was implemented and REJECTED: R101 (50) 10 850 vs 11 902, R101 (100)
  22 386 vs 22 884 — the random-order full descent is what finds the improvements.
* Improvements of R_best arrive at all depths of a descent (PR15: 96 of 124 within 200
  iterations of a shake, 28 later), so the descent is not truncated.
* Shake gaps between successive improvements: at most 88 shakes (PR11), otherwise
  <= 46; idle shakes after the last improvement under maxIter = 13000: 12 to 276.
  Hence termination by S = 100 consecutive idle shakes (`--max-idle-shakes`), which
  keeps every improvement of the final runs and cuts the idle tails.
* The (post, cons) arithmetic of the previous shake repeats its first pair after
  cons(cons+1)/2 = k removals (k = 10, c = 4), and the code compared unreduced `post`
  values, so "exhausted" almost never fired. Replaced by the explicit enumeration
  (`--sweep enum`): one sweep of the route per cons = 1..c, then the next level.
* Caveat found on the way: the energy tie-break coefficient 1e-5 is below the barrier's
  optimality tolerance on f ~ 2e4, so among information-equivalent schedules the
  returned E_R (and hence cap) is solver-dependent on degenerate routes (energies of
  the two models differed by 3.5% of E_max on one R101 route at equal objective).
  The capacity tie-break of the reordering acceptance reads this noise.

Adopted (all opt-in flags of `run_ils_time_matched.py`, campaign `paper_runs/run_design.py`):
`--fast-sets --scaled-socp --reorder-rcl 20 --sweep enum --max-idle-shakes 100`, stall off.
Same-machine checks in the cloud container (pip Gurobi license, routes <= 32 targets):
R101 (100) `--fast-sets` alone reproduces the base trajectory exactly (22 884.32 at
iteration 351, 13 351 iterations); R102 (100) base 30 443.71 at 104 s / 252 s run vs new
30 592.27 at 10 s / 29 s run (208 shakes). RC104 (100) and R1_2_1 (200): see the
session report. The full 14-instance campaign needs the licensed machine.

Addendum (same day). The first explicit enumeration was level by level (all single-target
removals, then all pairs, ...). On R1_2_1 (200) it fell into a 27-target basin (14 716 vs
16 950 for the base on the same machine): the first k shakes remove one target and the next
k/2 remove two, the removals that improve the best in about 1% of the cases, whereas the
(post, cons) arithmetic escalates at every shake. `_enumeration` now interleaves the levels
(next block of size 1, of size 2, ..., of size c, and again), which keeps the escalation and
the complete coverage. Section 4.4 describes the interleaved order.

Same-machine check (cloud container, pip Gurobi license: routes above 32 targets are refused
and counted as infeasible, so R1_2_1 is not a clean comparison; single runs, seed 42):

| Instance | 2026-09-26 base: best (t_best / run) | Section 4 design: best (t_best / run) |
|---|---|---|
| R101 (100) | 22 884.32 (1.6 s / 47 s) | 22 884.44 (0.4 s / 7 s) |
| R102 (100) | 30 443.71 (104 s / 252 s) | 30 443.78 (66 s / 95 s) |
| RC104 (100) | 35 823.27 (238 s / 444 s) | 36 231.33 (34 s / 55 s) |
| R1_2_1 (200) | 16 950.18 (57 s / 163 s), 356 refused solves | 16 367.83 (18 s / 35 s), 103 refused solves |

Time per SOCP solve in the loop: 20-40 ms physical (NumericFocus 3) vs 3.6-7 ms scaled.
The long-route instances (C104, PR15, PR10), where the O(1) construction matters most,
exceed the license here and are to be run with `paper_runs/run_design.py` on the licensed machine.

Local-machine results relayed by the author (2026-09-27), new design vs 2026-09-26 base:

| Instance | base | new, L_r = 20 | new, L_r = 0 (no list) |
|---|---|---|---|
| PR11 (48) | 6 519.29, 227 s, 295 shakes | 6 473.95, 17 s, 146 shakes | 6 509.95, 43 s, 313 shakes |

L_r = 20 against the unrestricted set: +7.78% on PR15, +4.62% on PR10, +0.99% on C104,
with shakes 116 -> 691, 89 -> 311, 36 -> 381; -0.70% on PR11. Reading: on a short route
with wide windows the full reorder set is affordable and informative, and the list discards
moves that would have been accepted; on the long routes the list is what buys the shakes.
S = 100 confirmed by the author on the new design. Section 4.3 states the trade-off;
Section 5.8 is to report it (PR11 added to the L_r grid of the note).

Full campaign of the new design on the licensed machine (M3 Pro, `run_design.py new`,
D = 3, L_r = 20, S = 100; relayed by the author 2026-09-27), against the 2026-09-26 base
(paper_runs/results/final.csv). Objective / t_best / run time:

| Instance | base | new | obj change |
|---|---|---|---|
| R101 (50) | 11 902.02 / 0.6 s / 12 s | 11 902.02 / 0.1 s / 1.9 s | 0 |
| R101 (100) | 22 884.32 / 0.8 / 21 | 22 884.44 / 0.2 / 4.2 | +0.00% |
| R1_2_1 (200) | 16 586.21 / 25.5 / 77 | 16 586.35 / 9.2 / 17.8 | +0.00% |
| C101 (50) | 8 591.92 / 38.9 / 240 | 8 591.92 / 1.5 / 33.4 | 0 |
| C101 (100) | 11 153.74 / 188.3 / 281 | 11 324.41 / 110.2 / 147.3 | +1.53% |
| C1_2_1 (200) | 11 083.27 / 39.2 / 187 | 11 146.32 / 78.0 / 138.7 | +0.57% |
| RC1_2_1 (200) | 17 669.96 / 64.3 / 194 | 17 973.77 / 70.1 / 90.3 | +1.72% |
| R102 (100) | 30 443.78 / 61.6 / 135 | 30 443.78 / 42.9 / 59.2 | 0 |
| R104 (100) | 38 060.94 / 169.3 / 240 | 38 105.79 / 39.9 / 57.7 | +0.12% |
| C104 (100) | 18 233.89 / 378.9 / 587 | 18 414.60 / 217.4 / 304.3 | +0.99% |
| RC104 (100) | 35 809.19 / 45.9 / 156 | 36 231.33 / 17.2 / 27.8 | +1.18% |
| PR11 (48) | 6 519.29 / 133.3 / 227 | 6 473.95 / 6.9 / 17.3 | -0.70% |
| PR15 (240) | 18 221.45 / 578.9 / 982 | 19 639.68 / 609.4 / 704.8 | +7.78% |
| PR10 (288) | 15 958.48 / 114.6 / 354 | 16 696.23 / 128.4 / 196.6 | +4.62% |

Total run time 1 801 s against 3 693 s; t_best under 100 s on 10 instances against 8;
every run ended by the shake limit. Table 5.9 and the 5.8 parameter paragraph carry these
numbers (commit of 2026-09-27).

## Long-route check of the scaled SOCP (local, 2026-09-27)

Step 0.3 of `experiments/RERUN_PLAN.md` run on this machine with the full Gurobi
licence. The script filtered routes to at most 32 targets, which was the cloud's
pip-licence limit; with that filter PR15 and C104 contributed only short routes
(C104 contributed none at all, so it printed "0 routes"). The filter is removed.

On genuinely long routes the picture is not the clean 40/40 of the short ones:

| instance | routes | phys feasible | combo | agree w/ physical | false-infeas vs taut | ms/solve |
|---|---|---|---|---|---|---|
| PR15 (240) | 40 | 22 | default | 40/40 | 0 | 13.3 |
| | | | P0 | 40/40 | 0 | 12.8 |
| | | | **P0+BH** | **39/40** | **1** | **7.1** |
| | | | P0+BH+NF3 | 40/40 | 0 | 20.1 |
| C104 (100) | 40 | 16 | default | 39/40 | 0 | 23.7 |
| | | | P0 | 39/40 | 0 | 23.7 |
| | | | **P0+BH** | **39/40** | **0** | **5.5** |
| | | | P0+BH+NF3 | 38/40 | 1 | 16.7 |

Reading. On C104 the taut string calls 17 routes feasible and the physical model
only 16, so the single "disagreement" of P0+BH is the route where the *physical*
model is the outlier; P0+BH matches the taut string on all 40 there. On PR15 the
physical model and the taut string agree exactly, and P0+BH is genuinely wrong on
one route in forty, a false infeasibility -- the same class as the
`socp_numeric_infeasible` counter of the old design (93 occurrences on C101 (100)).

The speed argument for P0+BH is strong and is what the long routes show best:
5.5 ms against 23.7 ms on C104 (4.3x) and 7.1 ms against 12.8 ms on PR15 (1.8x).
NumericFocus 3 removes the PR15 miss but costs 3x on C104 and is worse there.

Not changed: the design keeps P0+BH, per the rule of the rerun plan. Recorded so
that Section 5.8 can state the false-infeasibility rate rather than imply 40/40,
and so the caveat list carries a measured number for long routes.

## Cloud session, 27 September 2026 (afternoon): one driver, one table writer, a runbook

Section 5.8 now states the long-route check above (one false infeasibility in forty on
PR15, none on C104, per-solve times of the two campaigns).

Code changes, none of which alters a mixed-regime run:

* `--reorder-rcl` breaks ties by the distance saving: `heapq.nlargest` on
  `(exchange value, -dd)`. Ties between exchange values have measure zero with sampled
  slopes, but with static rewards every exchange value is zero and `nlargest` kept the
  first L_r pairs in construction order, all with p = 1. Section 4.3 says so in one clause.
* `build_R4` (and the screen before it) evaluate candidates under the subproblem of the
  run (`instance.no_loiter`). Found by the smoke test of the driver: with `--no-loiter`
  the R4 tour of R101 (50) was built with loitering allowed and the run died on an
  infeasible start. The reference configuration is unaffected.
* The runner prints the flown length, energy and mission time of the best route
  (`best flown ... m energy ... J time ... s`), for the loitering and speed tables.
* `paper_runs/run_design.py`: knobs `DESIGN_INIT`, `DESIGN_INIT_SEED`, `DESIGN_DYNAMICS`;
  columns `init_obj`, `init_size`, `flown_km`, `energy_pct`, `time_s`, `commit`; a CSV
  with the earlier columns is moved to `.v1.csv` and the campaign restarts; refuses to
  run with `ILS_LICENSE_GUARD` set. Replaces `run_capdiv.py`, `run_capdiv_fixed.py`,
  `run_initial_tours.py`, `run_matheuristic.py` and `campaign/final_run.py`.
* `paper_runs/fill_tables.py` writes the bodies of the five matheuristic tables from the
  CSVs; `make_convergence_figure.py` and `make_capdiv_figure.py` read the driver's
  traces and write into `paper/fig/`. Tested end to end on R101 (50) in the container
  (driver, five tables, two figures, pdflatex clean).
* `experiments/RERUN_PLAN.md` rewritten as the runbook (seven steps, then the prose).

The greedy column f(R4) of Table 5.9 will change when Step 1 of the runbook runs: the
old column equals the fixed-speed evaluation of the R4 tour (R101 (50): 4 643.08, which
is what `--fixed-speed` reproduces), the new one is the value of the tour the run
starts from with the speed free (5 863.95).

### Paper against code, 28 September 2026 (submission pass on Section 4 and 5.1 to 5.3)

Three statements of Section 4 did not describe the code that produced the results and
were rewritten; nothing in the code changed.

* Reordering weights. The text clamped the exchange value at zero. The runner
  (`ILS_SELECT=scaled`, `ILS_SCALE_EPS=0.01`) shifts the exchange values of the moves
  still in the set onto a nonnegative scale, `x - x_min + 0.01 (x_max - x_min)`, so the
  least attractive reordering keeps a small probability. Section 4.3 now states the
  shift as equation `eq:exch-weight`, defines the exchange value `x(p, q)` once and
  uses it for Swap (`x_m = x(p, q)`) and 2-opt (`x_m = S(p, q)`, equation `eq:exch-2opt`).
* No-return rule. The text refused a reordering that returned to the previous route
  (one hop). `ILS_NO_RETURN=2` bars, at the draw, a reordering that would extend an
  A-B-A alternation to A-B-A-B, and another move is drawn. Section 4.4 and Algorithm 1
  say so.
* The evaluated-route store. The text claimed a run without the store reproduces the
  same objective at the same iteration. It does not have to: a stored-infeasible route
  is dropped and another move drawn in the same iteration, which consumes a random
  draw. The text now claims only that the store changes no acceptance decision.

Numbers replaced because no run supports them: "12 347 routes discarded against 93
infeasible" (an old-design count) by the `socp_numeric_infeasible` column of
`design_new.csv` (at most 2 per run); "at most 1.7% of 300 random proposals" by the
shares of `paper_runs/reorder_share.py` on the best routes of `design_new.csv`
(0.2 to 2.6% co-monotone, 14 to 23% inverted, under 1% Cordeau, where the packed best
routes rather than the windows fix the order); "Cordeau instances use only 78 to 86% of
the energy" contradicted `misocp_s1.csv` (98.7 to 100%) and is replaced by a reference
to the energy column of Table `tab:scalability`. The construction heuristics do not use
the window midpoint estimate (R4 solves the subproblem), and the claim was dropped.

### Strengthening the two lever tables (28 September 2026)

At eta = 1 both settings of the loitering table use the whole budget on every
instance and the gain is 0.04 to 6.2 percent, while the exact solver's R-class table
shows the gain rising as the budget tightens (R101 (50): 12.5 percent at eta = 0.75).
The mechanism: waiting without loitering means slow flight below v_mp at rising induced
power, so it wastes energy exactly when energy is scarce. The speed table shows the
opposite dependence in the same data: fixed-speed routes leave 0.4 to 8 percent of the
budget unused and schedule 3 to 29 fewer targets, since speed above v_mr is what
converts energy into targets. Added: `--eta` in the runner (`make_instance(path, eta)`),
`DESIGN_ETA` in the driver, an energy-use column in `tab:fixed-speed`, and the table
`tab:levers-eta` (gain of each lever at eta = 0.75, 1, 1.25), filled by runbook Step 8.
Smoke test on R101 (50) at eta = 0.75 in the container: reference, no-loiter and
fixed-speed variants run; the objectives fall below the eta = 1 values as they must.

Files identified as superseded, to be removed by the author (git history keeps them):
`experiments/tm_ils_*` (runner outputs, now ignored), `archive/`, `animation/traces/`,
`animation/.viewer_template.bak`, `figures/`, `fig/` (stale copies; the tex reads
`paper/fig/`), `texput.log`, `paper/ArXiv-version.{aux,bbl,blg,log,out}`,
`paper/Section4-matheuristic.tex`, `paper/Section5.1-5.2-datasets-information.tex`,
`paper/Table5.9-and-5.8-parameters.tex`, `paper/ref-additions.bib` (merged into
`ref.bib`), `paper_runs/run_capdiv.py`, `run_capdiv_fixed.py`, `show_runs.py`,
`run_matheuristic.py`, `run_initial_tours.py`, `paper_runs/campaign/`,
`experiments/run_fig_convergence_tm.py`, `run_fig_pr15_dynamics.py`, `run_ils_rcl.py`,
`experiments/initial_tour_results.csv`, and under `paper_runs/results`: `anim_exch.csv`,
`capdiv*.csv`, `verify_d3.csv`, `stall_ten.csv`, `iratio_*.csv`, `initial_tours*.csv`,
`ils_*.csv`, `final.csv`, `tuning/`, `figures/`, `details/{aba,dynamics,capdiv_traces,
theta_traces,ils_traces,gain1h_traces}`, `details/*.log`, and every `newdesign/*.log`
except `anim_*.log` and `design_*.log`. Kept on purpose: `animation/datasets/` (the
built viewer pages), the MISOCP result files, `design_*.csv`, `paper/fig/`.
