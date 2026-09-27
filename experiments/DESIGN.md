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
