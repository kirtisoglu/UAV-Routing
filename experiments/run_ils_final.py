"""
Final operator design (per DESIGN.md, session decisions).

Algorithm:
  - F/B precomputation + v_max forward cascade as operator filter.
  - Operators in pool: add_random_node, replace_random_node.
    swap_two_nodes and swap_two_opt dropped (~0% SOCP-feasible).
  - Acceptance: hill-climbing only (tilted run OFF).
  - Perturbation: consecutive segment removal of k in {2, 3} nodes,
    triggered by stagnation >= t_improve, always accepted. The removed
    nodes are tabu'd for `tabu_tenure` iterations.
    `perturbation_mode = "segment"` (default) or `"random_k"` (revert
    to the original perturb_state's random non-consecutive removal,
    for comparison).
  - t_improve = 10 (lowered from 30 so perturbation actually fires).
  - Local-search style: random per-iteration (NOT classical
    first-improvement to exhaustion; that needs SOCP budget >= 5000).

Run on R101 (50) at SOCP_BUDGET = 1000.
"""
import os, sys, random, csv, argparse

_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
os.chdir(_repo_root)

from collections import Counter
import matplotlib.pyplot as plt

from uav_routing.environment import Environment
from uav_routing.environment.calibration import calibrate
from uav_routing.environment.graph import Graph
from uav_routing.environment.drone import Drone
from uav_routing.local_search.initial_solution import build_R3
from uav_routing.local_search.state import State
from uav_routing.local_search.optimization import Tally
from uav_routing.local_search.proposal import (
    perturb_state, route_to_nx,
)
from run_ils_fb_cascade_demo import (
    assign_slopes, make_instance, precompute_FB,
    add_fb_cascade, replace_fb_cascade,
)


R_CLASS_INSTANCES = [
    ("R101 (50)",     "datasets/data/50_r101.txt"),
    ("R101 (100)",    "datasets/data/r101.txt"),
    ("R1_2_1 (200)",  "datasets/homberger_200/r1_2_1.txt"),
]

C_CLASS_INSTANCES = [
    ("C101 (50)",     "datasets/data/50_c101.txt"),
    ("C101 (100)",    "datasets/data/c101.txt"),
    ("C1_2_1 (200)",  "datasets/homberger_200/c1_2_1.txt"),
]

ALL_INSTANCES = R_CLASS_INSTANCES + C_CLASS_INSTANCES

# Instance-expansion benchmark: spans window structure (co-monotone ->
# overlapping -> inverted, measured by the Kendall tau between window
# openings and closings) and instance size (48-288), on standard public
# OPTW/TOPTW benchmarks. Selected by structural covariates, not at random.
EXPANSION_INSTANCES = [
    ("R102 (100)",    "datasets/c_r_rc_100_100_Vansteen/r102.txt"),   # tau  0.21
    ("R104 (100)",    "datasets/c_r_rc_100_100_Vansteen/r104.txt"),   # tau -0.60
    ("C104 (100)",    "datasets/c_r_rc_100_100_Vansteen/c104.txt"),   # tau -0.73
    ("RC104 (100)",   "datasets/c_r_rc_100_100_Vansteen/rc104.txt"),  # tau -0.64
    ("RC1_2_1 (200)", "datasets/homberger_200/rc1_2_1.txt"),          # tau  1.00
    ("PR11 (48)",     "datasets/pr11_20/pr11.txt"),                   # tau  0.57
    ("PR15 (240)",    "datasets/pr11_20/pr15.txt"),                   # tau  0.59
    ("PR10 (288)",    "datasets/pr01_10_Vansteen/pr10.txt"),          # tau  0.86
]

GRAPH_SEED  = 1
ILS_SEED    = 42
SORTIE_TIME = 3.0

DEFAULT_ILS_STEPS = 1000
T_IMPROVE   = 10          # lowered from 30 -- decision 3
K_REMOVE    = 2           # used by both perturbation modes
TABU_TENURE = 10
SEG_LENGTHS = (2, 3)
# tilted run is OFF -- decision 2 (acceptance is hill-climbing)


def segment_remove(route, seg_lengths=SEG_LENGTHS):
    """Remove a consecutive segment of length k in seg_lengths.
    Returns (new_route, removed_nodes) or (None, None) if route too short.
    Used as the perturbation when perturbation_mode == 'segment'."""
    candidate_ks = [k for k in seg_lengths if len(route) >= 1 + k + 1]
    if not candidate_ks:
        return None, None
    k = random.choice(candidate_ks)
    max_start = len(route) - k
    start = random.randint(1, max_start)
    removed = route[start : start + k]
    new_route = route[:start] + route[start + k:]
    return new_route, removed


def local_move(state, graph, depot, v_max, T_max, F, B, tabu_set):
    current = state.solver.tour_nodes
    complement_set = (set(graph.nodes) - set(current)) - tabu_set
    l = len(current)
    # Only add and replace; swap operators dropped per decision.
    if l == 1:
        methods = ["add_random_node"]
    else:
        methods = ["add_random_node", "replace_random_node"]
    if not complement_set:
        # Nothing to add or replace with; signal no-op.
        return None, "no_op_full_tour"

    chosen_name = random.choice(methods)
    if chosen_name == "add_random_node":
        new_route = add_fb_cascade(current, complement_set, F, B, graph,
                                    depot, v_max, T_max)
    elif chosen_name == "replace_random_node":
        new_route = replace_fb_cascade(current, complement_set, F, B, graph,
                                        depot, v_max, T_max)
    else:
        raise RuntimeError(f"unknown: {chosen_name}")
    if new_route is None:
        return None, chosen_name
    new_state = state.flip(route_to_nx(new_route))
    new_state.last_operator = chosen_name
    return new_state, chosen_name


def apply_perturbation(state, perturbation_mode, k_remove, current_iter,
                      tabu_tenure):
    """Apply the configured perturbation. Returns (new_state, new_tabu_entries)
    or (None, {}) if not applicable."""
    if perturbation_mode == "segment":
        current_route = state.solver.tour_nodes
        new_route, removed = segment_remove(current_route)
        if new_route is None:
            return None, {}
        new_state = state.flip(route_to_nx(new_route))
        new_state.last_operator = "perturbation"
        new_state.is_perturbation = True
        new_tabu = {n: current_iter + tabu_tenure for n in removed}
        return new_state, new_tabu
    elif perturbation_mode == "random_k":
        # Original perturb_state from the package (non-consecutive random k).
        new_state, new_tabu = perturb_state(state, k_remove, current_iter,
                                            tabu_tenure)
        new_state.is_perturbation = True
        return new_state, new_tabu
    else:
        raise ValueError(f"unknown perturbation_mode: {perturbation_mode}")


def ils_run(state0, graph, depot, v_max, T_max, F, B, perturbation_mode,
            total_steps, t_improve, k_remove, tabu_tenure):
    best_state = state0
    best_score = state0.value
    current = state0

    tabu = {}
    stagnation = 0
    tally = Tally()
    saturated = Counter()

    f_best_trace = [(0, best_score)]
    f_curr_trace = [(0, current.value)]

    for i in range(1, total_steps + 1):
        # Purge expired tabu entries
        tabu = {n: e for n, e in tabu.items() if e > i}

        if stagnation >= t_improve:
            # Perturbation -- always accepted (subject to SOCP being feasible).
            perturbed, new_tabu = apply_perturbation(
                current, perturbation_mode, k_remove, i, tabu_tenure)
            if perturbed is None:
                # Route too short for segment removal; skip.
                tally.record_attempt("perturbation")
                saturated["perturbation"] += 1
            else:
                tabu.update(new_tabu)
                stagnation = 0
                tally.record_attempt("perturbation")
                if perturbed.solver is not None and perturbed.solver.solution is not None:
                    tally.record("perturbation", "accepted")
                    current = perturbed
                else:
                    tally.record("perturbation", "infeasible")
        else:
            proposed, op_name = local_move(current, graph, depot, v_max, T_max,
                                            F, B, set(tabu.keys()))
            if proposed is None:
                saturated[op_name] += 1
            else:
                tally.record_attempt(proposed.last_operator)
                if proposed.solver is None or proposed.solver.solution is None:
                    tally.record(proposed.last_operator, "infeasible")
                else:
                    prop_score = proposed.value
                    if prop_score >= current.value:
                        tally.record(proposed.last_operator, "accepted")
                        current = proposed
                        stagnation = 0
                        if prop_score > best_score:
                            best_state = proposed
                            best_score = prop_score
                    else:
                        # Hill-climbing only -- worse moves rejected.
                        tally.record(proposed.last_operator, "worse")
                        stagnation += 1
        f_best_trace.append((i, best_score))
        f_curr_trace.append((i, current.value if current.solver and current.solver.solution else None))
    return best_state, best_score, tally, saturated, f_best_trace, f_curr_trace


def run_one(instance_name, instance_path, perturbation_mode, ils_steps):
    instance, graph, drone = make_instance(instance_path)
    init_rng = random.Random(ILS_SEED)
    init_tour = build_R3(instance, rng=init_rng)
    depot = drone.base; v_max = drone.speed_max; T_max = instance.time_horizon
    F, B = precompute_FB(graph, depot, T_max)

    random.seed(ILS_SEED)
    state0 = State.initial_state(instance, init_tour)
    init_obj = state0.value
    init_n = sum(1 for n in init_tour.nodes if n != depot)
    print(f"\n{'#'*78}\n{instance_name}  final design  "
          f"perturbation_mode={perturbation_mode}  "
          f"t_improve={T_IMPROVE}  steps={ils_steps}\n{'#'*78}")
    print(f"[init] f(R_3) = {init_obj:.2f}  tour size: {init_n}")

    best_state, best_score, tally, saturated, f_best, f_curr = ils_run(
        state0, graph, depot, v_max, T_max, F, B, perturbation_mode,
        ils_steps, T_IMPROVE, K_REMOVE, TABU_TENURE)

    print(f"\n[final] best objective: {best_score:.2f}")
    best_size = (sum(1 for n in best_state.tour.nodes if n != depot)
                 if best_state else None)
    print(f"[final] best tour size: {best_size}")

    print(f"\nOPERATOR TALLY ({instance_name}, ILS_STEPS={ils_steps}, "
          f"perturbation={perturbation_mode})")
    print("-" * 110)
    rows = tally.summary()
    cols = ["operator", "attempts", "accepted", "infeasible", "worse",
            "saturated", "SOCP_feas", "SOCP_feas_%"]
    for r in rows:
        r["saturated"] = saturated.get(r["operator"], 0)
    expected_ops = ["add_random_node", "replace_random_node", "perturbation"]
    seen = {r["operator"] for r in rows}
    for op in expected_ops:
        if op not in seen:
            rows.append({"operator": op, "attempts": 0, "accepted": 0,
                         "infeasible": 0, "worse": 0, "saturated": 0})
    for op, cnt in saturated.items():
        if op not in seen and op not in expected_ops:
            rows.append({"operator": op, "attempts": 0, "accepted": 0,
                         "infeasible": 0, "worse": 0, "saturated": cnt})
    for r in rows:
        a = r["attempts"]
        r["SOCP_feas"] = a - r["infeasible"]
        r["SOCP_feas_%"] = f"{(r['SOCP_feas'] / a * 100):.1f}%" if a else "n/a"
    rows.sort(key=lambda r: r["operator"])
    widths = {c: max(len(c), max(len(str(r[c])) for r in rows)) for c in cols}
    print(" | ".join(c.ljust(widths[c]) for c in cols))
    print("-+-".join("-" * widths[c] for c in cols))
    for r in rows:
        cells = [str(r[c]) for c in cols]
        print(" | ".join(c.ljust(widths[cols[i]]) for i, c in enumerate(cells)))
    total_a = sum(r["attempts"]   for r in rows)
    total_ok = sum(r["accepted"]   for r in rows)
    total_inf = sum(r["infeasible"] for r in rows)
    total_w  = sum(r["worse"]      for r in rows)
    total_sat = sum(r["saturated"]  for r in rows)
    feas = total_a - total_inf
    feas_pct = (feas / total_a * 100) if total_a else 0
    print(f"[totals] attempts={total_a} accepted={total_ok} "
          f"infeasible={total_inf} worse_rej={total_w} saturated={total_sat}")
    print(f"  SOCP feasibility overall: {feas}/{total_a} = {feas_pct:.1f}%")

    safe = instance_name.lower().replace(" ", "_").replace("(", "").replace(")", "")
    out_csv = f"experiments/ils_final_{perturbation_mode}_{safe}_{ils_steps}.csv"
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({k: r[k] for k in cols})
    print(f"Wrote {out_csv}")

    fig, ax = plt.subplots(figsize=(10, 5))
    valid = [(i, v) for i, v in f_curr if v is not None]
    if valid:
        ax.plot([x[0] for x in valid], [x[1] for x in valid],
                color='tab:orange', linewidth=1.0, alpha=0.5, label='Current obj')
    ax.plot([x[0] for x in f_best], [x[1] for x in f_best],
            color='tab:blue', linewidth=1.8, label='Best obj', zorder=5)
    ax.set_xlabel("ILS iteration")
    ax.set_ylabel("Objective")
    ax.set_title(f"Final ILS on {instance_name}  ({ils_steps} iters, "
                 f"perturbation={perturbation_mode}, t_improve={T_IMPROVE})")
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)
    out_png = f"experiments/ils_final_{perturbation_mode}_{safe}_{ils_steps}.png"
    fig.tight_layout()
    fig.savefig(out_png, dpi=150, bbox_inches='tight')
    print(f"Wrote {out_png}")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--perturbation-mode", choices=["segment", "random_k"],
                        default="segment")
    parser.add_argument("--steps", type=int, default=DEFAULT_ILS_STEPS)
    parser.add_argument("--instance", default="all",
                        help="all (default, R-class), all-c (C-class), all-six, "
                             "or a specific instance name (e.g. 'R101 (50)', 'C101 (100)')")
    args = parser.parse_args()

    if args.instance == "all":
        targets = R_CLASS_INSTANCES
    elif args.instance == "all-c":
        targets = C_CLASS_INSTANCES
    elif args.instance == "all-six":
        targets = ALL_INSTANCES
    else:
        targets = [(name, path) for name, path in ALL_INSTANCES
                   if name == args.instance]
        if not targets:
            raise SystemExit(f"unknown instance: {args.instance}")

    for name, path in targets:
        run_one(name, path, args.perturbation_mode, args.steps)


if __name__ == "__main__":
    main()
