"""
F/B edge-first sampling with v_max cascade as the inner correctness gate.

Pre-filter:  candidates_for_edge_ij = F[i] & B[j] & complement
Inner gate:  v_max cascade check (TW push-forward through u and downstream
             + horizon)

For each add-step:
  for edge (i, j) in random shuffle of route_edges:
      cands = F[i] & B[j] & complement
      if cands empty:  this edge contributes nothing; continue
      for u in random shuffle of cands:
          if cascade_feasible(route, position_for_edge_ij, u, ...):
              insert u between i and j; return new state
      # if no u in cands passed cascade, this edge has no feasible insertion
  return SATURATED -- no edge admits any insertion that passes both filters

Saturated outcomes are tracked SEPARATELY from infeasible -- they indicate
the chain has no admissible move under the operator, not that a tried move
failed at the SOCP. We do NOT call the SOCP on a saturated step (no
duplicate solve on the unchanged route) and we do NOT increment the
stagnation counter on saturation (the user's principle: stagnation is
about local optima, not about "the operator couldn't move").

Reports the same Tally + objective trajectory plot, plus a new
saturated-counter row per operator.
"""
import os, sys, random, csv

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
    swap_two_nodes, swap_two_opt,
)


INSTANCES = [
    ("R101 (50)",  "datasets/data/50_r101.txt"),
    ("R101 (100)", "datasets/data/r101.txt"),
]

GRAPH_SEED  = 1
ILS_SEED    = 42
SORTIE_TIME = 3.0

ILS_STEPS   = 1000
T_IMPROVE   = 30
K_REMOVE    = 2
TABU_TENURE = 10


def assign_slopes(graph, seed=1, lo=0.5, hi=1.0):
    rng = random.Random(seed)
    for node_id, data in graph.nodes(data=True):
        delta_t = data['time_window'][1] - data['time_window'][0]
        if delta_t == 0:
            graph.nodes[node_id]['info_slope'] = 0.0
            continue
        bound = data['info_at_lowest'] / delta_t
        sign = rng.choice([-1.0, 1.0])
        magnitude = rng.uniform(lo * bound, hi * bound)
        graph.nodes[node_id]['info_slope'] = sign * magnitude


def make_instance(path, eta=1.0):
    g = Graph(path=path, slope='zero', seed=GRAPH_SEED)
    assign_slopes(g, seed=GRAPH_SEED)
    drone = Drone(base=g.graph['base'])
    calib = calibrate(graph=g, tour_length=len(g.nodes)//2, drone=drone,
                      drone_sortie_time=SORTIE_TIME,
                      calibration_method="campaign")
    inst = Environment(calib, drone, eta=eta)     # eta scales the energy budget only
    # Return the calibrated graph (distances in meters, time windows in
    # seconds), the same graph the SOCP uses. The raw benchmark graph must
    # not be used for the feasibility checks, since v_max, T_max and E_max
    # are physical quantities.
    return inst, inst.graph, drone


def precompute_FB(graph, depot, T_max):
    def e_of(n): return 0.0 if n == depot else graph.nodes[n]['time_window'][0]
    def l_of(n): return T_max if n == depot else graph.nodes[n]['time_window'][1]
    nodes = list(graph.nodes)
    F = {i: {u for u in nodes if u != i and l_of(u) > e_of(i)} for i in nodes}
    B = {j: {u for u in nodes if u != j and e_of(u) < l_of(j)} for j in nodes}
    return F, B


def compute_epsilon(route, graph, depot, v_max):
    eps = {depot: 0.0}
    prev = 0.0
    for q in range(1, len(route)):
        node = route[q]
        d = graph[route[q-1]][node]['distance']
        e_q = graph.nodes[node]['time_window'][0] if node != depot else 0.0
        prev = max(e_q, prev + d / v_max)
        eps[node] = prev
    return eps


def cascade_feasible_add(route, p, u, graph, depot, v_max, T_max, eps):
    i = route[p-1]
    j = route[p] if p < len(route) else depot
    e_u, l_u = graph.nodes[u]['time_window']
    eps_i = eps[i]
    eps_u = max(e_u, eps_i + graph[i][u]['distance'] / v_max)
    if eps_u > l_u: return False
    if j != depot:
        e_j = graph.nodes[j]['time_window'][0]
        l_j = graph.nodes[j]['time_window'][1]
        prev = max(e_j, eps_u + graph[u][j]['distance'] / v_max)
        if prev > l_j: return False
        for q in range(p + 1, len(route)):
            node_q = route[q]
            prev = max(graph.nodes[node_q]['time_window'][0],
                       prev + graph[route[q-1]][node_q]['distance'] / v_max)
            if prev > graph.nodes[node_q]['time_window'][1]:
                return False
        last = route[-1]
    else:
        prev = eps_u
        last = u
    if prev + graph[last][depot]['distance'] / v_max > T_max:
        return False
    return True


def cascade_feasible_replace(route, p, u, graph, depot, v_max, T_max, eps):
    i = route[p-1]
    j = route[p+1] if p+1 < len(route) else depot
    e_u, l_u = graph.nodes[u]['time_window']
    eps_i = eps[i]
    eps_u = max(e_u, eps_i + graph[i][u]['distance'] / v_max)
    if eps_u > l_u: return False
    if j != depot:
        e_j = graph.nodes[j]['time_window'][0]
        l_j = graph.nodes[j]['time_window'][1]
        prev = max(e_j, eps_u + graph[u][j]['distance'] / v_max)
        if prev > l_j: return False
        for q in range(p + 2, len(route)):
            node_q = route[q]
            prev = max(graph.nodes[node_q]['time_window'][0],
                       prev + graph[route[q-1]][node_q]['distance'] / v_max)
            if prev > graph.nodes[node_q]['time_window'][1]:
                return False
        last = route[-1] if route[-1] != route[p] else u
    else:
        prev = eps_u
        last = u
    if prev + graph[last][depot]['distance'] / v_max > T_max:
        return False
    return True


def add_fb_cascade(route, complement_set, F, B, graph, depot, v_max, T_max):
    eps = compute_epsilon(route, graph, depot, v_max)
    edges = list(enumerate(zip(route, route[1:] + [depot])))
    random.shuffle(edges)
    for p_idx, (i, j) in edges:
        # Position for inserting between route[p_idx] = i and route[p_idx+1] = j
        # is p_idx + 1 (i.e., the new node sits at that index in the new route).
        p = p_idx + 1
        cands = list((F[i] & B[j]) & complement_set)
        if not cands: continue
        random.shuffle(cands)
        for u in cands:
            if cascade_feasible_add(route, p, u, graph, depot, v_max, T_max, eps):
                new_route = list(route)
                new_route.insert(p, u)
                return new_route
    return None  # saturated


def replace_fb_cascade(route, complement_set, F, B, graph, depot, v_max, T_max):
    if len(route) < 2: return None
    eps = compute_epsilon(route, graph, depot, v_max)
    positions = list(range(1, len(route)))
    random.shuffle(positions)
    for p in positions:
        i = route[p-1]
        j = route[p+1] if p+1 < len(route) else depot
        cands = list((F[i] & B[j]) & complement_set)
        if not cands: continue
        random.shuffle(cands)
        for u in cands:
            if cascade_feasible_replace(route, p, u, graph, depot, v_max, T_max, eps):
                new_route = list(route)
                new_route[p] = u
                return new_route
    return None  # saturated


def local_move_fb_cascade(state, graph, depot, v_max, T_max, F, B, tabu_set):
    """Returns (new_state_or_None, chosen_name). new_state is None when
    the operator is saturated under the F/B + cascade filter."""
    current = state.solver.tour_nodes
    complement_set = (set(graph.nodes) - set(current)) - tabu_set
    l = len(current)
    if l == 1:
        methods = ["add_random_node"]
    elif l == 2:
        methods = ["add_random_node", "replace_random_node"]
    elif l == 3:
        methods = ["add_random_node", "replace_random_node", "swap_two_nodes"]
    else:
        methods = ["add_random_node", "replace_random_node",
                   "swap_two_nodes", "swap_two_opt"]
    if not complement_set:
        methods = ["swap_two_nodes", "swap_two_opt"]

    chosen_name = random.choice(methods)
    if chosen_name == "add_random_node":
        new_route = add_fb_cascade(current, complement_set, F, B, graph,
                                    depot, v_max, T_max)
        if new_route is None:
            return None, chosen_name
    elif chosen_name == "replace_random_node":
        new_route = replace_fb_cascade(current, complement_set, F, B, graph,
                                        depot, v_max, T_max)
        if new_route is None:
            return None, chosen_name
    elif chosen_name == "swap_two_nodes":
        new_route = swap_two_nodes(current, list(complement_set))
    elif chosen_name == "swap_two_opt":
        new_route = swap_two_opt(current, list(complement_set))
    else:
        raise RuntimeError(f"unknown: {chosen_name}")
    new_state = state.flip(route_to_nx(new_route))
    new_state.last_operator = chosen_name
    return new_state, chosen_name


def ils_run(state0, graph, depot, v_max, T_max, F, B, total_steps,
            t_improve, k_remove, tabu_tenure):
    best_state = state0
    best_score = state0.value
    current = state0

    tabu = {}
    stagnation = 0
    tally = Tally()
    saturated = Counter()   # op_name -> count of saturation events

    f_best_trace = [(0, best_score)]
    f_curr_trace = [(0, current.value)]

    for i in range(1, total_steps + 1):
        tabu = {n: e for n, e in tabu.items() if e > i}

        if stagnation >= t_improve:
            perturbed, new_tabu = perturb_state(current, k_remove, i, tabu_tenure)
            tabu.update(new_tabu)
            stagnation = 0
            perturbed.is_perturbation = True
            tally.record_attempt(perturbed.last_operator)
            if perturbed.solver is not None and perturbed.solver.solution is not None:
                tally.record(perturbed.last_operator, "accepted")
                current = perturbed
            else:
                tally.record(perturbed.last_operator, "infeasible")
        else:
            proposed, op_name = local_move_fb_cascade(
                current, graph, depot, v_max, T_max, F, B, set(tabu.keys()))
            if proposed is None:
                # Saturated: don't record attempt in main tally,
                # don't call SOCP, don't update stagnation.
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
                        tally.record(proposed.last_operator, "worse")
                        stagnation += 1

        f_best_trace.append((i, best_score))
        f_curr_trace.append((i, current.value if current.solver and current.solver.solution else None))

    return best_state, best_score, tally, saturated, f_best_trace, f_curr_trace


def run_one(instance_name, instance_path):
    instance, graph, drone = make_instance(instance_path)
    init_rng = random.Random(ILS_SEED)
    init_tour = build_R3(instance, rng=init_rng)
    depot = drone.base
    v_max = drone.speed_max
    T_max = instance.time_horizon
    F, B = precompute_FB(graph, depot, T_max)

    random.seed(ILS_SEED)
    state0 = State.initial_state(instance, init_tour)
    init_obj = state0.value
    init_n = sum(1 for n in init_tour.nodes if n != depot)
    print(f"\n{'#'*78}\n{instance_name}  v_max={v_max:.2f}  T_max={T_max:.0f}s\n{'#'*78}")
    print(f"[init] f(R_3) initial objective: {init_obj:.2f}  tour size: {init_n}")

    best_state, best_score, tally, saturated, f_best, f_curr = ils_run(
        state0, graph, depot, v_max, T_max, F, B,
        ILS_STEPS, T_IMPROVE, K_REMOVE, TABU_TENURE)

    print(f"\n[final] best objective after {ILS_STEPS} iters: {best_score:.2f}")
    best_size = (sum(1 for n in best_state.tour.nodes if n != depot)
                 if best_state else None)
    print(f"[final] best tour size: {best_size}")

    print(f"\nOPERATOR TALLY ({instance_name}, {ILS_STEPS} iters, F/B + cascade)")
    print("-" * 92)
    rows = tally.summary()
    # Annotate each operator with its saturation count.
    cols = ["operator", "attempts", "accepted", "infeasible", "worse",
            "saturated", "reject_rate"]
    for r in rows:
        r["saturated"] = saturated.get(r["operator"], 0)
    # Also include operators that ONLY appeared as saturated (no attempts).
    seen_ops = {r["operator"] for r in rows}
    for op, cnt in saturated.items():
        if op not in seen_ops:
            rows.append({"operator": op, "attempts": 0, "accepted": 0,
                         "infeasible": 0, "worse": 0, "saturated": cnt,
                         "reject_rate": 0.0})
    rows.sort(key=lambda r: r["operator"])
    widths = {c: max(len(c), max(len(str(r[c]) if c != "reject_rate"
                                     else f"{r[c]:.2%}") for r in rows))
              for c in cols}
    print(" | ".join(c.ljust(widths[c]) for c in cols))
    print("-+-".join("-" * widths[c] for c in cols))
    for r in rows:
        cells = [str(r["operator"]), str(r["attempts"]), str(r["accepted"]),
                 str(r["infeasible"]), str(r["worse"]), str(r["saturated"]),
                 f"{r['reject_rate']:.2%}"]
        print(" | ".join(c.ljust(widths[cols[i]]) for i, c in enumerate(cells)))
    total_a = sum(r["attempts"]   for r in rows)
    total_ok = sum(r["accepted"]   for r in rows)
    total_inf = sum(r["infeasible"] for r in rows)
    total_w  = sum(r["worse"]      for r in rows)
    total_sat = sum(r["saturated"]  for r in rows)
    print(f"[totals] attempts={total_a} accepted={total_ok} "
          f"infeasible={total_inf} worse={total_w} saturated={total_sat}")
    print(f"  (attempts + saturated = {total_a + total_sat} of {ILS_STEPS} ILS iters; "
          f"remainder are perturbation calls)")

    safe = instance_name.lower().replace(" ", "_").replace("(", "").replace(")", "")
    out_csv = f"experiments/ils_fb_cascade_{safe}.csv"
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
                color='tab:orange', linewidth=1.0, alpha=0.6, label='Current obj')
    ax.plot([x[0] for x in f_best], [x[1] for x in f_best],
            color='tab:blue', linewidth=1.8, label='Best obj', zorder=5)
    ax.set_xlabel("ILS iteration")
    ax.set_ylabel("Objective")
    ax.set_title(f"ILS w/ F_i, B_j + cascade on {instance_name}  ({ILS_STEPS} iters)")
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)
    out_png = f"experiments/ils_fb_cascade_{safe}.png"
    fig.tight_layout()
    fig.savefig(out_png, dpi=150, bbox_inches='tight')
    print(f"Wrote {out_png}")
    plt.close(fig)


def main():
    for name, path in INSTANCES:
        run_one(name, path)


if __name__ == "__main__":
    main()
