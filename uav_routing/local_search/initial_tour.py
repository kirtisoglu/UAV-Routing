
import random
import gurobipy as gp
import networkx as nx

from uav_routing.solver.socp import Solver, TourResult
from uav_routing.environment.graph import directed_cycle


def check_feasibility(instance, tour_nodes, gurobi_env=None):
    """Solve SOCP for a candidate tour (list of nodes starting with depot).
    Returns TourResult (feasible or not)."""
    tour_graph = directed_cycle(tour_nodes, instance.graph)
    solver = Solver(tour_graph, instance, _gurobi_env=gurobi_env)
    return solver.get_tour_data()


def feasible_extensions(instance, tour_nodes, candidates, gurobi_env=None):
    """For each candidate node, check if appending it to the tour and
    returning to depot produces a feasible SOCP solution.

    Returns list of (node, TourResult) for feasible candidates only.
    """
    depot = instance.drone.base
    feasible = []
    for j in candidates:
        test_tour = tour_nodes + [j]
        result = check_feasibility(instance, test_tour, gurobi_env)
        if result.feasible:
            feasible.append((j, result))
    return feasible


# ======================== R1: Nearest-node tour ========================

def r1_nearest_node(instance, tour_length=None, gurobi_env=None):
    """Starting from depot, iteratively select the nearest unvisited node
    among those that keep the tour SOCP-feasible.
    Fixed length: floor(n/2) nodes."""
    graph = instance.graph
    depot = instance.drone.base
    if tour_length is None:
        tour_length = (len(graph.nodes) - 1) // 2

    tour_nodes = [depot]
    unvisited = set(n for n in graph.nodes if n != depot)

    for _ in range(tour_length):
        if not unvisited:
            break
        # Sort candidates by distance from current node
        current = tour_nodes[-1]
        sorted_candidates = sorted(unvisited,
                                   key=lambda n: graph.edges[(current, n)]['distance'])

        # Check feasibility in nearest-first order, pick first feasible
        added = False
        for j in sorted_candidates:
            test_tour = tour_nodes + [j]
            result = check_feasibility(instance, test_tour, gurobi_env)
            if result.feasible:
                tour_nodes.append(j)
                unvisited.discard(j)
                added = True
                break
        if not added:
            break

    return _finalize(instance, tour_nodes, gurobi_env)


# ======================== R2: Earliest-opening tour ========================

def r2_earliest_opening(instance, tour_length=None, gurobi_env=None):
    """Starting from depot, iteratively select the unvisited node with
    the earliest time-window opening among feasible extensions.
    Fixed length: floor(n/2) nodes."""
    graph = instance.graph
    depot = instance.drone.base
    if tour_length is None:
        tour_length = (len(graph.nodes) - 1) // 2

    tour_nodes = [depot]
    unvisited = set(n for n in graph.nodes if n != depot)

    for _ in range(tour_length):
        if not unvisited:
            break
        sorted_candidates = sorted(unvisited,
                                   key=lambda n: graph.nodes[n]['time_window'][0])

        added = False
        for j in sorted_candidates:
            test_tour = tour_nodes + [j]
            result = check_feasibility(instance, test_tour, gurobi_env)
            if result.feasible:
                tour_nodes.append(j)
                unvisited.discard(j)
                added = True
                break
        if not added:
            break

    return _finalize(instance, tour_nodes, gurobi_env)


# ======================== R3: Random-then-nearest tour ========================

def r3_random_then_nearest(instance, tour_length=None, gurobi_env=None):
    """Select first node uniformly at random (among feasible), then
    iteratively select nearest feasible unvisited node.
    Fixed length: floor(n/2) nodes."""
    graph = instance.graph
    depot = instance.drone.base
    if tour_length is None:
        tour_length = (len(graph.nodes) - 1) // 2

    tour_nodes = [depot]
    unvisited = set(n for n in graph.nodes if n != depot)

    # First node: random among feasible
    candidates = list(unvisited)
    random.shuffle(candidates)
    first_added = False
    for j in candidates:
        test_tour = tour_nodes + [j]
        result = check_feasibility(instance, test_tour, gurobi_env)
        if result.feasible:
            tour_nodes.append(j)
            unvisited.discard(j)
            first_added = True
            break

    if not first_added:
        return _finalize(instance, tour_nodes, gurobi_env)

    # Remaining nodes: nearest feasible
    for _ in range(tour_length - 1):
        if not unvisited:
            break
        current = tour_nodes[-1]
        sorted_candidates = sorted(unvisited,
                                   key=lambda n: graph.edges[(current, n)]['distance'])
        added = False
        for j in sorted_candidates:
            test_tour = tour_nodes + [j]
            result = check_feasibility(instance, test_tour, gurobi_env)
            if result.feasible:
                tour_nodes.append(j)
                unvisited.discard(j)
                added = True
                break
        if not added:
            break

    return _finalize(instance, tour_nodes, gurobi_env)


# ======================== R4: Consumption-based tour ========================

def r4_consumption_based(instance, gurobi_env=None):
    """Build a feasible route as long as possible. At each step, add the
    unvisited node that increases cumulative capacity usage the least:
        j* = argmin max(t_ij / T_max, E(v_opt, d_ij) / E_max)
    Extends until no feasible extension exists."""
    graph = instance.graph
    drone = instance.drone
    depot = drone.base
    v_opt = drone.optimum_speed
    T_max = instance.time_horizon
    E_max = instance.max_energy

    tour_nodes = [depot]
    unvisited = set(n for n in graph.nodes if n != depot)

    while unvisited:
        current = tour_nodes[-1]

        # Score all unvisited nodes by capacity cost
        scored = []
        for j in unvisited:
            d_ij = graph.edges[(current, j)]['distance']
            t_ij = d_ij / v_opt
            capacity_cost = max(t_ij / T_max, drone.energy_function(v_opt, d_ij) / E_max)
            scored.append((capacity_cost, j))
        scored.sort()

        # Try in order of lowest capacity cost, pick first feasible
        added = False
        for _, j in scored:
            test_tour = tour_nodes + [j]
            result = check_feasibility(instance, test_tour, gurobi_env)
            if result.feasible:
                tour_nodes.append(j)
                unvisited.discard(j)
                added = True
                break
        if not added:
            break

    return _finalize(instance, tour_nodes, gurobi_env)


# ======================== R5: Random tour ========================

def r5_random(instance, tour_length=3, gurobi_env=None):
    """Select tour_length nodes uniformly at random, checking feasibility
    after each addition. Short seed tour to introduce diversity."""
    graph = instance.graph
    depot = instance.drone.base

    tour_nodes = [depot]
    unvisited = list(n for n in graph.nodes if n != depot)
    random.shuffle(unvisited)

    for j in unvisited:
        if len(tour_nodes) - 1 >= tour_length:
            break
        test_tour = tour_nodes + [j]
        result = check_feasibility(instance, test_tour, gurobi_env)
        if result.feasible:
            tour_nodes.append(j)

    return _finalize(instance, tour_nodes, gurobi_env)


# ======================== R6: Information-efficiency tour ========================

def r6_information_efficiency(instance, tour_length=None, gurobi_env=None):
    """Iteratively select the node maximizing marginal info per unit detour energy:
        j* = argmax (I_e_j + gamma_j * (l_j - e_j)) / delta_E_j
    where delta_E_j = E(v_opt, d_i,j) + E(v_opt, d_j,0) - E(v_opt, d_i,0)
    Fixed length: floor(n/2) nodes."""
    graph = instance.graph
    drone = instance.drone
    depot = drone.base
    v_opt = drone.optimum_speed
    if tour_length is None:
        tour_length = (len(graph.nodes) - 1) // 2

    tour_nodes = [depot]
    unvisited = set(n for n in graph.nodes if n != depot)

    for _ in range(tour_length):
        if not unvisited:
            break
        current = tour_nodes[-1]

        # Score all unvisited nodes by info-efficiency ratio
        scored = []
        for j in unvisited:
            info_j = graph.nodes[j]['info_at_lowest']
            slope_j = graph.nodes[j]['info_slope']
            tw = graph.nodes[j]['time_window']
            max_info = info_j + slope_j * (tw[1] - tw[0])

            d_ij = graph.edges[(current, j)]['distance']
            d_j0 = graph.edges[(j, depot)]['distance']
            d_i0 = graph.edges[(current, depot)]['distance']
            delta_E = (drone.energy_function(v_opt, d_ij)
                       + drone.energy_function(v_opt, d_j0)
                       - drone.energy_function(v_opt, d_i0))

            if delta_E <= 0:
                ratio = float('inf')
            else:
                ratio = max_info / delta_E
            scored.append((ratio, j))

        # Try in order of highest ratio, pick first feasible
        scored.sort(reverse=True)
        added = False
        for _, j in scored:
            test_tour = tour_nodes + [j]
            result = check_feasibility(instance, test_tour, gurobi_env)
            if result.feasible:
                tour_nodes.append(j)
                unvisited.discard(j)
                added = True
                break
        if not added:
            break

    return _finalize(instance, tour_nodes, gurobi_env)


# ======================== Helpers ========================

def _finalize(instance, tour_nodes, gurobi_env=None):
    """Solve final SOCP for the completed tour and return TourResult."""
    if len(tour_nodes) <= 1:
        return TourResult(sequence=tour_nodes, feasible=False)
    return check_feasibility(instance, tour_nodes, gurobi_env)


def build_all_initial_tours(instance, gurobi_env=None, verbose=True):
    """Run all 6 heuristics and return dict of {name: TourResult}."""
    env = gurobi_env
    if env is None:
        env = gp.Env(params={"OutputFlag": 0})

    heuristics = {
        "R1 (nearest-node)": r1_nearest_node,
        "R2 (earliest-opening)": r2_earliest_opening,
        "R3 (random-then-nearest)": r3_random_then_nearest,
        "R4 (consumption-based)": r4_consumption_based,
        "R5 (random)": r5_random,
        "R6 (info-efficiency)": r6_information_efficiency,
    }

    results = {}
    for name, func in heuristics.items():
        if verbose:
            print(f"  Building {name}...", end=" ")
        result = func(instance, gurobi_env=env)
        results[name] = result
        if verbose:
            if result.feasible:
                n_nodes = len(result.sequence) - 1  # exclude depot
                print(f"feasible, {n_nodes} nodes, obj={result.objective:.2f}")
            else:
                print("infeasible")

    return results
