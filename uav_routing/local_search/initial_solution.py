"""
initial_solution.py
====================
Initial tour construction heuristics (R1-R4) for the ILS framework.

All heuristics verify feasibility at each construction step by solving the
SOCP subproblem (Section 5.1) for the partial tour extended by the candidate
node and the return to the depot. A candidate is accepted only if the
resulting tour is SOCP-feasible.

References: Paper Section 5.2
"""

import random
import gurobipy as gp

from uav_routing.environment.graph import directed_cycle
from uav_routing.solver.socp import Solver


def _check_feasible(tour_nodes, graph, instance, gurobi_env):
    """Solve SOCP for a candidate tour. Returns True if feasible."""
    if len(tour_nodes) < 2:
        return False
    tour = directed_cycle(tour_nodes, graph)
    solver = Solver(tour, instance, _gurobi_env=gurobi_env)
    return solver.solution is not None


def build_R1(instance, n_target=4):
    """R1: Earliest-opening tour.

    Sort unvisited nodes by time-window opening e_i and select the first
    feasible one. Fixed tour length.
    """
    graph = instance.graph
    depot = graph.graph['base']

    env = gp.Env(params={"OutputFlag": 0})
    tour = [depot]
    visited = {depot}

    for _ in range(n_target):
        candidates = sorted(
            [n for n in graph.nodes if n not in visited],
            key=lambda n: graph.nodes[n]['time_window'][0]
        )
        added = False
        for node in candidates:
            if _check_feasible(tour + [node], graph, instance, env):
                tour.append(node)
                visited.add(node)
                added = True
                break
        if not added:
            break

    env.close()
    return directed_cycle(tour, graph)


def build_R2(instance, n_target=4, rng=None):
    """R2: Random-then-earliest tour.

    Select the first node uniformly at random among feasible nodes.
    Then sort remaining unvisited nodes by time-window opening e_i
    and select the first feasible one at each step. Fixed tour length.
    """
    if rng is None:
        rng = random.Random(42)

    graph = instance.graph
    depot = graph.graph['base']

    env = gp.Env(params={"OutputFlag": 0})
    tour = [depot]
    visited = {depot}

    # First node: random among feasible
    targets = [n for n in graph.nodes if n != depot]
    rng.shuffle(targets)
    first_added = False
    for node in targets:
        if _check_feasible(tour + [node], graph, instance, env):
            tour.append(node)
            visited.add(node)
            first_added = True
            break
    if not first_added:
        env.close()
        return directed_cycle(tour, graph)

    # Remaining: earliest-opening with feasibility
    for _ in range(n_target - 1):
        candidates = sorted(
            [n for n in graph.nodes if n not in visited],
            key=lambda n: graph.nodes[n]['time_window'][0]
        )
        added = False
        for node in candidates:
            if _check_feasible(tour + [node], graph, instance, env):
                tour.append(node)
                visited.add(node)
                added = True
                break
        if not added:
            break

    env.close()
    return directed_cycle(tour, graph)


def build_R3(instance, rng=None, length=4):
    """R3: Random tour.

    Select nodes uniformly at random from N \\ {0}, accepting each only
    if the partial tour remains feasible. Fixed tour length.
    """
    if rng is None:
        rng = random.Random(42)

    graph = instance.graph
    depot = graph.graph['base']

    env = gp.Env(params={"OutputFlag": 0})
    tour = [depot]
    visited = {depot}
    targets = [n for n in graph.nodes if n != depot]
    rng.shuffle(targets)

    count = 0
    for node in targets:
        if count >= length:
            break
        if _check_feasible(tour + [node], graph, instance, env):
            tour.append(node)
            visited.add(node)
            count += 1

    env.close()
    return directed_cycle(tour, graph)


def build_R4(instance, n_target=None):
    """R4: Information-efficiency tour.

    Greedy construction: at each step, solve the SOCP for every candidate
    tour T+[v] and select the node v* that minimizes:

        value(v) = max(1 - info_v / max_info, energy_v / max_energy)

    where info_v and energy_v come from the SOCP solution, max_info is the
    maximum collectible information at closing time across all nodes, and
    max_energy is the energy budget. Low value means high info and low
    energy — a well-balanced node.
    """
    graph = instance.graph
    depot = instance.drone.base
    E_max = instance.max_energy

    if n_target is None:
        n_target = len(graph.nodes) - 1

    # Precompute max_info: maximum info at closing time across all nodes
    max_info = 0.0
    for n in graph.nodes:
        if n == depot:
            continue
        e_n, l_n = graph.nodes[n]['time_window']
        info_n = (graph.nodes[n]['info_at_lowest']
                  + graph.nodes[n]['info_slope'] * (l_n - e_n))
        if info_n > max_info:
            max_info = info_n

    env = gp.Env(params={"OutputFlag": 0})
    tour = [depot]
    visited = {depot}

    for _ in range(n_target):
        candidates = [n for n in graph.nodes if n not in visited]
        if not candidates:
            break

        value = {}
        for v in candidates:
            candidate_tour = directed_cycle(tour + [v], graph)
            solver = Solver(candidate_tour, instance, _gurobi_env=env)
            if solver.solution is None:
                continue
            td = solver.get_tour_data()

            info_ratio = 1.0 - td.objective / max_info if max_info > 0 else 1.0
            energy_ratio = td.total_energy / E_max
            value[v] = max(info_ratio, energy_ratio)

        if not value:
            break

        best_node = min(value, key=value.get)

        if best_node is None:
            break

        tour.append(best_node)
        visited.add(best_node)

    env.close()
    return directed_cycle(tour, graph)


INITIAL_TOURS = {
    "R1 (earliest)": build_R1,
    "R2 (rand+earliest)": build_R2,
    "R3 (random)":   build_R3,
    "R4 (info-eff)": build_R4,
}
