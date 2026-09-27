"""
initial_solution.py
====================
Initial tour construction heuristics (R1-R4) for the ILS framework.

All heuristics verify feasibility at each construction step by solving the
SOCP subproblem (paper Section 4.1) for the partial tour extended by the candidate
node and the return to the depot. A candidate is accepted only if the
resulting tour is SOCP-feasible.

References: paper Section 4.2
"""

import random
import gurobipy as gp

from uav_routing.environment.graph import directed_cycle
from uav_routing.solver.socp import Solver


def _screen(tour_nodes, graph, instance):
    """Necessary conditions for SOCP feasibility, without solving.

    The forward arrival cascade at v_max decides the time windows exactly, and
    every leg costs at least its straight-line length times the minimum energy
    per meter. A tour failing either would come back from the subproblem as
    infeasible, so screening it out first changes nothing and skips a solve.
    """
    drone = instance.drone
    v_max = drone.speed_max
    depot = drone.base
    a = 0.0
    total_d = 0.0
    prev = tour_nodes[0]
    for node in list(tour_nodes[1:]) + [depot]:
        d = graph[prev][node]["distance"]
        total_d += d
        a = a + d / v_max
        if node != depot:
            e, l = graph.nodes[node]["time_window"]
            if a < e:
                a = e
            if a > l:
                return False
        prev = node
    if a > instance.time_horizon:
        return False
    v_mr = (drone.c_2 / drone.c_1) ** 0.25
    e_per_m = (drone.c_0 + drone.c_1 * v_mr ** 3 + drone.c_2 / v_mr) / v_mr
    return total_d * e_per_m <= instance.max_energy


def _check_feasible(tour_nodes, graph, instance, gurobi_env):
    """Solve SOCP for a candidate tour. Returns True if feasible."""
    if len(tour_nodes) < 2:
        return False
    if not _screen(tour_nodes, graph, instance):
        return False
    tour = directed_cycle(tour_nodes, graph)
    # the construction evaluates under the same subproblem as the search that follows
    # (a run with --no-loiter pins L_ij = d_ij here too, or its start could be infeasible)
    solver = Solver(tour, instance, no_loiter=getattr(instance, "no_loiter", False), _gurobi_env=gurobi_env)
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
    tour R+[v] (closed cycle returning to the depot) and select the node
    v* that maximizes information collected per unit of binding resource:

        capacity(R+[v]) = max( T(R+[v]) / T_max ,  E(R+[v]) / E_max )
        score(v)        = f(R+[v]) / capacity(R+[v])

    where f and E are the SOCP objective and total energy, T is the total
    closed-tour duration, and T_max, E_max are the mission horizon and
    energy budget. The denominator is the larger of the two normalized
    resource utilizations, so the ratio rewards candidates that gain a
    lot of information without exhausting either time or energy.
    """
    graph = instance.graph
    depot = instance.drone.base
    E_max = instance.max_energy
    T_max = instance.time_horizon

    if n_target is None:
        n_target = len(graph.nodes) - 1

    env = gp.Env(params={"OutputFlag": 0})
    tour = [depot]
    visited = {depot}

    for _ in range(n_target):
        candidates = [n for n in graph.nodes if n not in visited]
        if not candidates:
            break

        score = {}
        for v in candidates:
            if not _screen(tour + [v], graph, instance):
                continue          # the subproblem would return infeasible
            candidate_tour = directed_cycle(tour + [v], graph)
            solver = Solver(candidate_tour, instance, no_loiter=getattr(instance, "no_loiter", False),
                            _gurobi_env=env)
            if solver.solution is None:
                continue
            td = solver.get_tour_data()

            total_time = sum(td.times.values())
            time_ratio   = total_time / T_max
            energy_ratio = td.total_energy / E_max
            capacity = max(time_ratio, energy_ratio)
            if capacity <= 0:
                continue
            score[v] = td.objective / capacity

        if not score:
            break

        best_node = max(score, key=score.get)
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
