



# ── Individual pruning rules ──────────────────────────────────────────────────

def prune_nodes(graph, instance):
    """
    Rule: depot→node→depot round-trip exceeds time or energy budget.
    Returns a set of infeasible node IDs.
    """
    depot   = graph.graph['base']
    T_max   = instance.time_horizon
    E_max   = instance.max_energy
    drone   = instance.drone
    epsilon = drone.energy_per_meter(drone.optimum_speed)

    infeasible = set()
    for node in graph.nodes:
        if node == depot:
            continue
        d_total = graph[depot][node]['distance'] + graph[node][depot]['distance']
        if d_total / drone.speed_max > T_max or epsilon * d_total > E_max:
            infeasible.add(node)
    return infeasible


def prune_edges(graph, instance, feasible_nodes):
    """
    Rule: depot→i→j→depot round-trip exceeds time or energy budget.
    Only considers arcs between feasible nodes.
    Returns a set of infeasible (i, j) directed arc pairs.
    """
    depot   = graph.graph['base']
    T_max   = instance.time_horizon
    E_max   = instance.max_energy
    drone   = instance.drone
    epsilon = drone.energy_per_meter(drone.optimum_speed)

    infeasible = set()
    for u, v, data in graph.edges(data=True):
        for i, j in [(u, v), (v, u)]:   # undirected → both directions
            if i not in feasible_nodes or j not in feasible_nodes:
                infeasible.add((i, j))
                continue
            if i == depot or j == depot:
                continue
            d_total = graph[depot][i]['distance'] + data['distance'] + graph[j][depot]['distance']
            if d_total / drone.speed_max > T_max or epsilon * d_total > E_max:
                infeasible.add((i, j))
    return infeasible


def prune_by_time_windows(graph, instance, feasible_nodes):
    """
    Rule: earliest possible arrival at j via i exceeds j's time window close.
    Only considers arcs between feasible nodes.
    Returns a set of infeasible (i, j) directed arc pairs.
    """
    depot = graph.graph['base']
    drone = instance.drone

    infeasible = set()
    for u, v, data in graph.edges(data=True):
        for i, j in [(u, v), (v, u)]:
            if j == depot:
                continue
            if i not in feasible_nodes or j not in feasible_nodes:
                infeasible.add((i, j))
                continue
            d_depot_i   = graph[depot][i]['distance'] if i != depot else 0.0
            e_i         = graph.nodes[i]['time_window'][0]
            l_j         = graph.nodes[j]['time_window'][1]
            earliest_i  = max(e_i, d_depot_i / drone.speed_max)
            earliest_j  = earliest_i + data['distance'] / drone.speed_max
            if earliest_j > l_j:
                infeasible.add((i, j))
    return infeasible


# ── Connectivity clean-up ─────────────────────────────────────────────────────

def _remove_disconnected(feasible_arcs, base):
    """
    Iteratively remove non-depot nodes with no outgoing or no incoming arc
    (they can never be part of a valid tour).
    Returns the pruned arc set.
    """
    arcs = set(feasible_arcs)
    removed = True
    while removed:
        removed = False
        nodes_present = {i for i, _ in arcs} | {j for _, j in arcs}
        for n in list(nodes_present):
            if n == base:
                continue
            has_out = any(i == n for i, _ in arcs)
            has_in  = any(j == n for _, j in arcs)
            if not has_out or not has_in:
                arcs = {(i, j) for i, j in arcs if i != n and j != n}
                removed = True
    return arcs


# ── Main entry point ──────────────────────────────────────────────────────────

def prune(instance):
    """
    Apply all pruning rules and return feasible node / arc sets.

    Returns
    -------
    feasible_nodes : set
    feasible_arcs  : set of (i, j) directed pairs
    """
    graph = instance.graph
    base = graph.graph['base']
    N    = set(graph.nodes)

    # --- node pruning ---
    infeasible_nodes = prune_nodes(graph, instance)
    feasible_nodes   = (N - infeasible_nodes) | {base}

    # --- arc pruning ---
    infeasible_energy = prune_edges(graph, instance, feasible_nodes)
    infeasible_tw     = prune_by_time_windows(graph, instance, feasible_nodes)
    infeasible_arcs   = infeasible_energy | infeasible_tw

    all_arcs = {(i, j) for i in feasible_nodes for j in feasible_nodes if i != j}
    feasible_arcs = all_arcs - infeasible_arcs

    # --- connectivity clean-up ---
    feasible_arcs = _remove_disconnected(feasible_arcs, base)

    # nodes still reachable after connectivity clean-up
    reachable = {i for i, _ in feasible_arcs} | {j for _, j in feasible_arcs}
    feasible_nodes = feasible_nodes & reachable

    if base not in feasible_nodes:
        raise ValueError("Base node was pruned away — problem is infeasible.")

    _prune_report(N, infeasible_nodes, all_arcs, infeasible_energy, infeasible_tw, feasible_nodes, feasible_arcs)
    return feasible_nodes, feasible_arcs


# ── Report ────────────────────────────────────────────────────────────────────

def _prune_report(all_nodes, infeasible_nodes, all_arcs,
                  infeasible_energy, infeasible_tw, feasible_nodes, feasible_arcs):

    n_all   = len(all_nodes)
    n_feas  = len(feasible_nodes)
    a_all   = len(all_arcs)
    a_feas  = len(feasible_arcs)
    n_pruned = len(infeasible_nodes)
    a_energy = len(infeasible_energy)
    a_tw     = len(infeasible_tw)
    a_total  = len(all_arcs - feasible_arcs)

    W = 72

    print(f"\n{'═'*W}")
    print(f"  PRE-SOLVE PRUNING REPORT")
    print(f"{'═'*W}")

    # Node pruning
    print(f"  {'NODES':<30}  {'Before':>8}  {'Pruned':>8}  {'After':>8}  {'% kept':>8}")
    print(f"  {'-'*62}")
    print(f"  {'Round-trip budget':<30}  {n_all:>8}  {n_pruned:>8}  {n_feas:>8}  {n_feas/n_all*100:>7.1f}%")

    print(f"\n  {'ARCS':<30}  {'Before':>8}  {'Pruned':>8}  {'After':>8}  {'% kept':>8}")
    print(f"  {'-'*62}")
    print(f"  {'Energy budget':<30}  {a_all:>8}  {a_energy:>8}  {a_all-a_energy:>8}  {(a_all-a_energy)/a_all*100:>7.1f}%")
    print(f"  {'Time windows':<30}  {a_all:>8}  {a_tw:>8}  {a_all-a_tw:>8}  {(a_all-a_tw)/a_all*100:>7.1f}%")
    a_conn = a_total - min(a_energy + a_tw, a_total)
    print(f"  {'Connectivity clean-up':<30}  {'':>8}  {a_conn:>8}  {'':>8}  {'':>8}")
    print(f"  {'-'*62}")
    print(f"  {'TOTAL':<30}  {a_all:>8}  {a_total:>8}  {a_feas:>8}  {a_feas/a_all*100:>7.1f}%")

    # Variable count impact
    var_types = {'w': ('node', n_all), 'a': ('node', n_all),
                 'x': ('arc',  a_all), 't': ('arc',  a_all),
                 'L': ('arc',  a_all), 'y': ('arc',  a_all),
                 'z': ('arc',  a_all), 's': ('arc',  a_all)}
    total_before = sum(v for _, v in var_types.values())
    total_after  = sum(n_feas if kind == 'node' else a_feas for _, (kind, _) in var_types.items())

    print(f"\n  {'VARIABLE IMPACT':<30}  {'Before':>8}  {'After':>8}  {'Reduced':>8}  {'% kept':>8}")
    print(f"  {'-'*62}")
    for var, (kind, before) in var_types.items():
        after   = n_feas if kind == 'node' else a_feas
        reduced = before - after
        print(f"  {var:<30}  {before:>8}  {after:>8}  {reduced:>8}  {after/before*100:>7.1f}%")
    print(f"  {'-'*62}")
    print(f"  {'TOTAL':<30}  {total_before:>8}  {total_after:>8}  {total_before-total_after:>8}  {total_after/total_before*100:>7.1f}%")
    print(f"{'═'*W}\n")
