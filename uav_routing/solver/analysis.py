"""
analysis.py
===========
Computational experiments for the UAV routing MISOCP model.

Experiments:
    1. alpha_sensitivity   – How objective changes as energy budget (alpha) increases
    3. arrival_time_histogram – Where within time windows does the drone arrive
    4. speed_histogram     – Distribution of arc speeds for a solved instance
    5. scalability_table   – Solve time vs instance size
    6. value_of_loitering  – Objective gap between full model and no-loiter model
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import gurobipy as gp
from gurobipy import GRB

from uav_routing.solver.exact import solve_model_gurobi, construct_tour


# ---------------------------------------------------------------------------
# 1. Alpha Sensitivity
# ---------------------------------------------------------------------------

def eta_sensitivity(instance, env, seed=42,
                    eta_values=(0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6),
                    time_limit=600):
    """
    Sweep eta (energy budget scaler) and collect solver metrics.

    Returns a DataFrame with columns:
        eta, nodes_visited, objective, solve_time, mip_gap,
        energy_used, energy_budget, energy_utilization
    """
    rows = []
    for eta in eta_values:
        instance.update_scaling(eta=eta)
        results = solve_model_gurobi(instance, seed=seed,
                                     time_limit=time_limit, env=env)

        if results and results.get('status') == GRB.OPTIMAL:
            tour = results['tour']
            n_visited = len(tour) - 1  # tour includes depot return

            total_energy = sum(
                instance.drone.socp_energy_function(
                    ad['t'], ad['y'], ad['z']
                ) for ad in results['arc_data'].values()
            )

            rows.append({
                'eta': eta,
                'nodes_visited': n_visited,
                'objective': round(results['obj'], 4),
                'solve_time': round(results['solve_time'], 2),
                'mip_gap': round(results['gap'], 6),
                'energy_used': round(total_energy, 2),
                'energy_budget': round(instance.max_energy, 2),
                'energy_util_pct': round(total_energy / instance.max_energy * 100, 2),
            })
        else:
            rows.append({
                'eta': eta,
                'nodes_visited': 0,
                'objective': None,
                'solve_time': None,
                'mip_gap': None,
                'energy_used': None,
                'energy_budget': round(instance.max_energy, 2),
                'energy_util_pct': None,
            })

    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# 3. Arrival Time Histogram
# ---------------------------------------------------------------------------

def arrival_time_histogram(instance, results_dict, ax=None):
    """
    Plot histogram of normalized arrival positions within time windows.

    For each visited node i:  position = (a_i - e_i) / (l_i - e_i)
    0 = arrived at earliest, 1 = arrived at latest.

    Parameters
    ----------
    instance : Environment
    results_dict : dict or list of (label, results_dict)
        Single results dict, or list of (label, dict) for overlay comparison.
    ax : matplotlib Axes, optional
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 5))

    # Normalize input to list of (label, results)
    if isinstance(results_dict, dict):
        pairs = [('', results_dict)]
    else:
        pairs = results_dict

    graph = instance.graph

    for label, res in pairs:
        arrivals = res['arrival_times']
        positions = []
        base = instance.drone.base

        for node, arr in arrivals.items():
            if node == base:
                continue
            e_i, l_i = graph.nodes[node]['time_window']
            delta = l_i - e_i
            if delta > 0:
                positions.append((arr - e_i) / delta)

        ax.hist(positions, bins=20, alpha=0.6, edgecolor='black',
                label=label if label else None, range=(0, 1))

    ax.set_xlabel('Normalized arrival position  (0 = earliest, 1 = latest)')
    ax.set_ylabel('Number of nodes')
    ax.set_title('Arrival Time Distribution Within Time Windows')
    if len(pairs) > 1:
        ax.legend()
    ax.set_xlim(0, 1)
    plt.tight_layout()
    return ax


def arrival_time_comparison(instance, env, seed=42, time_limit=600,
                            slope_configs=None):
    """
    Run solver with different slope configurations and overlay histograms.

    Parameters
    ----------
    slope_configs : list of (label, Graph)
        Each Graph should already be constructed with the desired slope setting.
        Example: [('zero', Graph(path, slope='zero')),
                  ('random', Graph(path, seed=1))]
    """
    from uav_routing.environment.calibration import calibrate
    from uav_routing.environment import Environment

    fig, ax = plt.subplots(figsize=(9, 5))
    drone = instance.drone

    pairs = []
    for label, graph in slope_configs:
        calib = calibrate(graph=graph,
                          calibration_method="campaign",
                          tour_length=len(graph.nodes) // 2,
                          drone=drone,
                          drone_sortie_time=3.0)
        inst = Environment(calib, drone, eta=instance.eta)
        results = solve_model_gurobi(inst, seed=seed,
                                     time_limit=time_limit, env=env)
        if results and results.get('status') == GRB.OPTIMAL:
            pairs.append((label, results))

    arrival_time_histogram(instance, pairs, ax=ax)
    return fig


# ---------------------------------------------------------------------------
# 4. Speed Histogram
# ---------------------------------------------------------------------------

def speed_histogram(instance, results, ax=None):
    """
    Histogram of arc speeds v = L / t for active arcs in a single solution.
    Marks v_opt and v_loiter with vertical lines.
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 5))

    drone = instance.drone
    speeds = []
    for arc, ad in results['arc_data'].items():
        if ad['t'] > 1e-6:
            speeds.append(ad['L'] / ad['t'])

    ax.hist(speeds, bins=20, alpha=0.7, edgecolor='black', color='steelblue')
    ax.axvline(drone.optimum_speed, color='red', linestyle='--',
               label=f'v_opt = {drone.optimum_speed:.1f} m/s')
    ax.axvline(drone.loiter_speed, color='orange', linestyle='--',
               label=f'v_loiter = {drone.loiter_speed:.1f} m/s')
    ax.axvline(drone.speed_min, color='gray', linestyle=':',
               label=f'v_min = {drone.speed_min:.0f} m/s')
    ax.axvline(drone.speed_max, color='gray', linestyle=':',
               label=f'v_max = {drone.speed_max:.0f} m/s')

    ax.set_xlabel('Arc speed (m/s)')
    ax.set_ylabel('Number of arcs')
    ax.set_title('Speed Distribution Across Active Arcs')
    ax.legend()
    plt.tight_layout()
    return ax


def speed_histogram_by_eta(instance, env, seed=42,
                           eta_values=(0.6, 1.0, 1.5),
                           time_limit=600):
    """
    Side-by-side speed histograms for different eta values.
    Shows how the drone adjusts speed when energy is tight vs abundant.
    """
    n = len(eta_values)
    fig, axes = plt.subplots(1, n, figsize=(5 * n, 4), sharey=True)
    if n == 1:
        axes = [axes]

    drone = instance.drone

    for ax, eta in zip(axes, eta_values):
        instance.update_scaling(eta=eta)
        results = solve_model_gurobi(instance, seed=seed,
                                     time_limit=time_limit, env=env)
        if results and results.get('status') == GRB.OPTIMAL:
            speeds = []
            for ad in results['arc_data'].values():
                if ad['t'] > 1e-6:
                    speeds.append(ad['L'] / ad['t'])

            ax.hist(speeds, bins=15, alpha=0.7, edgecolor='black',
                    color='steelblue', range=(drone.speed_min, drone.speed_max))
            ax.axvline(drone.optimum_speed, color='red', linestyle='--', lw=1)
            ax.axvline(drone.loiter_speed, color='orange', linestyle='--', lw=1)
        ax.set_title(f'eta = {eta}')
        ax.set_xlabel('Speed (m/s)')

    axes[0].set_ylabel('Number of arcs')
    fig.suptitle('Speed Distribution by Energy Budget', y=1.02)
    plt.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# 5. Scalability Table
# ---------------------------------------------------------------------------

def scalability_table(graph_path_list, drone_class, env, seed=42,
                      time_limit=600, eta=1.0):
    """
    Solve instances of varying size and report model statistics.

    Parameters
    ----------
    graph_path_list : list of (label, path)
        Each entry is (label, path_to_solomon_file).
    drone_class : class
        The Drone class to instantiate (e.g. Drone).
    env : gurobipy.Env

    Returns DataFrame with columns:
        label, nodes, variables, constraints, objective,
        solve_time, mip_gap, nodes_visited
    """
    from uav_routing.environment.calibration import calibrate
    from uav_routing.environment import Environment
    from uav_routing.environment.graph import Graph

    rows = []
    for label, path in graph_path_list:
        graph = Graph(path=path, seed=1)
        drone = drone_class(base=graph.graph['base'])
        calib = calibrate(graph=graph,
                          calibration_method="campaign",
                          tour_length=len(graph.nodes) // 2,
                          drone=drone,
                          drone_sortie_time=3.0)
        inst = Environment(calib, drone, eta=eta)

        # We need model stats, so use stats=True and capture output
        results = solve_model_gurobi(inst, seed=seed,
                                     time_limit=time_limit, env=env,
                                     stats=False)

        n_nodes = len(graph.nodes)
        n_arcs = n_nodes * (n_nodes - 1)
        # 8 var families: x, t, L, y, z, s (arc-indexed) + w, a (node-indexed)
        n_vars = 6 * n_arcs + 2 * n_nodes
        # Rough constraint count: flow(2n) + tw(2n) + per-arc(~8 per arc) + mtz(2*arcs) + energy(1)
        n_constrs = 2 * n_nodes + 2 * n_nodes + 8 * n_arcs + 2 * n_arcs + 1

        row = {
            'label': label,
            'nodes': n_nodes,
            'arcs': n_arcs,
            'variables': n_vars,
            'constraints_approx': n_constrs,
        }

        if results and results.get('status') == GRB.OPTIMAL:
            row.update({
                'objective': round(results['obj'], 4),
                'solve_time': round(results['solve_time'], 2),
                'mip_gap': round(results['gap'], 6),
                'nodes_visited': len(results['tour']) - 1,
            })
        else:
            row.update({
                'objective': None,
                'solve_time': time_limit,
                'mip_gap': None,
                'nodes_visited': 0,
            })

        rows.append(row)

    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# 6. Value of Loitering
# ---------------------------------------------------------------------------

def value_of_loitering(instance, env, seed=42,
                       eta_values=(0.6, 0.8, 1.0, 1.2, 1.5),
                       time_limit=600):
    """
    Compare full model vs no-loiter model across eta values.

    Loitering allows L[i,j] > d[i,j], meaning the drone can fly extra
    distance to adjust arrival times. The no-loiter model forces L = d*x.

    Returns DataFrame with columns:
        eta, obj_full, obj_no_loiter, gap_pct, nodes_full, nodes_no_loiter
    """
    rows = []
    for eta in eta_values:
        instance.update_scaling(eta=eta)

        res_full = solve_model_gurobi(instance, seed=seed,
                                      time_limit=time_limit, env=env,
                                      no_loiter=False)
        res_nol = solve_model_gurobi(instance, seed=seed,
                                     time_limit=time_limit, env=env,
                                     no_loiter=True)

        obj_full = res_full['obj'] if res_full and res_full.get('status') == GRB.OPTIMAL else None
        obj_nol = res_nol['obj'] if res_nol and res_nol.get('status') == GRB.OPTIMAL else None
        n_full = len(res_full['tour']) - 1 if obj_full is not None else 0
        n_nol = len(res_nol['tour']) - 1 if obj_nol is not None else 0

        gap_pct = None
        if obj_full and obj_nol and obj_nol > 0:
            gap_pct = round((obj_full - obj_nol) / obj_nol * 100, 4)

        rows.append({
            'eta': eta,
            'obj_full': round(obj_full, 4) if obj_full else None,
            'obj_no_loiter': round(obj_nol, 4) if obj_nol else None,
            'loiter_gain_pct': gap_pct,
            'nodes_full': n_full,
            'nodes_no_loiter': n_nol,
        })

    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# 7. MIP Gap Convergence Plot (slope impact on solver performance)
# ---------------------------------------------------------------------------

def _gap_callback(model, where):
    """Gurobi callback that records (time, gap) pairs during MIP solving."""
    if where == GRB.Callback.MIP:
        runtime = model.cbGet(GRB.Callback.RUNTIME)
        obj_best = model.cbGet(GRB.Callback.MIP_OBJBST)
        obj_bound = model.cbGet(GRB.Callback.MIP_OBJBND)
        if obj_best < GRB.INFINITY and abs(obj_best) > 1e-10:
            gap = abs(obj_bound - obj_best) / abs(obj_best)
            model._gap_log.append((runtime, gap * 100))


def solve_with_gap_log(instance, seed, time_limit, env, no_loiter=False):
    """
    Solve the model and return (results, gap_log) where gap_log is
    a list of (time_seconds, gap_percent) recorded during solving.
    """
    drone = instance.drone
    graph = instance.graph
    t_norm = instance._t_norm
    d_norm = instance._d_norm
    v_opt = drone.optimum_speed

    with gp.Model("UAV_MISOCP_Routing", env=env) as mdl:
        mdl.Params.Seed = seed
        mdl.Params.TimeLimit = time_limit
        mdl.Params.MIPFocus = 1
        mdl.Params.Threads = 0
        mdl.Params.Method = 2
        mdl.Params.ScaleFlag = 2

        N = list(graph.nodes)
        E = [(i, j) for i in N for j in N if i != j]
        base = drone.base
        T_max = instance.T_max_s
        speed_max = instance.speed_max_s
        speed_min = instance.speed_min_s

        max_t = T_max
        max_L = speed_max * T_max
        max_z = T_max / speed_min
        max_s = speed_max * max_L
        max_y = (speed_max ** 3) * T_max

        x = mdl.addVars(E, vtype=GRB.BINARY, name='x')
        t = mdl.addVars(E, lb=0, ub=max_t, vtype=GRB.CONTINUOUS, name='t')
        L = mdl.addVars(E, lb=0, ub=max_L, vtype=GRB.CONTINUOUS, name='L')
        y = mdl.addVars(E, lb=0, ub=max_y, vtype=GRB.CONTINUOUS, name='y')
        z = mdl.addVars(E, lb=0, ub=max_z, vtype=GRB.CONTINUOUS, name='z')
        s = mdl.addVars(E, lb=0, ub=max_s, vtype=GRB.CONTINUOUS, name='s')
        w = mdl.addVars(N, vtype=GRB.BINARY, name='w')
        a = mdl.addVars(N, lb=0, ub=T_max, vtype=GRB.CONTINUOUS, name='a')

        obj = gp.quicksum(
            graph.nodes[i]['info_at_lowest'] * w[i] +
            graph.nodes[i]['info_slope'] * t_norm * (
                a[i] - graph.nodes[i]['time_window'][0] / t_norm * w[i])
            for i in N
        )
        mdl.setObjective(obj, GRB.MAXIMIZE)

        mdl.addConstr(w[base] == 1, name='start_at_depot')
        for i in N:
            mdl.addConstr(gp.quicksum(x[i, j] for (ii, j) in E if ii == i) == w[i])
            mdl.addConstr(gp.quicksum(x[j, i] for (j, ii) in E if ii == i) == w[i])
        for i in N:
            e_i, l_i = graph.nodes[i]['time_window']
            mdl.addConstr(a[i] >= e_i / t_norm * w[i])
            mdl.addConstr(a[i] <= l_i / t_norm * w[i])

        for (i, j) in E:
            dist = instance.d_scaled[i, j]
            mdl.addConstr(L[i, j] >= dist * x[i, j])
            if no_loiter:
                mdl.addConstr(L[i, j] <= dist * x[i, j])
            mdl.addConstr(L[i, j] <= speed_max * t[i, j])
            mdl.addConstr(L[i, j] >= speed_min * t[i, j])
            mdl.addConstr(t[i, j] <= T_max * x[i, j])
            mdl.addConstr(t[i, j] * t[i, j] <= z[i, j] * L[i, j])
            mdl.addConstr(L[i, j] * L[i, j] <= t[i, j] * s[i, j])
            mdl.addConstr(s[i, j] * s[i, j] <= L[i, j] * y[i, j])
            mdl.addConstr(y[i, j] <= speed_max ** 3 * t[i, j])
            mdl.addConstr(z[i, j] <= t[i, j] / speed_min)

        for (i, j) in E:
            if i == base:
                mdl.addConstr(a[j] >= t[base, j] - T_max * (1 - x[base, j]))
                mdl.addConstr(a[j] <= t[base, j] + T_max * (1 - x[base, j]))
            else:
                mdl.addConstr(a[j] >= a[i] + t[i, j] - T_max * (1 - x[i, j]))
                mdl.addConstr(a[j] <= a[i] + t[i, j] + T_max * (1 - x[i, j]))

        base_energy = instance.calib.scaled_max_energy
        c1_n = drone.c_1 * v_opt ** 2 * d_norm / base_energy
        c2_n = drone.c_2 / v_opt ** 2 * d_norm / base_energy
        total_energy = gp.quicksum(
            c1_n * y[i, j] + c2_n * z[i, j] for (i, j) in E
        )
        mdl.addConstr(total_energy <= instance.eta, name='energy_budget')
        mdl.update()

        # Attach gap log and solve with callback
        mdl._gap_log = []
        mdl.optimize(_gap_callback)

        results = {}
        if mdl.Status == GRB.OPTIMAL or mdl.SolCount > 0:
            arrival_times = {i: a[i].X * t_norm for i in N}
            active_arcs = [edge for edge, var in x.items() if var.X > 0.5]
            arc_data = {}
            for (i, j) in active_arcs:
                arc_data[(i, j)] = {
                    "t": t[i, j].X * t_norm,
                    "L": L[i, j].X * d_norm,
                    "y": y[i, j].X * (d_norm * v_opt ** 2),
                    "z": z[i, j].X * (d_norm / v_opt ** 2),
                    "d": graph[i][j]['distance'],
                    "tw": graph.nodes[j]['time_window'],
                }
            results = {
                "status": mdl.Status,
                "obj": mdl.ObjVal,
                "solve_time": mdl.Runtime,
                "gap": mdl.MIPGap,
                "arrival_times": arrival_times,
                "arc_data": arc_data,
                "active_arcs": active_arcs,
                "tour": construct_tour(base, active_arcs),
            }

        gap_log = list(mdl._gap_log)

    return results, gap_log


def gap_convergence_plot(instance, env, seed=42, time_limit=600,
                         slope_configs=None):
    """
    Plot MIP gap vs. solve time for different slope configurations.

    Parameters
    ----------
    slope_configs : list of (label, Graph)
        Each Graph constructed with desired slope setting.
        Example: [('random slopes', Graph(path, seed=1)),
                  ('zero slopes',   Graph(path, slope='zero'))]
    """
    from uav_routing.environment.calibration import calibrate
    from uav_routing.environment import Environment

    fig, ax = plt.subplots(figsize=(10, 5))
    drone = instance.drone
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']

    for idx, (label, graph) in enumerate(slope_configs):
        calib = calibrate(graph=graph,
                          calibration_method="campaign",
                          tour_length=len(graph.nodes) // 2,
                          drone=drone,
                          drone_sortie_time=3.0)
        inst = Environment(calib, drone, eta=instance.eta)
        _, gap_log = solve_with_gap_log(inst, seed=seed,
                                        time_limit=time_limit, env=env)

        if gap_log:
            times, gaps = zip(*gap_log)
            color = colors[idx % len(colors)]
            ax.plot(times, gaps, label=label, color=color, linewidth=1.5)

    ax.set_xlabel('Solve time (seconds)')
    ax.set_ylabel('MIP Gap (%)')
    ax.set_title('MIP Gap Convergence: Slope Impact on Solver Performance')
    ax.legend()
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    return fig
