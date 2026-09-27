"""
exact.py
========
Gurobi MISOCP solver for the full UAV routing problem (Paper Section 4).

Jointly optimizes node selection, visit sequence, and arc speeds using
mixed-integer second-order cone programming with MTZ subtour elimination.
All variables are normalized for numerical stability; physical units are
recovered in the results dictionary.
"""

import time
import pandas as pd
from typing import Optional
import gurobipy as gp
from gurobipy import GRB
from uav_routing.solver.prune import prune




def solve_model_gurobi(instance, seed, time_limit, env, prune=False, stats=False, no_loiter=False,
                       warm_tour=None, warm_arc_solution=None, log_file=None,
                       threads=None, energy_tiebreak=1e-5):
    """Solve the full MISOCP UAV routing model with Gurobi.

    Parameters
    ----------
    instance : Environment
        Calibrated and normalized problem instance.
    seed : int
        Gurobi random seed for reproducibility.
    time_limit : float
        Maximum solve time in seconds.
    env : gurobipy.Env
        Shared Gurobi environment (suppresses license messages).
    prune : bool
        If True, apply node/arc pruning before solving.
    stats : bool
        If True, print model statistics after building.
    no_loiter : bool
        If True, force L[i,j] == d[i,j]*x[i,j] (no extra distance).
    warm_tour : list of int, optional
        Depot-first node sequence (no closing depot) injected as a MIP start.
    warm_arc_solution : dict, optional
        Fixed-tour SOCP solution for ``warm_tour`` in physical units, with
        keys ``arrivals`` (node -> s; include the depot's return time),
        ``times`` and ``lengths`` (arc -> s / m). When given, a complete
        start vector is derived so Gurobi only validates it.
    log_file : str, optional
        If given, write the full Gurobi log to this path (enables output).

    Returns
    -------
    dict
        Results with keys: status, obj, solve_time, gap, arrival_times,
        arc_data, active_arcs, tour. Empty dict if no feasible solution.
    """
    """
    Build a Gurobi MISOCP model for UAV routing with energy constraints.

    The model uses three rotated second-order cone constraints to linearize
    the nonlinear energy function E(v,d) = c_0*d/v + c_1*v^2*d + c_2*d/v.

    Parameters
    ----------
    no_loiter : bool
        If True, forces L[i,j] == d[i,j] * x[i,j] (no extra distance allowed).
    """


    # 2. Create the model using the silent environment
    with gp.Model("UAV_MISOCP_Routing", env=env) as mdl:

        # ---- Initialize Gurobi Model ----å
        #mdl = gp.Model('UAV_MISOCP_Routing')
        
        mdl.Params.Seed = seed 
        mdl.Params.TimeLimit = time_limit
        mdl.Params.MIPFocus = 1           # Reproducibility
        mdl.Params.Threads = 0          # Use all available cores
        if threads is not None:
            mdl.Params.Threads = threads
        mdl.Params.Method = 2           # Barrier (stable for SOCP)
        #mdl.Params.NumericFocus = 3     # Maximum numerical precision = slower solving
        mdl.Params.ScaleFlag = 2        # Aggressive automatic scaling
        #mdl.Params.DualReductions = 0   # Helps diagnose infeasibility
        #mdl.Params.InfUnbdInfo = 1      # Extra info on infeasible/unbounded
        #mdl.Params.ConcurrentMIP = 1     # Allow Gurobi to choose the best algorithm for MIP
        
        drone = instance.drone
        graph = instance.graph
        t_norm = instance._t_norm
        d_norm = instance._d_norm
        v_opt = instance.drone.optimum_speed
            
        # ---- Pruning (Optional) ----
        feasible_nodes, feasible_arcs = None, None
        if prune:
            feasible_nodes, feasible_arcs = prune(instance)
        
        # ---- Problem Parameters ----
        N = list(feasible_nodes) if feasible_nodes is not None else list(graph.nodes)
        E = list(feasible_arcs) if feasible_arcs is not None else [(i, j) for i in N for j in N if i != j]
        
        base = drone.base
        T_max     = instance.T_max_s  # = time_horizon / _t_norm (fixed normalization)
        speed_max = instance.speed_max_s
        speed_min = instance.speed_min_s
        max_energy = instance.max_energy

        # ---- Tight upper bounds for auxiliary variables ----
        # These reduce the feasible region and improve LP relaxation quality.
        max_t = T_max                                    # time on any single arc
        max_L = speed_max * T_max                        # max distance = max_speed * max_time
        max_z = T_max / speed_min                 # z models t/v, max when v=v_min
        max_s = speed_max * max_L                      # s models L*v, max when v=v_max
        max_y = (speed_max**3) * T_max                  # y models L*v^2, max when v=v_max

        # ---- Decision Variables (with tight bounds) ----

        x = mdl.addVars(E, vtype=GRB.BINARY, name='x')
        t = mdl.addVars(E, lb=0, ub=max_t, vtype=GRB.CONTINUOUS, name='t')
        L = mdl.addVars(E, lb=0, ub=max_L, vtype=GRB.CONTINUOUS, name='L')
        y = mdl.addVars(E, lb=0, ub=max_y, vtype=GRB.CONTINUOUS, name='y')
        z = mdl.addVars(E, lb=0, ub=max_z, vtype=GRB.CONTINUOUS, name='z')
        s = mdl.addVars(E, lb=0, ub=max_s, vtype=GRB.CONTINUOUS, name='s')

        w = mdl.addVars(N, vtype=GRB.BINARY, name='w')
        a = mdl.addVars(N, lb=0, ub=T_max, vtype=GRB.CONTINUOUS, name='a')

        # ---- Objective Function ----
        # Maximize total collected info: base_info * visited + slope * (arrival - earliest)
        # a[i] is normalized by _t_norm (fixed). Unscale inline:
        #   info = info_at_lowest + slope * (a_physical - e_physical)
        #        = info_at_lowest + slope * _t_norm * (a_scaled - e_scaled)
        t_norm = instance._t_norm
        
        

        # ---- Constraints ----
        
        # 1. Start at Depot
        mdl.addConstr(w[base] == 1, name='start_at_depot')

        # 2. Flow Conservation
        for i in N:
            mdl.addConstr(gp.quicksum(x[i, j] for (ii, j) in E if ii == i) == w[i], name=f'flow_out_{i}')
            mdl.addConstr(gp.quicksum(x[j, i] for (j, ii) in E if ii == i) == w[i], name=f'flow_in_{i}')  

        # 3. Time Windows (a[i] normalized by _t_norm)
        for i in N:
            e_i, l_i = graph.nodes[i]['time_window']
            mdl.addConstr(a[i] >= e_i / t_norm * w[i], name=f'tw_lb_{i}')
            mdl.addConstr(a[i] <= l_i / t_norm * w[i], name=f'tw_ub_{i}')

        # 4. Distance, Speed, and SOCP Cones
        for (i, j) in E:
            dist = instance.d_scaled[i, j]

            # Path and Speed Bounds
            mdl.addConstr(L[i, j] >= dist * x[i, j],        name=f'dist_min_{i}_{j}')
            if no_loiter:
                mdl.addConstr(L[i, j] <= dist * x[i, j],    name=f'no_loiter_{i}_{j}')
                
            mdl.addConstr(L[i, j] <= speed_max * t[i, j],   name=f'speed_max_bound_{i}_{j}')
            mdl.addConstr(L[i, j] >= speed_min * t[i, j],   name=f'speed_min_bound_{i}_{j}')
            mdl.addConstr(t[i, j] <= T_max * x[i, j],       name=f'time_bound_{i}_{j}')

            # Conic Constraints (rotated second-order cones)
            # 1. t² ≤ z · L   → z models t/v (≈ 1/speed)
            mdl.addConstr(t[i, j] * t[i, j] <= z[i, j] * L[i, j],   name=f'cone1_{i}_{j}')
            # 2. L² ≤ t · s   → s models L·v (= L²/t)
            mdl.addConstr(L[i, j] * L[i, j] <= t[i, j] * s[i, j],   name=f'cone2_{i}_{j}')
            # 3. s² ≤ L · y   → y models L·v² (combines to give v³·t in energy)
            mdl.addConstr(s[i, j] * s[i, j] <= L[i, j] * y[i, j],   name=f'cone3_{i}_{j}')

            # Linking bounds (tighten the conic relaxation)
            # 4. y ≤ v_max³ · t  (when x=0, t=0 forces y=0 via cone)
            mdl.addConstr(y[i, j] <= speed_max**3 * t[i, j],         name=f'y_link_{i}_{j}')
            # 5. z ≤ t / v_min
            mdl.addConstr(z[i, j] <= t[i, j] / speed_min,            name=f'z_link_{i}_{j}')

        # 5. MTZ Subtour Elimination
        for (i, j) in E:
            # Desrochers-Laporte-style per-leg big-Ms (paper Section 3.3).
            # Lower link: with x_ij = 0 the worst case is a_i = l_i, a_j = 0,
            # so M = l_i suffices (depot-out legs need none: a_j >= t_0j is
            # valid outright). Upper link: worst case a_j = l_j, a_i = 0, so
            # M = l_j (l_j - e_i would wrongly bind when w_i = 0).
            M_lo = graph.nodes[i]['time_window'][1] / t_norm \
                if i != base else 0.0
            M_up = graph.nodes[j]['time_window'][1] / t_norm
            if i == base:
                mdl.addConstr(a[j] >= t[base, j] - M_lo * (1 - x[base, j]), name=f'base_mtz_lb_{j}')
                mdl.addConstr(a[j] <= t[base, j] + M_up * (1 - x[base, j]), name=f'base_mtz_ub_{j}')
            else:
                mdl.addConstr(a[j] >= a[i] + t[i, j] - M_lo * (1 - x[i, j]), name=f'mtz_lb_{i}_{j}')
                mdl.addConstr(a[j] <= a[i] + t[i, j] + M_up * (1 - x[i, j]), name=f'mtz_ub_{i}_{j}')

        # 6. Energy Budget
        # Variables y, z are normalized by _d_norm (fixed, alpha-independent).
        # Physical energy: E = c_1 * v_opt² * y_physical + c_2/v_opt² * z_physical
        # With y_physical = y_scaled * _d_norm * v_opt²  and  z_physical = z_scaled * _d_norm / v_opt²:
        #   E = (c_1 * v_opt² * _d_norm) * y_scaled + (c_2 / v_opt² * _d_norm) * z_scaled
        # Normalizing both sides by max_energy → RHS = alpha (changes correctly with alpha)
        # Coefficients use base_energy (fixed, alpha=1), RHS = alpha
        d_norm      = instance._d_norm
        v_opt       = drone.optimum_speed
        base_energy = instance.calib.scaled_max_energy  # alpha=1 energy budget
        c1_n   = drone.c_1 * v_opt**2 * d_norm / base_energy   # fixed coefficient for y
        c2_n   = drone.c_2 / v_opt**2 * d_norm / base_energy   # fixed coefficient for z

        total_energy = gp.quicksum(
            c1_n * y[i, j] + c2_n * z[i, j]
            for (i, j) in E
        )
        mdl.addConstr(total_energy <= instance.eta, name='energy_budget')

        reward_expr = gp.quicksum(
            graph.nodes[i]['info_at_lowest'] * w[i] +
            graph.nodes[i]['info_slope'] * t_norm * (a[i] - graph.nodes[i]['time_window'][0] / t_norm * w[i])
            for i in N
        )
        # Energy tie-break: among schedules collecting the same reward, prefer
        # the one that burns less energy. The coefficient must be large enough
        # to be decisive against the solver's own tolerances and small enough
        # that it never trades reward for energy; the reported value is the
        # reward itself (results["reward"]), not this penalised objective.
        obj = reward_expr - energy_tiebreak * total_energy
        
    
        mdl.setObjective(obj, GRB.MAXIMIZE)

        mdl.update()

        # ---- Optional Gurobi log capture ----
        if log_file is not None:
            mdl.Params.OutputFlag = 1
            mdl.Params.LogFile = log_file

        # ---- Optional MIP start from a heuristic tour ----
        # The caller passes the tour plus its fixed-tour SOCP solution in
        # PHYSICAL units (arrivals, per-arc travel times and path lengths);
        # every variable's start value is derived here in the model's own
        # normalization, with the auxiliaries y, z, s set cone-tight. Gurobi
        # then only has to CHECK the vector, not complete it: a binaries-only
        # partial start dies in start-completion on the 200-node instances,
        # and solving with hard-fixed binaries trips a barrier
        # false-infeasibility on the degenerate inactive cones.
        if warm_tour is not None:
            visited = set(warm_tour)
            tour_arcs = set(zip(warm_tour, warm_tour[1:]))
            tour_arcs.add((warm_tour[-1], base))
            for i in N:
                w[i].Start = 1.0 if i in visited else 0.0
            for (i, j) in E:
                x[i, j].Start = 1.0 if (i, j) in tour_arcs else 0.0
            if warm_arc_solution is not None:
                arr = warm_arc_solution["arrivals"]   # node -> seconds
                tt = warm_arc_solution["times"]       # arc -> seconds
                LL = warm_arc_solution["lengths"]     # arc -> meters
                for i in N:
                    a[i].Start = arr.get(i, 0.0) / t_norm
                for (i, j) in E:
                    if (i, j) in tour_arcs:
                        tp = tt[(i, j)]
                        Lp = LL[(i, j)]
                        zp = tp * tp / Lp
                        sp = Lp * Lp / tp
                        yp = sp * sp / Lp
                        t[i, j].Start = tp / t_norm
                        L[i, j].Start = Lp / d_norm
                        y[i, j].Start = yp / (d_norm * v_opt ** 2)
                        z[i, j].Start = zp * v_opt ** 2 / d_norm
                        s[i, j].Start = sp / (d_norm * v_opt)
                    else:
                        for var in (t[i, j], L[i, j], y[i, j],
                                    z[i, j], s[i, j]):
                            var.Start = 0.0

        # ---- Solve ----
        mdl.optimize()

        
        # ---- Print Model Statistics ----
        if stats == True:
            print("="*70)
            print("MODEL STATISTICS")
            print("="*70)
            print(f"Number of variables: {mdl.NumVars}")
            print(f"  - Binary variables: {sum(1 for v in mdl.getVars() if v.VType == GRB.BINARY)}")
            print(f"  - Continuous variables: {sum(1 for v in mdl.getVars() if v.VType == GRB.CONTINUOUS)}")
            print(f"Number of linear constraints: {mdl.NumConstrs}")
            print(f"Number of quadratic constraints: {mdl.NumQConstrs}")
            print(f"  T_max = {T_max * t_norm}")
            print(f"  max_energy = {max_energy}")
            print(f"\nVariable upper bounds:")
            print(f"  max_L = {max_L:.2e}")
            print(f"  max_y = {max_y:.2e}")
            print(f"  max_z = {max_z:.2e}")
            print(f"  max_s = {max_s:.2e}")
            print(f"  Unscaled energy RHS = {max_energy:.2f}")
            print("="*70)
            
        results = {}
        # ---  Data Extraction ---
        if mdl.SolCount > 0:

            arrival_times = {i: a[i].X * t_norm for i in N}
            active_arcs = [edge for edge, var in x.items() if var.X > 0.5]
            arc_data = {}
            
            for (i, j) in active_arcs:
                arc_data[(i, j)] = {
                    "t": t[i, j].X * t_norm,
                    "L": L[i, j].X * d_norm,
                    "y": y[i, j].X * (d_norm * v_opt**2),
                    "z": z[i, j].X * (d_norm / v_opt**2),
                    "d": graph[i][j]['distance'],
                    "tw": graph.nodes[j]['time_window'],
                }
                
            results = {
                "status": mdl.Status,
                "obj": mdl.ObjVal,
                "reward": reward_expr.getValue(),
                "energy_tiebreak": energy_tiebreak,
                "objbound": mdl.ObjBound,
                "solve_time": mdl.Runtime,
                "gap": mdl.MIPGap,
                "arrival_times": arrival_times,
                "arc_data": arc_data,
                "active_arcs": active_arcs,
                "tour": construct_tour(base, active_arcs)
            }
            
        else:
            print("No feasible solution found!")
    
    mdl.dispose() # dispose of the MODEL only, keep the environment alive
    
    return results
            



def print_table(instance, results):
    """Print a detailed tour analysis table with per-arc energy, speed, and timing.

    Parameters
    ----------
    instance : Environment
        The problem instance used for solving.
    results : dict
        Results dictionary from solve_model_gurobi.
    """
    import pandas as pd
    
    graph = instance.graph
    drone = instance.drone

    max_energy = instance.max_energy
    base = drone.base

    active_arcs = results["active_arcs"]
    arrival_times = results['arrival_times']
    
    tour = results["tour"]
    tour_data = []
    
    total_t, total_l, total_e, total_e_2 = 0, 0, 0, 0

    for idx in range(len(tour) - 1):
        
        i, j = tour[idx], tour[idx+1]
        arc_values = results["arc_data"][(i,j)]

        L = arc_values['L'] 
        t =  arc_values['t'] 
        v = L / t
        d = arc_values['d']
        y = arc_values['y'] 
        z = arc_values['z'] 
        tw = arc_values['tw']
        
        loiter_time = max(0, t - (d / v))
        
        energy = drone.energy_function(v, L) 
        
        energy_2 = drone.socp_energy_function(t, y, z)
        energy_pct = (energy / max_energy) * 100

        tour_data.append({
            "Edge": f"{i}->{j}",
            "Total Time (s)": round(t, 2),
            "Loiter Time (s)": round(loiter_time, 2),
            "Speed (m/s)": round(v, 2),
            "SOCP Energy (J)": round(energy_2, 2),
            "Real Energy (J)": round(energy, 2),
            "Energy %": round(energy_pct, 4),
            "Arrival Time (s)": round(arrival_times[j], 2),
            "Time Window": f"[{tw[0]:.1f}, {tw[1]:.1f}]",
            "Slope": graph.nodes[j]['info_slope'],                      # graphtan bilgi cekiyosun
        })

        total_t += t
        total_l += loiter_time
        total_e += energy
        total_e_2 += energy_2

    df = pd.DataFrame(tour_data)
    if not df.empty:
        summary_row = pd.DataFrame([{
            "Edge": "TOTAL TOUR",
            "Total Time (s)": round(total_t, 2),
            "Loiter Time (s)": round(total_l, 2),
            "SOCP Energy (J)": round(total_e_2, 2),
            "Real Energy (J)": round(total_e, 2),
            "Energy %": round((total_e_2 / max_energy) * 100, 4),
        }])
        df = pd.concat([df, summary_row], ignore_index=True)

    from IPython.display import display

    summary_df = pd.DataFrame([{
        "Objective": f"{results['obj']:.2f}",
        "MIP Gap": f"{results['gap']:.4%}",
        "Tour": '-'.join(map(str, tour)),
        "Solve Time (s)": f"{results['solve_time']:.2f}",
    }])

    print("\n--- Gurobi Solver Report ---")
    display(df)
    display(summary_df)

    
    
    
def construct_tour(base, active_arcs):
    """Reconstruct the ordered tour sequence from active arcs.

    Parameters
    ----------
    base : int
        Depot node ID.
    active_arcs : list of (int, int)
        Active arcs from the MISOCP solution.

    Returns
    -------
    list of int
        Ordered tour starting and ending at the depot.
    """
    seq = [base]
    curr = base
    visited = {base}
    
    while True:
        next_node = next((j for i, j in active_arcs if i == curr), None)
        if next_node is None or next_node == base or next_node in visited:
            if next_node == base:
                seq.append(base)
            break
        seq.append(next_node)
        visited.add(next_node)
        curr = next_node
    return seq



def find_large_coefficients(mdl, threshold=1e4):
    """Find all constraints with coefficients outside [1/threshold, threshold]."""
    print(f"\nCoefficients > {threshold:.0e} or < {1/threshold:.0e}:")
    print("-" * 70)
    
    for c in mdl.getConstrs():
        row = mdl.getRow(c)
        for k in range(row.size()):
            coeff = abs(row.getCoeff(k))
            if coeff > threshold or (coeff > 0 and coeff < 1/threshold):
                print(f"  [{c.ConstrName:35s}] "
                      f"var={row.getVar(k).VarName:12s} "
                      f"coeff={coeff:.4e}")
    
    # Also check RHS
    print(f"\nRHS > {threshold:.0e} or < {1/threshold:.0e}:")
    print("-" * 70)
    for c in mdl.getConstrs():
        rhs = abs(c.RHS)
        if rhs > threshold or (rhs > 0 and rhs < 1/threshold):
            print(f"  [{c.ConstrName:35s}] RHS={rhs:.4e}")

    # Check bounds
    print(f"\nBounds > {threshold:.0e} or < {1/threshold:.0e}:")
    print("-" * 70)
    for v in mdl.getVars():
        if v.UB != GRB.INFINITY and abs(v.UB) > threshold:
            print(f"  [{v.VarName:20s}] UB={v.UB:.4e}")
        if abs(v.LB) > threshold:
            print(f"  [{v.VarName:20s}] LB={v.LB:.4e}")

