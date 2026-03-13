

import time
import pandas as pd
from typing import Optional
import gurobipy as gp
from gurobipy import GRB
from uav_routing.solver.prune import prune 
    



def solve_model_gurobi(instance, seed, time_limit, env, prune=False, stats=False, no_loiter=False):
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
        obj = gp.quicksum(
            graph.nodes[i]['info_at_lowest'] * w[i] +
            graph.nodes[i]['info_slope'] * t_norm * (a[i] - graph.nodes[i]['time_window'][0] / t_norm * w[i])
            for i in N
        )
        mdl.setObjective(obj, GRB.MAXIMIZE)

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
            if i == base:
                mdl.addConstr(a[j] >= t[base, j] - T_max * (1 - x[base, j]), name=f'base_mtz_lb_{j}')
                mdl.addConstr(a[j] <= t[base, j] + T_max * (1 - x[base, j]), name=f'base_mtz_ub_{j}')
            else:
                mdl.addConstr(a[j] >= a[i] + t[i, j] - T_max * (1 - x[i, j]), name=f'mtz_lb_{i}_{j}')
                mdl.addConstr(a[j] <= a[i] + t[i, j] + T_max * (1 - x[i, j]), name=f'mtz_ub_{i}_{j}')

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
        mdl.update()
        

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
    """
    Detailed Tour Analysis Table
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

    print("\n--- Gurobi Solver Report ---")
    print(df.to_string(index=False))
    print("-" * 60)
    print(f"Objective Value:      {results['obj']:.2f}")
    print(f"MIP Gap:              {results['gap']:.4%}")
    print(f"Tour Sequence:        {'-'.join(map(str, tour))}")
    print(f"Solve Time:           {results['solve_time']:.2f} seconds")

    
    
    
def construct_tour(base, active_arcs):
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

