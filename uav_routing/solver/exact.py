

from docplex.mp.model import Model
from docplex.mp.conflict_refiner import ConflictRefiner

import time
import pandas as pd

from typing import Optional
from uav_routing.solver.prune import prune_and_report
import gurobipy as gp
from gurobipy import GRB


def debug(sol, x, w, a, graph, nodes, feasible_arcs):
    
    print("\n=== SOLUTION DEBUG ===")
    print("Visited nodes (w[i] >= 0.5):")
    for i in nodes:
        if sol.get_value(w[i]) >= 0.5:
            print(f"  w[{i}] = {sol.get_value(w[i]):.3f}")

    print("\nActive edges (x[i,j] >= 0.5):")
    active_count = 0
    for (i,j) in feasible_arcs:
        if sol.get_value(x[i,j]) >= 0.5:
            print(f"  x[{i},{j}] = {sol.get_value(x[i,j]):.3f}")
            active_count += 1

    print(f"\nTotal active edges: {active_count}")
    print(f"Objective breakdown:")
    obj_val = 0
    for i in nodes:
        w_val = sol.get_value(w[i])
        if w_val >= 0.01:
            a_val = sol.get_value(a[i])
            info_base = graph.nodes[i]['info_at_lowest']
            info_slope = graph.nodes[i]['info_slope']
            info_contrib = info_base * w_val + info_slope * (a_val - graph.nodes[i]['time_window'][0] * w_val)
            obj_val += info_contrib
            print(f"  Node {i}: w={w_val:.3f}, a={a_val:.1f}, contrib={info_contrib:.2f}")

    print(f"Total objective: {obj_val:.2f}")
    print(f"Solver reported: {sol.objective_value:.2f}")
    

def construct_tour(sol, x_vars, base):
    # Pre-filter: Get only the arcs used in the solution
    active_arcs = { (i, j) for (i, j), var in x_vars.items() if sol.get_value(var) > 0.5 }
    
    seq = [base]
    curr = base
    visited = {base}
    
    while True:
        next_node = next((j for i, j in active_arcs if i == curr), None)
        
        # Optional check
        #if sol.get_value(w[next_node]) < 0.5:
            #print(f"Warning: Path entered node {next_node} but w_{next_node} is 0!")
        
        if next_node is None:
            # If we were at a node but have no active outgoing arc
            if curr == base and len(seq) == 1:
                return [base] # Case where drone never leaves depot
            raise ValueError(f"Flow Error: Node {curr} has no outgoing active arc in solution.")
        
        if next_node == base:
            seq.append(base)
            break
            
        if next_node in visited:
            print(f"Warning: Subtour detected! Node {next_node} visited twice.")
            seq.append(next_node)
            break
            
        seq.append(next_node)
        visited.add(next_node)
        curr = next_node
        
        # Absolute safety break
        if len(seq) > len(active_arcs) + 1:
            break
            
    return seq


# Usage:
# mdl = build_uav_misocp(graph, drone)
# print_model_pre_solve_stats(mdl)

def solve_exact_benchmark_subgraph(graph, drone, prune=False, time_limit=600):
    max_energy = drone.max_energy
    # 1. Determine Subset


        
    mdl, x = build_model(graph, drone)
    #mdl.parameters.mip.tolerances.mipgap = 0.005
    #mdl.parameters.mip.tolerances.mipgap = 1e-9
    mdl.parameters.mip.tolerances.absmipgap = 0.05
    # Equivalent to "Numerical Focus" or "High Precision Mode"  
    mdl.parameters.emphasis.numerical = 1
    #mdl.parameters.barrier.convergetol = 1e-12
    #mdl.parameters.timelimit = time_limit
    mdl.parameters.dettimelimit = 2000000
    # Enable Kappa computation before solving
    #mdl.parameters.mip.strategy.kappastats = 1
    mdl.parameters.emphasis.mip = 0
    #mdl.parameters.preprocessing.presolve = 0
    mdl.parameters.parallel = 1
    mdl.parameters.randomseed = 1
    mdl.parameters.threads = 11
    # 1. Disable General Heuristics frequency
    #mdl.parameters.mip.strategy.heuristicfreq = -1
    mdl.parameters.barrier.convergetol = 1e-6
    mdl.parameters.mip.tolerances.integrality = 1e-06
    mdl.parameters.read.datacheck = 1
    mdl.parameters.read.scale = 1 
    #mdl.parameters.mip.strategy.rinsheur = -1
    mdl.parameters.mip.strategy.miqcpstrat = 2
    mdl.parameters.mip.strategy.startalgorithm = 0 

    # 2. Corrected Polishing Parameter 
    #mdl.parameters.mip.polishafter.time = 0

    # 3. Corrected Sub-MIP node limit
    # This replaces the 'limits.submipnodes' which caused the exception
    #mdl.parameters.mip.strategy.submipnodes = 0
    
    #print(mdl.print_information())
    # Disable the cutoff behavior by setting it to a very small number (for maximization)
    #mdl.parameters.mip.tolerances.uppercutoff = -1e15 

    # Turn off Presolve so the Refiner works on the original constraints
    #mdl.parameters.preprocessixng.presolve = 0

    # Ensure the solver focuses on finding ANY solution first
    #mdl.parameters.mip.emphasis = 1
    #save_model_as_lp(mdl, 'uav_model_before_solve.lp')
    # Solve
    #mdl.parameters.timelimit = time_limit
    #mdl.parameters.mip.tolerances.mipgap = 0.5 
    start_t = time.time()
    sol = mdl.solve(log_output=True) 
    solve_time = time.time() - start_t
    
    if not sol:
        # After solve() fails
        #cr = ConflictRefiner()
        #conflict_sets = cr.refine_conflict(mdl)
        #for c in conf lict_sets:
        #    print(f"Conflict found in: {c.element}")
    #    sol_relax =solve_lp_relaxation(mdl, x, w, nodes, arcs, graph, drone)

    #    if sol_relax:
    #        print("\n✓ LP Relaxation is FEASIBLE")
    #        print("  This means a continuous solution exists.")
    #        print("  Integer solution may be infeasible if constraints conflict at integer level.")
    #    else:
    #        print("\n✗ LP Relaxation is INFEASIBLE")
    #        print("  The problem has NO feasible solution (constraints are contradictory).")
            
        return {"feasible": False, "status": "Infeasible/Timeout"}

    # construct tour sequence
    seq = construct_tour(sol, x, drone.base)

    # 6. Detailed Tour Analysis Table
    tour_data = []
    total_t, total_l, total_e, total_e_2 = 0, 0, 0, 0
    arrival_times = {i: sol.get_value(f'a_{i}') for i in graph.nodes}
    
    #debug(sol, x, w, a, graph, nodes, arcs)
    for idx in range(len(seq) - 1):
        i, j = seq[idx], seq[idx+1]
        
        # Get solver variables for this specific edge
        t_ij = sol.get_value(f't_{i}_{j}')
        L_ij = sol.get_value(f'L_{i}_{j}') # Distance + Loiter dist
        d_ij = graph.edges[(i, j)]['distance'] # Real distance
        y_ij = sol.get_value(f'y_{i}_{j}')
        z_ij = sol.get_value(f'z_{i}_{j}')
        
        
        # Calculations 
        v_ij = L_ij / t_ij if t_ij > 0 else 0
        loiter_time = max(0, t_ij - (d_ij / v_ij)) if v_ij > 0 else 0
        energy_ij = drone.energy_function(v_ij, L_ij)
        energy_ij_2 = drone.socp_energy_function(t_ij, y_ij, z_ij)
        energy_pct = (energy_ij / max_energy) * 100
        
        #collected = drone.collected_info(arrival_times[j], sub_graph.nodes[j]['time_window'][0], 
        #                                                sub_graph.nodes[j]['info_slope'], sub_graph.nodes[j]['info_at_lowest'])
         
        tour_data.append({
            "Edge": f"{i}->{j}",
            "Total Time (s)": round(t_ij, 2),
            "Loiter Time (s)": round(loiter_time, 2),
            "Speed (m/s)": round(v_ij, 2),
            "Model Energy (J)": round(energy_ij_2, 2),
            "Real Energy (J)": round(energy_ij, 2), 
            "Energy %": round(energy_pct, 4),
            "Arrival Time (s)": round(arrival_times[j], 2),
            "Time Window": f"[{graph.nodes[j]['time_window'][0]}, {graph.nodes[j]['time_window'][1]}]",
            "Slope": graph.nodes[j]['info_slope'],
            "Earliest Info": graph.nodes[j]['info_at_lowest'],
            "Latest Info": round(graph.nodes[j]['info_slope']*(graph.nodes[j]['time_window'][1]-graph.nodes[j]['time_window'][0]) + graph.nodes[j]['info_at_lowest'], 2),
            #"Collected Info": round(collected, 2) 
        })
        
        total_t += t_ij
        total_l += loiter_time
        total_e += energy_ij
        total_e_2 += energy_ij_2

    # Create DataFrame
    df = pd.DataFrame(tour_data)
    
    # Add Summary Row
    summary_row = pd.DataFrame([{
        "Edge": "TOTAL TOUR",
        "Total Time (s)": round(total_t, 2),
        "Loiter Time (s)": round(total_l, 2),
        "Model Energy (J)": round(total_e_2, 2),
        "Real Energy (J)": round(total_e, 2),
        "Energy %": round((total_e / max_energy) * 100, 4)
    }])
    df = pd.concat([df, summary_row], ignore_index=True)

    # 7. Metadata Reporting
    print("\n--- Solver Report ---")
    print(df.to_string(index=False))
    print("-" * 60)
    print(f"Objective Value:      {sol.objective_value:.2f}")
    print(f"MIP Gap:              {mdl.solve_details.mip_relative_gap:.4%}")
    print(f"Tour Sequence:        {'-'.join(map(str, seq))}")
    print(f"Solve Time:           {solve_time:.2f} seconds")

    return {
        "df": df,
        "objective": sol.objective_value,
        "gap": mdl.solve_details.mip_relative_gap,
        "sequence": seq,
        "arrival_times": arrival_times,
        "solve_time": solve_time,
    }, sol

    
    

def build_uav_misocp(graph, drone, prune: Optional[bool]=True):
    
    T_max = drone.max_time
    base = drone.base
    
    if prune:
        nodes, feasible_arcs, G = prune_and_report(graph, drone)
    else: 
        nodes = set(graph.nodes)
        feasible_arcs = set()
        for edge in graph.edges:
            feasible_arcs.add(edge)
            feasible_arcs.add((edge[1], edge[0]))
        import networkx as nx
        G = nx.DiGraph(feasible_arcs)
        
    print("prune:", prune)
    mdl = Model(name='UAV_MISOCP_Routing')

    # --- Variables: ONLY create what is needed ---
    # x[i,j] only exists if (i,j) is in feasible_arcs
    x = { (i,j): mdl.binary_var(name=f'x_{i}_{j}') for (i,j) in feasible_arcs }
    t = { (i,j): mdl.continuous_var(lb=0, name=f't_{i}_{j}') for (i,j) in feasible_arcs }
    L = { (i,j): mdl.continuous_var(lb=0, name=f'L_{i}_{j}') for (i,j) in feasible_arcs }
    
    # Auxiliary SOCP variables follow the same sparsity
    y = { (i,j): mdl.continuous_var(lb=0, name=f'y_{i}_{j}') for (i,j) in feasible_arcs }
    z = { (i,j): mdl.continuous_var(lb=0, name=f'z_{i}_{j}') for (i,j) in feasible_arcs }
    s = { (i,j): mdl.continuous_var(lb=0, name=f's_{i}_{j}') for (i,j) in feasible_arcs }

    # Node variables (w and a) stay the same
    w = mdl.binary_var_dict(nodes, name='w')
    a = mdl.continuous_var_dict(nodes, lb=0, name='a')

    # --- Objective ---
    # Maximize total collected info: info incentive + penalty for delay/early arrival
    obj = mdl.sum(graph.nodes[i]['info_at_lowest'] * mdl.sum(x[j,i] for j in nodes if (j,i) in feasible_arcs) + graph.nodes[i]['info_slope'] * (a[i] - graph.nodes[i]['time_window'][0] * mdl.sum(x[j,i] for j in nodes if (j,i) in feasible_arcs)) for i in nodes)
    #mdl.add_constraint(mdl.sum(w[i] for i in nodes) == 3, ctname='restriction')
    mdl.maximize(obj)

    # --- Routing & Flow Constraints ---
    mdl.add_constraint(w[base] == 1, ctname='start_at_depot')
    
    #mdl.add_constraint(mdl.sum(x[base,j] for j in G.nodes if (base,j) in G.edges) == 1)
    #mdl.add_constraint(mdl.sum(x[i, base] for i in G.nodes if (i, base) in G.edges) == 1)
    
    #mdl.add_constraint(mdl.sum(w[i] for i in nodes) >= 2) 
    # --- Constraints: Adjust loops to use feasible_arcs ---
    #for i in nodes:
    #   for j in nodes:
    #       if (i,j) in feasible_arcs:
    #           mdl.add_constraint(x[i,j] <= w[i])
    #           mdl.add_constraint(x[i,j] <= w[j])
               
    for i in nodes:
        ## Flow in: sum only over j where (j,i) is feasible
        incoming = [x[j, i] for j in nodes if (j, i) in feasible_arcs]
        mdl.add_constraint(mdl.sum(incoming) == w[i], ctname=f'flow_in_{i}')

        # Flow out: sum only over j where (i,j) is feasible
        outgoing = [x[i, j] for j in nodes if (i, j) in feasible_arcs]
        mdl.add_constraint(mdl.sum(outgoing) == w[i], ctname=f'flow_out_{i}')

               
    # --- Time Windows ---
        mdl.add_constraint(a[i] >= graph.nodes[i]['time_window'][0] * w[i])
        mdl.add_constraint(a[i] <= graph.nodes[i]['time_window'][1] * w[i])

      
    # ---  Distance Constraints ---
    for arc in feasible_arcs:
        i, j = arc[0], arc[1]
        mdl.add_constraint(L[i, j] >= graph[i][j]['distance'] * x[i, j])
        mdl.add_constraint(L[i, j] <= drone.speed_max * t[i, j])
        mdl.add_constraint(L[i, j] >= drone.speed_min * t[i, j])
        mdl.add_constraint(t[i, j] <= T_max * x[i, j])


    # --- SOCP Conic Constraints ---
        # 1. t^2 <= z * L
        mdl.add_constraint(t[i, j]**2 <= z[i, j] * L[i, j])
        # 2. L^2 <= t * s
        mdl.add_constraint(L[i, j]**2 <= t[i, j] * s[i, j])
        # 3. s^2 <= L * y
        mdl.add_constraint(s[i, j]**2 <= L[i, j] * y[i, j])
        # 4. y <= v_max * t
        mdl.add_constraint(y[i, j] <= (drone.speed_max**3) * t[i, j])
        # 5. z <= v_min * t
        mdl.add_constraint(z[i, j] <= t[i, j] / drone.speed_min)

    
    # --- MTZ Subtour Elimination ---
        if i != 0: 
            # a_j >= a_i + t_ij - T_max(1 - x_ij)
            mdl.add_constraint(a[j] >= a[i] + t[i, j] - T_max * (1 - x[i, j]), ctname=f'mtz_lb_{i}_{j}')
            # a_j <= a_i + t_ij + T_max(1 - x_ij)
            mdl.add_constraint(a[j] <= a[i] + t[i, j] + T_max * (1 - x[i, j]), ctname=f'mtz_ub_{i}_{j}')


        if i == 0 and j != 0:
            mdl.add_constraint(a[j] >= t[0, j] - T_max * (1 - x[0, j]))
            mdl.add_constraint(a[j] <= t[0, j] + T_max * (1 - x[0, j]))
    
    
    max_energy = drone.max_energy
    # --- Energy Constraint --- 
    # SHOULD BE:
    total_energy = mdl.sum(drone.socp_energy_function(t[i, j], y[i, j], z[i, j]) for (i,j) in feasible_arcs)
    mdl.add_constraint(total_energy <= max_energy, ctname='energy_budget')

    return mdl, x, w, a, nodes, feasible_arcs

def save_model_as_lp(mdl, filename='uav_model.lp'):
    """
    Save the CPLEX model as an LP file for inspection.
    
    Parameters:
    -----------
    mdl : docplex.mp.model.Model
        The CPLEX model to save
    filename : str
        Output filename (default: 'uav_model.lp')
    """
    try:
        mdl.export_as_lp(filename)
        print(f"✓ Model exported to {filename}")
        print(f"  File size: {os.path.getsize(filename) / 1024:.2f} KB")
        
        # Print summary
        print(f"\nModel Summary:")
        print(f"  Variables: {mdl.number_of_variables}")
        print(f"  Constraints: {mdl.number_of_constraints}")
        print(f"  Binary vars: {mdl.number_of_binary_variables}")
        print(f"  Continuous vars: {mdl.number_of_continuous_variables}")
        
    except Exception as e:
        print(f"✗ Error exporting model: {e}")





def build_model(graph, drone):
    
    
    mdl = Model(name='UAV_MISOCP_Routing')
    #mdl.parameters.mip.tolerances.mipgap = 0.001
    
    N = sorted(list(graph.nodes))
    base = drone.base
    T_max = drone.max_time 
    
    
    # VARIIABLES
    keys = []
    for i in N:
        for j in N:
            if i!=j:
                keys.append((i,j))
     
    x = mdl.binary_var_dict(keys, lb=0, ub=None, name='x')
    
    t = mdl.continuous_var_dict(keys, lb=0, ub=None, name='t')
    L = mdl.continuous_var_dict(keys, lb=0, ub=None, name='L')
    y = mdl.continuous_var_dict(keys, lb=0, ub=None, name='y')
    z = mdl.continuous_var_dict(keys, lb=0, ub=None, name='z')
    s = mdl.continuous_var_dict(keys, lb=0, ub=None, name='s')

      
    # Node variables (w and a) stay the same
    w = mdl.binary_var_dict(N, name='w')
    a = mdl.continuous_var_dict(N, lb=0, name='a')
    
    
    
    #mdl.add_constraint(mdl.sum(w[i] for i in nodes) == 3, ctname='restriction')
    
    
    # CONSTRAINTS
    mdl.add_constraint(w[base] == 1, ctname='start_at_depot')
   
    
    
    # 
        
    for i in N:
        mdl.add_constraint(mdl.sum(x[i, j] for j in N if i!=j) == w[i], ctname=f'flow_out_{i}')
        mdl.add_constraint(mdl.sum(x[j, i] for j in N if i!=j) == w[i], ctname=f'flow_in_{i}')


               
    # --- Time Windows ---
        mdl.add_constraint(a[i] >= graph.nodes[i]['time_window'][0] * w[i]) # a_i >= e_i w_i
        mdl.add_constraint(a[i] <= graph.nodes[i]['time_window'][1] * w[i])
    
    
    
    for i in N:
        for j in N:
            if i!=j:

                mdl.add_constraint(L[i, j] >= graph[i][j]['distance'] * x[i, j])
                mdl.add_constraint(L[i, j] <= drone.speed_max * t[i, j])
                mdl.add_constraint(L[i, j] >= drone.speed_min * t[i, j])
                mdl.add_constraint(t[i, j] <= T_max * x[i, j])
        
                
                #t[i,j]**2 + ((z[i,j]-L[i,j])/2)**2 <= ((z[i,j]+L[i,j])/2)**2
            # --- SOCP Conic Constraints ---


                # 1. t^2 <= z * L
                #mdl.add_constraint(t[i, j]**2 <= z[i, j] * L[i, j])
                mdl.add_constraint(t[i,j]**2 + ((z[i,j]-L[i,j])/2)**2 <= ((z[i,j]+L[i,j])/2)**2)
                # 2. L^2 <= t * s
                #mdl.add_constraint(L[i, j]**2 <= t[i, j] * s[i, j])
                mdl.add_constraint(L[i,j]**2 + ((t[i,j]-s[i,j])/2)**2 <= ((t[i,j]+s[i,j])/2)**2)
                # 3. s^2 <= L * y
                #mdl.add_constraint(s[i, j]**2 <= L[i, j] * y[i, j])
                mdl.add_constraint(s[i,j]**2 + ((L[i,j]-y[i,j])/2)**2 <= ((L[i,j]+y[i,j])/2)**2)
                
                # 4. y <= v_max * t
                #mdl.add_constraint(y[i, j] <= (drone.speed_max**3) * t[i, j])
                # 5. z <= v_min * t
                #mdl.add_constraint(z[i, j] <= t[i, j] / drone.speed_min)


    # --- MTZ Subtour Elimination ---
    for i in N:
        for j in N:
            if i != j and i != 0:

                # a_j >= a_i + t_ij - T_max(1 - x_ij)
                mdl.add_constraint(a[j] >= a[i] + t[i, j] - T_max * (1 - x[i, j]), ctname=f'mtz_lb_{i}_{j}')
                # a_j <= a_i + t_ij + T_max(1 - x_ij)
                mdl.add_constraint(a[j] <= a[i] + t[i, j] + T_max * (1 - x[i, j]), ctname=f'mtz_ub_{i}_{j}')

    for j in N:
        if j != 0:
            mdl.add_constraint(a[j] >= t[0, j] - T_max * (1 - x[0, j]))
            mdl.add_constraint(a[j] <= t[0, j] + T_max * (1 - x[0, j]))
    


    max_energy =drone.max_energy
    total_energy = mdl.sum(drone.socp_energy_function(t[i, j], y[i, j], z[i, j]) for (i,j) in keys)
    mdl.add_constraint(total_energy <= max_energy, ctname='energy_budget')

    obj = mdl.sum(graph.nodes[i]['info_at_lowest'] * w[i] + graph.nodes[i]['info_slope'] * (a[i] - graph.nodes[i]['time_window'][0] * w[i]) for i in N)
    mdl.maximize(obj)
    
    return mdl, x


def build_model_splitted(graph, drone):
    
    
    mdl = Model(name='UAV_MISOCP_Routing')
    #mdl.parameters.mip.tolerances.mipgap = 0.001
    
    N = list(graph.nodes)
    base = drone.base
    T_max = drone.max_time 
    
    
    # VARIIABLES
    keys = []
    for i in N:
        for j in N:
            if i!=j:
                keys.append((i,j))
            
    x = mdl.binary_var_dict(keys, lb=0, ub=None, name='x')
    
    t = mdl.continuous_var_dict(keys, lb=0, ub=None, name='t')
    tt = mdl.continuous_var_dict(keys, lb=0, ub=None, name='tt') # loiter travel time
    L = mdl.continuous_var_dict(keys, lb=0, ub=None, name='L') # loiter distance
    y = mdl.continuous_var_dict(keys, lb=0, ub=None, name='y')
    z = mdl.continuous_var_dict(keys, lb=0, ub=None, name='z')
    s = mdl.continuous_var_dict(keys, lb=0, ub=None, name='s')

      
    # Node variables (w and a) stay the same
    w = mdl.binary_var_dict(N, name='w')
    a = mdl.continuous_var_dict(N, lb=0, name='a')
    
    
    
    #mdl.add_constraint(mdl.sum(w[i] for i in nodes) == 3, ctname='restriction')
    
    
    # CONSTRAINTS
    mdl.add_constraint(w[base] == 1, ctname='start_at_depot')
   
    
    for i in N:
        mdl.add_constraint(mdl.sum(x[i, j] for j in N if i!=j) == w[i], ctname=f'flow_out_{i}')
        mdl.add_constraint(mdl.sum(x[j, i] for j in N if i!=j) == w[i], ctname=f'flow_in_{i}')


               
    # --- Time Windows ---
        mdl.add_constraint(a[i] >= graph.nodes[i]['time_window'][0] * w[i]) # a_i >= e_i w_i
        mdl.add_constraint(a[i] <= graph.nodes[i]['time_window'][1] * w[i])
    
    
    
    for i in N:
        for j in N:
            if i!=j:
 
                mdl.add_constraint(L[i, j] <= drone.speed_max * tt[i, j])
                mdl.add_constraint(L[i, j] >= drone.speed_min * tt[i, j])
                
                mdl.add_constraint(graph[i][j]['distance'] <= drone.speed_max * t[i, j])
                
                mdl.add_constraint(t[i, j] + tt[i, j] <= T_max * x[i, j])
        
                
                #t[i,j]**2 + ((z[i,j]-L[i,j])/2)**2 <= ((z[i,j]+L[i,j])/2)**2
            # --- SOCP Conic Constraints ---


                # 1. t^2 <= z * L
                #mdl.add_constraint(t[i, j]**2 <= z[i, j] * L[i, j])
                mdl.add_constraint(tt[i,j]**2 + ((z[i,j]-L[i,j])/2)**2 <= ((z[i,j]+L[i,j])/2)**2)
                # 2. L^2 <= t * s
                #mdl.add_constraint(L[i, j]**2 <= t[i, j] * s[i, j])
                mdl.add_constraint(L[i,j]**2 + ((t[i,j]-s[i,j])/2)**2 <= ((t[i,j]+s[i,j])/2)**2)
                # 3. s^2 <= L * y
                #mdl.add_constraint(s[i, j]**2 <= L[i, j] * y[i, j])
                mdl.add_constraint(s[i,j]**2 + ((L[i,j]-y[i,j])/2)**2 <= ((L[i,j]+y[i,j])/2)**2)
                
                # 1. t^2 <= z * L
                #mdl.add_constraint(t[i, j]**2 <= z[i, j] * L[i, j])
                mdl.add_constraint(tt[i,j]**2 + ((z[i,j]-L[i,j])/2)**2 <= ((z[i,j]+L[i,j])/2)**2)
                # 2. L^2 <= t * s
                #mdl.add_constraint(L[i, j]**2 <= t[i, j] * s[i, j])
                mdl.add_constraint(L[i,j]**2 + ((t[i,j]-s[i,j])/2)**2 <= ((t[i,j]+s[i,j])/2)**2)
                # 3. s^2 <= L * y
                #mdl.add_constraint(s[i, j]**2 <= L[i, j] * y[i, j])
                mdl.add_constraint(s[i,j]**2 + ((L[i,j]-y[i,j])/2)**2 <= ((L[i,j]+y[i,j])/2)**2)
                
                # 4. y <= v_max * t
                #mdl.add_constraint(y[i, j] <= (drone.speed_max**3) * t[i, j])
                # 5. z <= v_min * t
                #mdl.add_constraint(z[i, j] <= t[i, j] / drone.speed_min)


    # --- MTZ Subtour Elimination ---
    for i in N:
        for j in N:
            if i != j and i != 0:

                # a_j >= a_i + t_ij - T_max(1 - x_ij)
                mdl.add_constraint(a[j] >= a[i] + t[i, j] + tt[i, j] - T_max * (1 - x[i, j]), ctname=f'mtz_lb_{i}_{j}')
                # a_j <= a_i + t_ij + T_max(1 - x_ij)
                mdl.add_constraint(a[j] <= a[i] + t[i, j] + tt[i, j] + T_max * (1 - x[i, j]), ctname=f'mtz_ub_{i}_{j}')

    for j in N:
        if j != 0:
            mdl.add_constraint(a[j] >= t[0, j] + tt[i, j] - T_max * (1 - x[0, j]))
            mdl.add_constraint(a[j] <= t[0, j] + tt[i, j] + T_max * (1 - x[0, j]))
    


    max_energy =drone.max_energy
    total_energy = mdl.sum(drone.socp_energy_function(t[i, j], y[i, j], z[i, j]) for (i,j) in keys)
    mdl.add_constraint(total_energy <= max_energy, ctname='energy_budget')

    obj = mdl.sum(graph.nodes[i]['info_at_lowest'] * w[i] + graph.nodes[i]['info_slope'] * (a[i] - graph.nodes[i]['time_window'][0] * w[i]) for i in N)
    mdl.maximize(obj)
    
    return mdl, x



def build_model_gurobi(graph, drone):
    # Initialize Gurobi Model
    mdl = gp.Model('UAV_MISOCP_Routing')
    # Force exact repeatability
    mdl.Params.Threads = 0      # Use all available cores
    mdl.Params.Seed = 1         
    mdl.Params.Method = 2       # Barrier (stable for SOCP)
    mdl.Params.NumericFocus = 3 # High precision
    mdl.Params.DualReductions = 0
    mdl.Params.InfUnbdInfo = 1
    mdl.Params.FeasibilityTol = 1e-9

    # Parameters
    N = list(graph.nodes)
    base = drone.base
    T_max = drone.max_time 
    speed_max = drone.speed_max
    speed_min = drone.speed_min
    max_energy = drone.max_energy

    # Decision Variables
    keys = [(i, j) for i in N for j in N if i != j]
    
    x = mdl.addVars(keys, vtype=GRB.BINARY, name='x')
    t = mdl.addVars(keys, lb=0, vtype=GRB.CONTINUOUS, name='t')
    L = mdl.addVars(keys, lb=0, vtype=GRB.CONTINUOUS, name='L')
    y = mdl.addVars(keys, lb=0, vtype=GRB.CONTINUOUS, name='y')
    z = mdl.addVars(keys, lb=0, vtype=GRB.CONTINUOUS, name='z')
    s = mdl.addVars(keys, lb=0, vtype=GRB.CONTINUOUS, name='s')
    
    #y = mdl.addVars(keys, lb=0, ub=1e6, vtype=GRB.CONTINUOUS, name='y')
    #z = mdl.addVars(keys, lb=0, ub=1e6, vtype=GRB.CONTINUOUS, name='z')
    #s = mdl.addVars(keys, lb=0, ub=1e6, vtype=GRB.CONTINUOUS, name='s')
    # Use tighter bounds based on your problem:
    #max_y = (speed_max**3) * T_max  # Maximum possible y value
    #y = mdl.addVars(keys, lb=0, ub=max_y, vtype=GRB.CONTINUOUS, name='y')

    #max_z = T_max / speed_min  # Maximum possible z value
    #z = mdl.addVars(keys, lb=0, ub=max_z, vtype=GRB.CONTINUOUS, name='z')

    #max_L = speed_max * T_max
    #L = mdl.addVars(keys, lb=0, ub=max_L, vtype=GRB.CONTINUOUS, name='L')
    # Similar for s
    #max_s = (max_L * max_y) ** 0.5  # sqrt(L * y)
    #s = mdl.addVars(keys, lb=0, ub=max_s, vtype=GRB.CONTINUOUS, name='s')

    w = mdl.addVars(N, vtype=GRB.BINARY, name='w')
    a = mdl.addVars(N, lb=0, vtype=GRB.CONTINUOUS, name='a')

    # Objective Function
    # Information = info_at_lowest * w_i + info_slope * (arrival_time - start_of_window * w_i)
    obj = gp.quicksum(
        graph.nodes[i]['info_at_lowest'] * w[i] + 
        graph.nodes[i]['info_slope'] * (a[i] - graph.nodes[i]['time_window'][0] * w[i]) 
        for i in N
    )
    mdl.setObjective(obj, GRB.MAXIMIZE)

    # --- Constraints ---
    
    # 1. Start at Depot
    mdl.addConstr(w[base] == 1, name='start_at_depot')

    # 2. Flow and Routing
    for i in N:
        mdl.addConstr(gp.quicksum(x[i, j] for j in N if i != j) == w[i], name=f'flow_out_{i}')
        mdl.addConstr(gp.quicksum(x[j, i] for j in N if i != j) == w[i], name=f'flow_in_{i}')  

    # 3. Time Windows
    for i in N:
        e_i, l_i = graph.nodes[i]['time_window']
        mdl.addConstr(a[i] >= e_i * w[i], name=f'tw_lb_{i}')
        mdl.addConstr(a[i] <= l_i * w[i], name=f'tw_ub_{i}')


    # 4. Distance, Speed, and SOCP Cones
    for i in N:
        for j in N:
            if i!=j:
                dist = graph[i][j]['distance']
                
                # Path and Speed Bounds
                mdl.addConstr(L[i, j] >= dist * x[i, j], name=f'dist_min_{i}_{j}')
                mdl.addConstr(L[i, j] <= speed_max * t[i, j], name=f'speed_max_bound_{i}_{j}')
                mdl.addConstr(L[i, j] >= speed_min * t[i, j], name=f'speed_min_bound_{i}_{j}')
                mdl.addConstr(t[i, j] <= T_max * x[i, j], name=f'time_bound_{i}_{j}')
                
                # Conic Constraints reformulated as x^2 <= y * z
                # 1. t^2 <= z * L
                mdl.addConstr(t[i, j] * t[i, j] <= z[i, j] * L[i, j], name=f'cone1_{i}_{j}')

                # 2. L^2 <= t * s
                mdl.addConstr(L[i, j] * L[i, j] <= t[i, j] * s[i, j], name=f'cone2_{i}_{j}')

                # 3. s^2 <= L * y
                mdl.addConstr(s[i, j] * s[i, j] <= L[i, j] * y[i, j], name=f'cone3_{i}_{j}')
                
                # 4. y <= v_max * t
                mdl.addConstr(y[i, j] <= (speed_max**3) * t[i, j], name=f'y_link_{i}_{j}')
                # 5. z <= v_min * t
                mdl.addConstr(z[i, j] <= t[i, j] / speed_min, name=f'z_link_{i}_{j}')


    # --- MTZ Subtour Elimination ---
    # Part 1: Transitions between non-base nodes
    for i in N:
        for j in N:
            if i != j and i != base:
                # a_j >= a_i + t_ij - T_max(1 - x_ij)
                mdl.addConstr(a[j] >= a[i] + t[i, j] - T_max * (1 - x[i, j]), name=f'mtz_lb_{i}_{j}')
                
                # a_j <= a_i + t_ij + T_max(1 - x_ij)
                mdl.addConstr(a[j] <= a[i] + t[i, j] + T_max * (1 - x[i, j]), name=f'mtz_ub_{i}_{j}')

    # Part 2: Transitions starting from the base
    for j in N:
        if j != base:
            # a_j >= t_base_j - T_max(1 - x_base_j)
            mdl.addConstr(a[j] >= t[base, j] - T_max * (1 - x[base, j]), name=f'base_mtz_lb_{j}')
            
            # a_j <= t_base_j + T_max(1 - x_base_j)
            mdl.addConstr(a[j] <= t[base, j] + T_max * (1 - x[base, j]), name=f'base_mtz_ub_{j}')



    # 7. Energy Budget
    # Assuming drone.socp_energy_function returns a linear combination of t, y, z
    total_energy = gp.quicksum(drone.socp_energy_function(t[i, j], y[i, j], z[i, j]) for (i, j) in keys)
    #total_energy = gp.quicksum(drone.c_1 * L[i,j]**3/t[i,j]**2 + drone.c_2*t[i,j]**2/L[i,j] for (i, j) in keys)
    mdl.addConstr(total_energy <= max_energy, name='energy_budget')
    #mdl.Params.NonConvex = 2
    # Update model to integrate variables/constraints
    mdl.update()
    # Update model to integrate variables/constraints


    # Print model statistics for verification
    print("="*70)
    print("MODEL STATISTICS")
    print("="*70)
    print(f"Number of variables: {mdl.NumVars}")
    print(f"  - Binary variables: {sum(1 for v in mdl.getVars() if v.VType == GRB.BINARY)}")
    print(f"  - Continuous variables: {sum(1 for v in mdl.getVars() if v.VType == GRB.CONTINUOUS)}")
    print(f"Number of linear constraints: {mdl.NumConstrs}")
    print(f"Number of quadratic constraints: {mdl.NumQConstrs}")
    print(f"Number of nodes: {len(N)}")
    print(f"Number of edges: {len(keys)}")
    print("\nDrone parameters:")
    print(f"  speed_max = {speed_max}")
    print(f"  speed_min = {speed_min}")
    print(f"  T_max = {T_max}")
    print(f"  max_energy = {max_energy}")
    print("\nCalculated bounds:")
    #print(f"  max_L = {max_L:.2e}")
    #print(f"  max_y = {max_y:.2e}")
    #print(f"  max_z = {max_z:.2e}")
    #print(f"  max_s = {max_s:.2e}")
    print("="*70)



    return mdl

def solve_gurobi_benchmark(graph, drone, time_limit=600):
    """
    Similar to solve_exact_benchmark_subgraph but for Gurobi.
    """
    import pandas as pd
    import time
    
    max_energy = drone.max_energy
    base = drone.base
    
    # 1. Build Model
    mdl = build_model_gurobi(graph, drone)
    
    # 2. Set Parameters
    mdl.Params.TimeLimit = time_limit
    mdl.Params.MIPFocus = 1  

    # 3. Solve
    start_t = time.time()
    mdl.optimize()
    solve_time = time.time() - start_t
    
    # 4. Handle Result
    results_base = {
        "feasible": False,
        "status": mdl.Status,
        "objective": 0,
        "sequence": [base],
        "arrival_times": {i: 0 for i in graph.nodes},
        "solve_time": solve_time,
        "gap": 0,
        "df": pd.DataFrame()
    }

    if mdl.SolCount == 0:
        print(f"\n*** Solver Status: {mdl.Status} --- No Solution Found ***")
        return results_base, mdl

    # 5. Extract Variable Values & Construct Tour
    active_arcs = []
    for (i, j) in [(i,j) for i in graph.nodes for j in graph.nodes if i!=j]:
        var = mdl.getVarByName(f'x[{i},{j}]')
        if var and var.X > 0.5:
            active_arcs.append((i, j))
    
    # Construct sequence
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
        if len(seq) > len(graph.nodes) + 1:
            break

    # 6. Detailed Tour Analysis Table
    tour_data = []
    total_t, total_l, total_e, total_e_2 = 0, 0, 0, 0
    arrival_times = {i: mdl.getVarByName(f'a[{i}]').X for i in graph.nodes}
    
    for idx in range(len(seq) - 1):
        i, j = seq[idx], seq[idx+1]
        
        t_ij = mdl.getVarByName(f't[{i},{j}]').X
        L_ij = mdl.getVarByName(f'L[{i},{j}]').X
        d_ij = graph[i][j]['distance']
        y_ij = mdl.getVarByName(f'y[{i},{j}]').X
        z_ij = mdl.getVarByName(f'z[{i},{j}]').X
        
        v_ij = L_ij / t_ij if t_ij > 1e-6 else 0
        loiter_time = max(0, t_ij - (d_ij / v_ij)) if v_ij > 1e-6 else 0
        energy_ij = drone.energy_function(v_ij, L_ij) if v_ij > 1e-6 else 0
        energy_ij_2 = drone.socp_energy_function(t_ij, y_ij, z_ij)
        energy_pct = (energy_ij / max_energy) * 100
         
        tour_data.append({
            "Edge": f"{i}->{j}",
            "Total Time (s)": round(t_ij, 2),
            "Loiter Time (s)": round(loiter_time, 2),
            "Speed (m/s)": round(v_ij, 2),
            "Model Energy (J)": round(energy_ij_2, 2),
            "Real Energy (J)": round(energy_ij, 2), 
            "Energy %": round(energy_pct, 4),
            "Arrival Time (s)": round(arrival_times[j], 2),
            "Time Window": f"[{graph.nodes[j]['time_window'][0]:.1f}, {graph.nodes[j]['time_window'][1]:.1f}]",
            "Slope": graph.nodes[j]['info_slope'],
        })
        
        total_t += t_ij
        total_l += loiter_time
        total_e += energy_ij
        total_e_2 += energy_ij_2

    df = pd.DataFrame(tour_data)
    if not df.empty:
        summary_row = pd.DataFrame([{
            "Edge": "TOTAL TOUR",
            "Total Time (s)": round(total_t, 2),
            "Loiter Time (s)": round(total_l, 2),
            "Model Energy (J)": round(total_e_2, 2),
            "Real Energy (J)": round(total_e, 2),
            "Energy %": round((total_e / max_energy) * 100, 4)
        }])
        df = pd.concat([df, summary_row], ignore_index=True)

    print("\n--- Gurobi Solver Report ---")
    print(df.to_string(index=False))
    print("-" * 60)
    print(f"Objective Value:      {mdl.ObjVal:.2f}")
    print(f"MIP Gap:              {mdl.MIPGap:.4%}")
    print(f"Tour Sequence:        {'-'.join(map(str, seq))}")
    print(f"Solve Time:           {solve_time:.2f} seconds")

    return {
        "feasible": True,
        "df": df,
        "objective": mdl.ObjVal,
        "gap": mdl.MIPGap,
        "sequence": seq,
        "arrival_times": arrival_times,
        "solve_time": solve_time,
    }, mdl