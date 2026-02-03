
# Note that, in PD, PE, and PT methods, the vehicle is assumed to
# be traveling at energy minimizing speed within the speed limits
# in ‘‘score per unit’’ calculations.

# TODO: initial_tour input and output consistency for the calls from environment_real
# TODO: Analysis 1: feasibility rate, Analysis 2: comparisions of the methods
from uav_routing.solver.socp import Solver
from uav_routing.environment.graph import directed_cycle, nearest_neighbors_cycle

from itertools import permutations
import pandas as pd
import random
import csv
import ast

import networkx as nx
import matplotlib.pyplot as plt


def get_top_k(df, k):
    # Get the 5 highest scoring feasible tours
    top_k = df[df['feasible'] == True].nlargest(k, 'objective')
    return top_k

def failure_reason_statistics(path):
    "Reads the detailed results file and prints failure reason statistics."
    df = pd.read_csv(path)
    print(df['failure_reason'].value_counts())
    print(df['failure_reason'].value_counts(normalize=True) * 100)
    
def best_tour():
    df = pd.read_csv("tour_results.csv")

    # Find the best feasible tour
    best_tour = df[df['feasible'] == True].sort_values(by='objective', ascending=False).iloc[0]
    print(f"Winner: {best_tour['sequence']} with Objective {best_tour['objective']}")



def run_detailed_experiment(drone, graph, k):
    filename = f"results_k{k}_detailed.csv"
    headers = ['sequence', 'feasible', 'objective', 'total_energy', 'total_dist', 'failure_reason', 'arrival_times']
    
    targets = [n for n in graph.nodes if n != drone.base]
    
    with open(filename, mode='w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        
        for combo in permutations(targets, k-1):
            tour_nodes = [drone.base] + list(combo)
            tour_structure = directed_cycle(tour_nodes, graph)
            
            # Calculate simple metrics first
            d_total = sum(graph.edges[e]['distance'] for e in tour_structure.edges)
            
            solver = Solver(tour_structure, graph, drone)
            res = solver.get_tour_data()
            
            reason = "None"
            if not res.feasible:
                reason = solver.get_failure_reason() # Call our new diagnostic
            
            writer.writerow({
                'sequence': "-".join(map(str, tour_nodes)),
                'feasible': res.feasible,
                'objective': res.objective if res.feasible else 0,
                'total_energy': res.total_energy if res.feasible else 0,
                'total_dist': d_total,
                'failure_reason': reason,
                'arrival_times': res.arrival_times if res.feasible else None,
            })



# ---------------------------- Tours ----------------------------


# 1. Nearest neighbor-based tour (T1)

# "We will run an optimization loop with 2-opt and swaps, the Nearest Neighbor path 
# is an excellent starting point (initial solution). It is usually much better than 
# a random path, meaning your SOCP will start with a relatively high 'information 
# value', helping your optimizer converge faster."
def tour_with_nearest_neighbors(graph, drone, df, length=4):
    """
    Constructs a feasible tour of length 'length' starting from and ending at the base node.
    The next node is selected based on its proximity to the latest node in the sequence.

    :param G: 
    :param drone: 
    :param length:
    :return: 
    :rtype: tuple[Graph, Any | None]
    """
    # 1. Filter for feasible tours only to act as our 'map'
    df_copy = df[df['feasible'] == True].copy()
    
    if df_copy.empty:
        print("No feasible tours found in the dataframe.")
        return None

    current = drone.base
    nn_sequence = [current]
    
    # 2. Iteratively build the tour
    while len(nn_sequence) < length:
        step_index = len(nn_sequence) # Next position in the sequence
        
        # Get all nodes that could feasibly follow the current path
        candidates = set()
        for seq in df_copy['sequence']:
            nodes = seq.split('-')
            if len(nodes) > step_index:
                candidates.add(int(nodes[step_index]))
        
        if not candidates:
            print(f"No feasible neighbors available after node {current}.")
            break
            
        # 3. Pick the NEAREST neighbor among the FEASIBLE candidates
        next_node = min(candidates, key=lambda n: graph.edges[(current, n)]['distance'])
        
        # 4. Update the path and filter the dataframe for the next step
        nn_sequence.append(next_node)
        current = next_node
        
        prefix = "-".join(map(str, nn_sequence))
        df_copy = df_copy[df_copy['sequence'].str.startswith(prefix)]

    # Since we strictly followed feasible branches, df_copy will contain the result
    if not df_copy.empty:
        res = df_copy.iloc[0]
        print(f"Feasible NN Tour Found: {'-'.join(map(str, nn_sequence))}")
        print(f"Objective: {res['objective']:.2f} | Distance: {res['total_dist']:.2f}")
        return res
    
    return nn_sequence


# 2. Consumption-based tour (T2)
def tour_with_max_ratio(graph, drone, df, length=4):
    """
        Builds a feasible tour of length 'length' as follows: Starts from the base node. 
        Picks the next node as the node maximizing the ratio info/capacity until the 
        length of tour reaches 'length'.  
        capacity: max(travel_time/campaign_time, energy/max_energy)
    """

    depot = drone.base
    tour = [depot]
    current_node = depot
    
    # We filter for feasible tours only to ensure the ratios are valid
    df_feasible = df[df['feasible'] == True].copy()
    while len(tour) < length:
        best_node = None
        max_ratio = -1
        
        # Candidates are nodes not yet in the tour
        visited = set(tour)
        candidates = [n for n in graph.nodes if n not in visited]
        
        for v in candidates:
            # Check the dataframe for a tour that starts with [current_path] + [v]
            potential_prefix = "-".join(map(str, tour + [v]))
            
            # Find all tours in the DF that start with this prefix
            matches = df_feasible[df_feasible['sequence'].str.startswith(potential_prefix)]
            if matches.empty:
                continue
                
            # We take the first match to get the consumption data for this leg
            match = matches.iloc[0]
        
            # 1. Get Actual Consumption from SOCP
            # (Note: Using total_energy of the tour as the consumption proxy for that leg)
            energy_val = match['total_energy']
            objective_val = match['objective']
            
            # 2. Capacity: max(Time_Ratio, Energy_Ratio)
            arrival_times_raw = match['arrival_times']
            if isinstance(arrival_times_raw, str):
                arrival_times = ast.literal_eval(arrival_times_raw)
            else:
                arrival_times = arrival_times_raw

            # Now you can safely access the value for node v
            # Note: Ensure v is the same type as the keys (string vs int)
            arrival_time = arrival_times.get(str(v), arrival_times.get(v, 0))
            capacity_cost = max(energy_val / drone.max_energy, arrival_time / drone.max_time)
            
            # 3. Ratio = Information / Capacity
            # Using the objective (reward) from the SOCP result
            ratio = objective_val / capacity_cost if capacity_cost > 0 else 0
            
            
            if ratio > max_ratio:
                max_ratio = ratio
                best_node = v
        
        if best_node is None:
            print("Warning: No feasible next node found in the dataframe.")
            break
            
        tour.append(best_node)
        
    return tour



# 3. Random tour (T3)
def random_tour(df):
    
    # Filter for only feasible tours
    feasible_df = df[df['feasible'] == True]
    
    if feasible_df.empty:
        print("No feasible tours found in the file.")
        return None
    # Sample 1 random row
    random_tour = feasible_df.sample(n=1).iloc[0]
    return random_tour



# 4. Random then nearesst method: 
def random_then_nn_tour(graph, drone, df, length=4):
    """
    Builds a tour by picking a random first-jump and then picking 
    the nearest available node that maintains feasibility according to the DF.
    """
    # Start only with feasible results
    df_copy = df[df['feasible'] == True].copy()
    
    if df_copy.empty:
        print("No feasible tours found in the dataframe.")
        return None

    depot = drone.base
    tour = [depot]
    
    # 1. Step: Picking the Second Node (First target)
    # Extract the second node from all feasible sequences
    valid_second_nodes = set()
    for seq in df_copy['sequence']:
        nodes = seq.split('-')
        valid_second_nodes.add(int(nodes[1]))
    
    first_target = random.choice(list(valid_second_nodes))
    tour.append(first_target)
    
    # Update df_copy: Keep only tours starting with "depot-first_target"
    prefix = f"{depot}-{first_target}"
    df_copy = df_copy[df_copy['sequence'].str.startswith(prefix)]
    
    # 2. Steps: Picking 3rd, 4th, ... nodes greedily
    while len(tour) < length:
        current_node = tour[-1]
        step_index = len(tour) # If tour is [0, 5], next index is 2
        
        # Extract all possible candidates for the NEXT position from current df_copy
        candidates = set()
        for seq in df_copy['sequence']:
            nodes = seq.split('-')
            if len(nodes) > step_index:
                candidates.add(int(nodes[step_index]))
        
        if not candidates:
            print(f"No further feasible branches found after node {current_node}.")
            break
            
        # Pick the nearest one among these feasible candidates
        next_node = min(candidates, key=lambda n: graph.edges[(current_node, n)]['distance'])
        
        # Add to tour and filter DF again
        tour.append(next_node)
        prefix = "-".join(map(str, tour))
        df_copy = df_copy[df_copy['sequence'].str.startswith(prefix)]

    # Return the final row from the dataframe
    if not df_copy.empty:
        return df_copy.iloc[0]
    return None


# 5. Longest tour with the highest cummulative max ratio 
def longest_max_ratio(graph, drone):
    
    max_energy = drone.max_energy
    max_time = drone.max_time
    base = drone.base
    tour_energy = 0
    best_ratio = 0 
    
    tour = [base]
    remaining_nodes = [node for node in graph.nodes if node != drone.base]

    
    while tour_energy <= max_energy:
        
        
        for node in remaining_nodes:
        
            tour.append(node)
            tour_graph = directed_cycle(tour, graph)
            solver = Solver(tour_graph, graph, drone)
            if solver.solution:
                tourdata = solver.get_tour_data()
                #arrivals = {n: solver.solution.get_value(solver.var_arrival[n]) 
                #            for n in solver.tour_nodes}
            else:
                continue
            
            energy_consumption = tourdata.total_energy / max_energy
            time_consumption = tourdata.arrival_times[base] / max_time
            capacity_cost = max(energy_consumption, time_consumption)
            
            # 3. Ratio = Information / Capacity
            # Using the objective (reward) from the SOCP result
            ratio = tourdata.objective / capacity_cost if capacity_cost > 0 else 0
            
            if ratio > best_ratio:
                best_ratio = ratio
                best_tour = tour
                tour_energy = tourdata.total_energy
                
            else:
                tour.pop(node)
   
        remaining_nodes = remaining_nodes - tour
    
        
    return best_tour





def run_strategy_comparison(graph, drone, df, length=4):
    # 1. Collect results from each function
    t1 = tour_with_nearest_neighbors(graph, drone, df, length)
    
    t2_nodes = tour_with_max_ratio(graph, drone, df, length) # Updated to your logic
    t2_seq = "-".join(map(str, t2_nodes))
    t2 = df[df['sequence'] == t2_seq].iloc[0] if not df[df['sequence'] == t2_seq].empty else None
    
    t3 = random_tour(df) # Logic from your T3
    t4 = random_then_nn_tour(graph, drone, df, length)
    
    # 2. Build the Comparison DataFrame
    results = []
    for method_name, res in zip(["T1 (NN)", "T2 (Ratio)", "T3 (Random)", "T4 (Vetted)"], [t1, t2, t3, t4]):
        if res is not None:
            results.append({
                "Strategy": method_name,
                "Sequence": res['sequence'],
                "Objective": res['objective'],
                "Total Dist": res['total_dist'],
                "Energy": res['total_energy'],

            })
            
    return pd.DataFrame(results)


# Execute
#comparison_df = run_strategy_comparison(df, graph, drone, length=4)
#print(comparison_df.to_string(index=False))



# 6. Score per unit distance method (PD): The next node is selected
#    as the node which has the largest ratio per unit distance
#    when it follows the latest node in the sequence.

# 7. Score per unit energy consumption method (PE): The next node
#    is selected as the node which has the largest score per
#    unit energy consumption when it follows latest node in the
#    sequence.

# 8. Score per unit time method (PT): The next node is selected
#    as the node which has the largest score per unit travel time
#    when it follows the latest node in the sequence.





