import networkx as nx



def distance_to_depot_feasibility(graph, drone):
    impossible_nodes = set()
    e_meter = drone.energy_per_meter()
    for i in graph.nodes:
        if i == 0: 
            continue
        
        # Round trip: 0 -> i -> 0
        dist_0i = graph.edges[(0, i)]['distance']
        dist_i0 = graph.edges[(i, 0)]['distance']
        round_trip_energy = (dist_0i + dist_i0) * e_meter # min energy for the distance

        if round_trip_energy > drone.max_energy:
            impossible_nodes.add(i)
    return impossible_nodes


def energy_feasibility(graph, drone):
    """To enhance computational efficiency, we perform a reachability analysis 
    based on the UAV's energy-optimal speed. An arc $(i, j)$ is pruned from 
    the decision space if the minimum energy required for the sequence 
    $0 \to i \to j \to 0$ exceeds the total energy budget $\mathcal{E}_{\max}$. 
    This preprocessing significantly reduces the number of binary variables 
    $x_{ij}$ and tightens the linear programming relaxation of the MISOCP model."""
    impossible_arcs = set()
    e_meter = drone.energy_per_meter()
    
    # since distance is symmetric
    for edge in graph.edges:
        if drone.base not in edge:
            i, j = edge[0], edge[1]
            
            
            # Energy for: Depot(0) -> i -> j -> Depot(0). Note that distances are Euclidean.
            mission_dist = (graph.edges[(0, i)]['distance'] 
                          + graph.edges[(i, j)]['distance'] 
                          + graph.edges[(j, 0)]['distance'])
            
            # min energy for the distance
            if (mission_dist * e_meter) > drone.max_energy:
                impossible_arcs.add((i, j))
                impossible_arcs.add((j, i))
                
    return impossible_arcs



def time_window_feasibility(graph, drone):
    """
    An arc $(i, j)$ is only feasible if the UAV can arrive at $i$ at its 
    earliest time $e_i$, travel to $j$, and arrive before $j$'s latest time 
    $\ell_j$.Logic: $e_i + t_{ij}^{min} \le \ell_j$Corridor Logic: $t_{0i}^{min} 
    + t_{ij}^{min} + t_{j0}^{min} \le T_{max}$In your code, $t_{ij}^{min}$ is 
    simply $d_{ij} / v_{max}$.
    """
    base = drone.base
    direct_time_impossible = set()
    total_time_impossible = set()
    
    for edge in graph.edges:
        i, j = edge[0], edge[1]
        
        # Min travel time between i and j. Symmetric.
        t_min_ij = graph.edges[(i, j)]['distance'] / drone.speed_max
        
        # Direct Time Window feasibility (Earliest i + travel > Latest j)
        if graph.nodes[i]['time_window'][0] + t_min_ij > graph.nodes[j]['time_window'][1]:
            direct_time_impossible.add((i, j))
        if graph.nodes[j]['time_window'][0] + t_min_ij > graph.nodes[i]['time_window'][1]:
            direct_time_impossible.add((j, i))
            continue
        
        # 4. Total Mission Time feasibility (0 -> i -> j -> 0). Symmetric.
        if base not in edge:
            t_min_0i = graph.edges[(base, i)]['distance'] / drone.speed_max
            t_min_j0 = graph.edges[(j, base)]['distance'] / drone.speed_max
            if (t_min_0i + t_min_ij + t_min_j0) > drone.max_time:
                total_time_impossible.add((i, j))
                total_time_impossible.add((j, i))
                
                    
    return direct_time_impossible, total_time_impossible



def prune_and_report(graph, drone):
    "Calls pruning functions, prints a report, returns feasible node and edge sets for the model."
    
    base = drone.base
    
    # prune nodes
    prune_nodes = distance_to_depot_feasibility(graph, drone)
    feasible_nodes_1 = set(graph.nodes) - prune_nodes
    
    # prune edges
    energy_prune_edges = energy_feasibility(graph, drone)
    direct_time_prune, total_time_prune = time_window_feasibility(graph, drone)
    infeasible_arcs = list(energy_prune_edges) + list(direct_time_prune) + list(total_time_prune)
    
    # all arcs
    all_arcs = set()
    for edge in graph.edges:
        all_arcs.add(edge)
        all_arcs.add((edge[1], edge[0]))
    
    # feasible arcs
    first_feasible_arcs = {arc for arc in all_arcs if arc not in set(infeasible_arcs)}
    
    G = nx.DiGraph(list(first_feasible_arcs))
    removed = True
    while removed:
        removed = False
        nodes_to_remove = [n for n in G.nodes 
                        if (G.out_degree(n) == 0 or G.in_degree(n) == 0) and n != base]
        
        if nodes_to_remove:
            G.remove_nodes_from(nodes_to_remove)
            removed = True
    
    # Check if base node is still connected
    if base not in G.nodes:
        raise TypeError("Base node was pruned away. Problem is infeasible.")
    
    if G.out_degree(base) == 0 or G.in_degree(base) == 0:
        raise TypeError(f"Base node has no feasible outgoing or incoming edges. "
                       f"Out-degree: {G.out_degree(base)}, In-degree: {G.in_degree(base)}. "
                       f"Consider relaxing pruning constraints (time, energy, or distance).")

    feasible_arcs = {edge for edge in G.edges}
    feasible_nodes = {node for node in G.nodes if node in feasible_nodes_1}
            
    summary = {"distance_to_depot": {'rule':"min [E(0, i) + E(i,O)] > E_max",
                                        'pruned': len(prune_nodes)
                    },
                "energy_edges": {'rule':"min [E(0, i) + E(i,j) + E(j,O)] > E_max",
                                    'pruned': len(energy_prune_edges)
                    },
                "direct_time": {'rule':"e_i + min t_ij > l_j",
                                        'pruned': len(direct_time_prune)  
                    },
                "total_time": {'rule':"[d_0i + d_ij + d_j0] \ v_opt > T_max",
                                        'pruned': len(total_time_prune)
                    }
        }
    
    # report 
    prune_report(summary, len(graph.nodes), len(feasible_nodes), len(all_arcs), len(feasible_arcs))
    
    return feasible_nodes, feasible_arcs, G


def prune_report(summary, len_all_nodes, len_feasible_nodes, len_all_arcs, len_feasible_arcs):
    
    
    # variable report
    node_dict = {'total': len_all_nodes, 'zeros': len_all_nodes - len_feasible_nodes}
    edge_dict = {'total': len_all_arcs, 'zeros': len_all_arcs - len_feasible_arcs}
    
    stats = {
        'w': node_dict,
        'a': node_dict,
        'x': edge_dict,
        't': edge_dict,
        'L': edge_dict,
        'y': edge_dict,
        'z': edge_dict,
        's': edge_dict,
    }

    total_vars = sum(value['total'] for value in stats.values())
    zero_fixed = sum(value['zeros'] for value in stats.values())

    # start printing a table
    print(f"\n{'='*70}") # hline
    print(f"PRE-SOLVE VARIABLE REPORT: UAV_MISOCP_Routing") # title
    print(f"{'='*70}") # hline
    
    # summarize prunning functions
    print(f"{'Prunning Type':<45} | {'Pruned':<10} | {'%'}") # columns
    print("-" * 70) # hline
    for key, value in summary.items():
        l = len_all_nodes if key == 'distance_to_depot' else len_all_arcs
        perc = (value['pruned'] / l * 100)  # percentage column
        print(f"{value['rule']:<45} | {value['pruned']:<10} | {perc:.1f}%") # print row
    print("-" * 70)
    print(f"num of nodes {len_all_nodes:<10} | feasible {len_feasible_nodes:<10} | {len_feasible_nodes/len_all_nodes*100:.1f}%")
    print(f"num of arcs {len_all_arcs:<11} | feasible {len_feasible_arcs:<10} | {len_feasible_arcs/len_all_arcs*100:.1f}%")
    print(f"\n{'='*70}") # hline
    
    # summarize variable fixing
    print(f"\n{'='*60}") # hline
    print(f"{'Var Type':<15} | {'Total':<10} | {'Fixed to Zero':<15} | {'% Zero'}") # columns
    print("-" * 60) # hline
    for g, s in stats.items():
        perc = (s['zeros'] / s['total'] * 100) if s['total'] > 0 else 0 # percentage column
        print(f"{g:<15} | {s['total']:<10} | {s['zeros']:<15} | {perc:.1f}%") # print row
    print("-" * 60)
    print(f"TOTAL MODEL:    | {total_vars:<10} | {zero_fixed:<15} | {(zero_fixed/total_vars*100):.1f}%")
    print(f"{'='*60}\n")