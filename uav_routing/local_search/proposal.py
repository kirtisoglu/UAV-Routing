import random
import networkx as nx
from typing import List, Set, Tuple


# TODO : Tour should be a list. Stop using nx.DiGraph for route representation.
# TODO : Define a dictionary of operators instead of hardcoding them. Merge two proposal functions.


class EdgeCaseError(Exception):
    "Raised when an edge case error occurs"


def route_to_nx(route: List[int]) -> nx.DiGraph:
    """Converts an ordered list of node IDs into a NetworkX DiGraph cycle."""
    G = nx.DiGraph()
    G.add_nodes_from(route)
    if len(route) > 1:
        edges = [(route[i], route[i+1]) for i in range(len(route)-1)]
        edges.append((route[-1], route[0]))
        G.add_edges_from(edges)
    return G



# ------------ Main Proposal Logic ---------------

def universal_proposal(state):
    """
    A standard proposal wrapper that can be used by SA, Ascent, and Tilt.
    We don't use perturbation or a Tabu set here so SA/Ascent can explore freely.
    Includes remove_random_node rather than perturbation.
    """
    current_route = state.solver.tour_nodes 
    available_nodes = list(set(state.graph.nodes) - set(current_route))
 
    l = len(current_route)
    
    if l == 0 or l>len(state.graph.nodes):
        raise EdgeCaseError("Current route is empty, cannot propose a new state.\n"
                            f"Route: {current_route}")
    if l==1:
        methods = [add_random_node]
    elif l == 2:
        methods = [add_random_node, replace_random_node, remove_random_node]
    elif l == 3:
        methods = [add_random_node, replace_random_node, swap_two_nodes, remove_random_node]
    else:
        methods = [add_random_node, swap_two_nodes, replace_random_node, swap_two_opt, remove_random_node]
    
    # if tour is an Hamiltonian cycle, cannot add/replace
    if not available_nodes:
        methods = [swap_two_nodes, swap_two_opt, remove_random_node]
    
    chosen = random.choice(methods)
    
    # 3. Apply and return a new state
    new_route = chosen(current_route, available_nodes)
    return state.flip(route_to_nx(new_route))


def random_flip_with_tabu(state, tabu_set: set):
    """
    Core proposal function. Uses state.solver.tour_nodes for high-performance
    access to the current ordered sequence.
    """
    # Use the pre-ordered nodes from the solver to save CPU time
    current_route = state.solver.tour_nodes 
    l = len(current_route)
    
    if l == 0 or l>len(state.graph.nodes):
        raise EdgeCaseError("Current route is empty, cannot propose a new state.\n"
                            f"Route: {current_route}")
        
    available_nodes = list(set(state.graph.nodes) - set(current_route) - tabu_set)

    # Logic for selecting methods
    if l==1:
        methods = [add_random_node]
    elif l == 2:
        methods = [add_random_node, replace_random_node]
    elif l == 3:
        methods = [add_random_node, replace_random_node, swap_two_nodes]
    else:
        methods = [add_random_node, replace_random_node, swap_two_nodes, swap_two_opt]

    # Filter based on availability
    if not available_nodes:
        methods = [swap_two_nodes, swap_two_opt]
    
    chosen = random.choice(methods)
    
    # Apply method (now passing the route list for speed where possible)
    new_route = chosen(current_route, available_nodes)
    
    # Convert list back to graph only once
    return state.flip(route_to_nx(new_route))



def perturb_state(state, k_remove: int, current_iteration: int, tabu_tenure: int):
    """The 'Shake' operation for ILS."""
    route = list(state.solver.tour_nodes)
    base = state.drone.base
    
    possibles = [n for n in route if n != base]
    num_to_remove = min(k_remove, len(possibles))
    nodes_to_remove = random.sample(possibles, num_to_remove)
    
    new_route = [n for n in route if n not in nodes_to_remove]
    new_tabu_entries = {n: current_iteration + tabu_tenure for n in nodes_to_remove}
    
    return state.flip(route_to_nx(new_route)), new_tabu_entries




# --------------- Operators (List-Based for Speed) ------------------

def add_random_node(route: List[int], complement: List[int]) -> List[int]:
    "Adds a random node from the complement into the route."
    new_node = random.choice(complement)
    new_route = list(route)
    # Insert anywhere after the base (index 0)
    if len(new_route) == 1:
        insertion_point = 1
    else:
        insertion_point = random.randint(1, len(new_route))
    new_route.insert(insertion_point, new_node)
    return new_route

def remove_random_node(route: List[int], complement: List[int], base: int) -> List[int]:
    "Removes a random node from the route (excluding the base)."
    possibles = [n for n in route if n != base]
    removed = random.choice(possibles)
    return [n for n in route if n != removed]

def swap_two_nodes(route: List[int], complement: List[int]) -> List[int]:
    "Swaps two nodes in the route (excluding the base)."
    new_route = list(route)
    idx1, idx2 = random.sample(range(1, len(new_route)), 2)
    new_route[idx1], new_route[idx2] = new_route[idx2], new_route[idx1]
    return new_route

def replace_random_node(route: List[int], complement: List[int]) -> List[int]:
    "Replaces a random node in the route with one from the complement."
    new_node = random.choice(complement)
    new_route = list(route)
    idx = random.randint(1, len(new_route)-1)
    new_route[idx] = new_node
    return new_route

def swap_two_opt(route: List[int], complement: List[int]) -> List[int]:
    "Performs a 2-opt swap on two non-adjacent edges in the route uniformly."
    max_attempts = 5
    for _ in range(max_attempts):
        i, j = sorted(random.sample(range(1, len(route)), 2))
        if j - i >= 2:
            # Reverses the segment between i and j
            return route[:i] + route[i:j+1][::-1] + route[j+1:]
            
    return route # Fallback if no valid pair found