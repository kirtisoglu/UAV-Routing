# operations.py

"""
Order preserved list operations to perform ...

Key functionalities include:

- swap_two_nodes
- swap_two_opt
- add_random_node
- remove_random_node
- replace_random_node

Last Updated: 

"""

import random
import networkx as nx

from typing import Callable, List, FrozenSet
from dataclasses import dataclass, field
from functools import partial
from uav_routing.state import State


@dataclass
class Flows:
    nodes_out: frozenset = field(default_factory=frozenset)
    nodes_in: frozenset = field(default_factory=frozenset)
    edges_out: frozenset =  field(default_factory=frozenset)
    edges_in: frozenset = field(default_factory=frozenset)


def random_flip(state: State
    ) -> State:
    """
    _summary_

    Example usage:
    
    Args:
        state (State): _description_
        method (Callable): _description_

    Returns:
        _type_: _description_
    """

    base=state.base 
    tour=state.tour
    l = len(tour)
    n = len(state.nodes)
    
    # be sure length is at least 1
    if l == 0:
        raise print("Length of tour cannot be 0.\n"
                    f"The current tour is f{tour}")
    elif l == 1:
        methods=[add_random_node]
    elif l == 2:
        methods = [remove_random_node, add_random_node, replace_random_node]
    elif l == 3: 
        methods=[add_random_node, remove_random_node, swap_two_nodes, replace_random_node]
    else: 
        if l < n:
            methods = [add_random_node, remove_random_node, swap_two_nodes, replace_random_node, swap_two_opt]
        else: 
            methods = [remove_random_node, swap_two_nodes, swap_two_opt]
    
    chosen = random.choice(methods)
    
    print("methods", methods)
    print("chosen method", chosen)
    print("tour nodes before operation", tour.nodes)
    print("tour edges before operation", tour.edges)
    
    new_tour, complement, flows =chosen(tour, state.complement, state.base)
    
    print("tour nodes after", new_tour.nodes)
    print("tour edges after", new_tour.edges)
    print("flows", flows)
    
    return state.flip(flows, new_tour, complement)


# TODO: keep tally to track methods and choosen methods


# ---------- Operations


def replace_random_node(cycle, complement, base): 
    # perform whenever 1 < len(cycle) < n 
    node_from_complement = random.choice(complement)
    complement.remove(node_from_complement)
    # random node from tour other than base and destination
    possibles = [node for node in cycle.nodes if node != base]
    random_node = random.choice(possibles)
    mapping = {random_node: node_from_complement}
    C = nx.relabel_nodes(cycle, mapping)
    complement.append(random_node)
    flows= Flows(nodes_out=frozenset({random_node}),
                 nodes_in=frozenset({node_from_complement}),
                 edges_out=frozenset({e for e in cycle.edges if random_node in e}),
                 edges_in=frozenset({e for e in C.edges if node_from_complement in e})
                )
    return C, complement, flows


def remove_random_node(cycle, complement, base):
    # perform when 2 <= l
    random_edge = random.choice(list(cycle.edges))
    print("random edge and the remove func", random_edge)
    n1, n2 = random_edge[0], random_edge[1]
    if n2 != base:
        removed = n2
        print("n2", n2)
        print("removed", removed)
        succ = list(cycle.successors(removed))[0]
        cycle_minor = nx.contracted_nodes(cycle, n1, n2, self_loops=False, copy=True)
        edge_in = (n1, succ)
    else:
        removed = n1
        print("n1", n1)
        print("removed", removed)
        pred =list(cycle.predecessors(removed))[0]
        cycle_minor = nx.contracted_nodes(cycle, n2, n1, self_loops=False, copy=True)
        edge_in = (pred ,n2)
    complement.append(removed)
    flows= Flows(nodes_out=frozenset({removed}),
                 nodes_in=frozenset(set()),
                 edges_out=frozenset({random_edge}),
                 edges_in=frozenset({edge_in})
                )
    return cycle_minor, complement, flows



def add_random_node(cycle, complement, base):
    # perform when l <= n-1
    cyc = cycle.copy()
    random_node = random.choice(complement)
    complement.remove(random_node)
    random_edge = random.choice(list(cyc.edges))
    n1, n2 = random_edge
    # Remove the original edge
    cyc.remove_edge(n1, n2)
    # Add intermediate nodes and new edges
    cyc.add_edges_from([(n1, random_node), (random_node, n2)])
    flows= Flows(nodes_out=frozenset(set()),
                 nodes_in=frozenset({random_node}),
                 edges_out=frozenset({random_edge}),
                 edges_in=frozenset({(n1, random_node), (random_node, n2)})
                )
    return cyc, complement, flows

                                                                             
def swap_two_nodes(cycle, complement, base):
    # perform whenever 2 < len(cycle) 
    possibles = [node for node in cycle.nodes if node!=base]
    random_nodes = list(random.sample(possibles, 2))
    n1, n2 = random_nodes[0], random_nodes[1]
    mapping = {n1: n2, n2: n1}
    C = nx.relabel_nodes(cycle, mapping)
    flows= Flows(nodes_out=frozenset(set()),
                 nodes_in=frozenset(set()),
                 edges_out=frozenset({e for e in cycle.edges if e not in C.edges}),
                 edges_in=frozenset({e for e in C.edges if e not in cycle.edges})
                )
    return C, complement, flows


def swap_two_opt(cycle, complement, base):
    # perform whenever 3 < len(cycle)
    C = cycle.copy()
    nx.directed_edge_swap(C, nswap=1)
    flows= Flows(nodes_out=frozenset(set()),
                 nodes_in=frozenset(set()),
                 edges_out=frozenset({e for e in cycle.edges if e not in C.edges}),
                 edges_in=frozenset({e for e in C.edges if e not in cycle.edges})
                )
    return C, complement, flows


