"""
graph.py
========
Graph construction utilities for UAV routing.
Converts node dictionaries to NetworkX graphs and provides cycle construction helpers.
"""

import networkx as nx
import math
import random

from typing import Optional
from .data import data_to_dict


# Conver this to an immutable graph.
class Graph(nx.Graph):
    """
    Holds raw Solomon data. The instance itself is the graph. 
    Never stores scaled values. Calibration is always applied externally.
    """
    def __init__(self, path: Optional[str] = None, seed: Optional[int] = None, slope: str = 'random', **kwargs):
        
        # Initialize the underlying nx.Graph structures
        super().__init__(**kwargs)
        
        if path is not None:
            node_dict, depot = data_to_dict(path)
            self.graph['base'] = int(depot)
            self._build_from_dict(node_dict)
            
            if slope == 'random':
                self._assign_info_slopes_randomly(seed=seed)
            elif slope == 'positive':
                self._assign_info_slopes_positive(seed=seed)
            elif slope == 'zero':
                for node in self.nodes:
                    self.nodes[node]['info_slope'] = 0.0
            else:
                raise ValueError(f"Unknown slope type '{slope}'. Use 'random', 'positive', or 'zero'.")

  
    def _build_from_dict(self, node_dict: dict):
        """
        Convert a dictionary to a Complete NetworkX Graph. Holds raw Solomon data. 
        Calibration is always applied externally in the Environment class.
        node attributes: position, info_at_lowest, time_window
        edge attributes: distance
        """ 
        for nid, data in node_dict.items():
            self.add_node(int(nid), **data)
            
        # Add Edges (Complete Graph)
        node_ids = list(node_dict.keys())
        for i in range(len(node_ids)):
            for j in range(i + 1, len(node_ids)):
                u, v = int(node_ids[i]), int(node_ids[j])
                pos_u = node_dict[u]['position']
                pos_v = node_dict[v]['position']
                
                dist = math.sqrt((pos_u[0]-pos_v[0])**2 + (pos_u[1]-pos_v[1])**2)  
                if dist == 0:
                    raise ValueError(f"Distance cannot be zero for {(u,v)}.")
                
                self.add_edge(u, v, distance=dist)    



    # must be done after calibration to pick from scaled time window.
    def _assign_info_slopes_randomly(self, seed: int = None):
        """
        Calculates the boundary for the slope and returns a random 
        value within that boundary to ensure 0 <= r_i <= 2*I_e.
        """
        rng = random.Random(seed)
        
        for node_id, data in self.nodes(data=True):
            random_slope = self._get_random_slope(node=node_id,
                                                  time_window=data['time_window'], 
                                                  info_at_lowest=data['info_at_lowest'],
                                                  seed=rng
                                                )
            self.nodes[node_id]['info_slope'] = random_slope



    def _assign_info_slopes_positive(self, seed: int = None):
        """
        Assigns a positive random slope to each node, sampled uniformly
        from [0, I_e / delta_t], so information can only grow over time.
        """
        rng = random.Random(seed)

        for node_id, data in self.nodes(data=True):
            delta_t = data['time_window'][1] - data['time_window'][0]
            if delta_t == 0:
                self.nodes[node_id]['info_slope'] = 0.0
            else:
                slope_bound = data['info_at_lowest'] / delta_t
                self.nodes[node_id]['info_slope'] = rng.uniform(0, slope_bound)

    def _get_random_slope(self, node, time_window, info_at_lowest, seed):
        """
        Calculates the boundary for the slope and returns a random 
        value within that boundary uniformly to ensure 0 <= r_i <= 2*I_e.
        """
        delta_t = time_window[1] - time_window[0]
        
        if delta_t == 0:
            print(f"Warning: Time window has zero duration for node {node} with time_window={time_window}. Setting slope to 0.")
            return 0.0
        
        slope_bound = info_at_lowest / delta_t
        return seed.uniform(-slope_bound, slope_bound)


    
def directed_cycle(tour_nodes, reference_graph):
    """
    Creates a DiGraph cycle from tour_nodes, pulling edge attributes 
    (like distance) from the reference_graph.
    """
    cycle = nx.DiGraph()
    # Iterate through nodes to create edges
    for i in range(len(tour_nodes)):
        u = tour_nodes[i]
        # Use modulo to wrap around to the first node at the end
        v = tour_nodes[(i + 1) % len(tour_nodes)]
        
        # Pull attributes from the master undirected graph
        if reference_graph.has_edge(u, v):
            attrs = reference_graph.get_edge_data(u, v)
            cycle.add_edge(u, v, **attrs)
        else:
            raise KeyError(f"Edge ({u}, {v}) not found in reference graph.")
    return cycle




def nearest_neighbor_tour_time(graph: nx.Graph, 
                          depot: int,
                          tour_length: int) -> nx.Graph:
    """
    Build a tour of exactly tour_length nodes starting and ending
    at the depot. At each step, the next node is the unvisited node 
    with the earliest time window opening.
    
    Parameters
    ----------
    graph       : nx.Graph with node attribute 'time_window' = (e_i, l_i)
    depot       : depot node id
    tour_length : number of customer nodes to visit

    Returns
    -------
    nx.Graph subgraph containing only the tour nodes and edges in order
    """
    unvisited = [n for n in graph.nodes if n != depot]
    
    tour_nodes = [depot]
    current = depot
    while len(tour_nodes) - 1 < tour_length and unvisited:
        nearest = min(unvisited, key=lambda n: graph.nodes[n]['time_window'][0])
        tour_nodes.append(nearest)
        unvisited.remove(nearest)
        current = nearest
    return directed_cycle(tour_nodes, graph)


def nearest_neighbor_tour(graph: nx.Graph, 
                          depot: int,
                          tour_length: int) -> nx.Graph:
    """
    Build a tour of exactly tour_length nodes starting and ending
    at the depot. At each step, the next node is the unvisited node 
    with the earliest time window opening.
    
    Parameters
    ----------
    graph       : nx.Graph with node attribute 'time_window' = (e_i, l_i)
    depot       : depot node id
    tour_length : number of customer nodes to visit

    Returns
    -------
    nx.Graph subgraph containing only the tour nodes and edges in order
    """
    unvisited = [n for n in graph.nodes if n != depot]
    
    tour_nodes = [depot]
    current = depot
    while len(tour_nodes) - 1 < tour_length and unvisited:
        nearest = min(unvisited, key=lambda n: graph.edges[(current,n)]['distance'])
        tour_nodes.append(nearest)
        unvisited.remove(nearest)
        current = nearest
    return directed_cycle(tour_nodes, graph)