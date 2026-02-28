
import networkx as nx
import math

class DistanceError(Exception):
    "Raised when distance d_ij == 0 for an edge (i,j)."


    
    
#graph attributes: base
#node attributes: position, info_at_lowest, time_window
#edge attributes: distance      
def dict_to_graph(nodes, base_id):
    """
    Convert a dictionary to a Complete NetworkX Graph.
    """
    G = nx.Graph()
    G.graph['base'] = base_id
    
    # Add Nodes
    for nid, data in nodes.items():
        G.add_node(int(nid), **data)
        
    # Add Edges (Complete Graph)
    node_ids = list(nodes.keys())
    for i in range(len(node_ids)):
        for j in range(i + 1, len(node_ids)):
            u, v = node_ids[i], node_ids[j]
            pos_u = nodes[u]['position']
            pos_v = nodes[v]['position']
            
            dist = math.sqrt((pos_u[0]-pos_v[0])**2 + (pos_u[1]-pos_v[1])**2) # converting 1 solomon unit to 1000 m. canceled.
            if dist == 0:
                raise DistanceError(f"Distance cannot be zero for the edge {(u,v)}.")
            G.add_edge(int(u), (v), distance=dist)
            
    return G


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


def nearest_neighbors_cycle(G, length):
    "Constructs a cycle of length 'length' starting from and ending at the base node."
    
    tour_nodes = []
    
    start_node = G.graph['base']
    tour_nodes.append(start_node)
    
    visited = {start_node}
    current_node = start_node
    
    while len(tour_nodes) <= length:
        neighbors = G[current_node]
        
        # Filter for those not yet in our path
        unvisited = [n for n in neighbors if n not in visited]
        
        if not unvisited:
            raise print(f"All neighbors of node {current_node} are visited in tour nodes {tour_nodes}.")  
            
        # Find the node with the minimum 'distance' attribute
        next_node = min(unvisited, key=lambda n: G[current_node][n]['distance'])
        
        tour_nodes.append(next_node)
        visited.add(next_node)
        current_node = next_node

    tour = directed_cycle(tour_nodes, G)
    
    return tour