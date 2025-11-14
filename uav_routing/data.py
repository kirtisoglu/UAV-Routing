
from collections import namedtuple
import itertools
import random
import math

import networkx as nx
import matplotlib.pyplot as plt
from pyvis.network import Network

Node = namedtuple('Node', 'position info_at_lowest info_slope time_window')

        
def grid_3x3():
    """
    Generator that yields node attributes on a 3x3 grid.
    """
    
    metadata = {
                "base": 0, 
                "destination": 9,
                "max_energy":6030, 
                "campaign_time": 100, 
                "speed_min": 20, 
                "speed_max": 100,
                }
    
    tour_nodes = [0,3,6] # must be sorted?? frozen???
    tour_edges = []
    for i in tour_nodes[0:-1]:
        idx = tour_nodes.index(i)
        tour_edges.append((i, tour_nodes[idx+1]))
    tour_edges.append((tour_nodes[-1], tour_nodes[0]))
    cycle = nx.DiGraph()
    cycle.add_edges_from(tour_edges)
    
    nodes = {} 

    for i in range(3):
        for j in range(3):
            
            position = (i, j)
            node = i + j * 3  # unique node id

            if i==0 and j==0:
                time_window=[0,0]
                slope=0
                info_at_lowest=0
            
            else:    
                if i == 1: 
                    time_window = [1000, 2000]
                    slope = 20
                    info_at_lowest = 100

                elif i == 2:  # rightmost column
                    time_window = [0, 2000]
                    slope = 100
                    info_at_lowest = 4000
                else:
                    time_window = [0, 2000]
                    slope = 20
                    info_at_lowest = 100
                    
            #slope = (info_at_highest - info_at_lowest) / (highest - lowest)
            nodes[node] = node_data = {"position":position,
                                        "info_at_lowest":info_at_lowest, 
                                        "info_slope":slope,
                                        "time_window":time_window
                                        }
            # final destination
            nodes[9] = {"position":(0,0),
                        "info_at_lowest":0, 
                        "info_slope":0,
                        "time_window":[0,100]
                        }

    return nodes, cycle, metadata
    

def tour_info_increasing():
    metadata = {
                "base": 0, 
                "destination": 5,
                "max_energy":10000, 
                "campaign_time": 10000, 
                "speed_min": 10, 
                "speed_max": 20,
                }
     
    tour_nodes = [0,1,2]
    
    tour_edges = []
    for i in tour_nodes[0:-1]:
        idx = tour_nodes.index(i)
        tour_edges.append((i, tour_nodes[idx+1]))
    tour_edges.append((tour_nodes[-1], tour_nodes[0]))
    cycle = nx.DiGraph()
    cycle.add_edges_from(tour_edges)
    
    nodes = {} 
    
    
    for i in range(3):

        nodes[i] = {"position":(0,i),
                    "info_at_lowest":10, 
                    "info_slope":10,
                    "time_window":[0,10*i]
                    }
    
    return nodes, cycle, metadata


def tour_info_decreasing():
    
    metadata = {
                "base": 0, 
                "destination": 5,
                "max_energy":10000, 
                "campaign_time": 10000, 
                "speed_min": 1, 
                "speed_max": 10,
                }
     
    tour_nodes = [0,1,2,3]
    
    tour_edges = []
    for i in tour_nodes[0:-1]:
        idx = tour_nodes.index(i)
        tour_edges.append((i, tour_nodes[idx+1]))
    tour_edges.append((tour_nodes[-1], tour_nodes[0]))
    cycle = nx.DiGraph()
    cycle.add_edges_from(tour_edges)
    
    nodes = {} 
    
    
    for i in range(4):

        nodes[i] = {"position":(0,i),
                    "info_at_lowest":110, 
                    "info_slope":-1/10,
                    "time_window":[0,1000]
                    }
    
    return nodes, cycle, metadata


def grid_10x10_feasible():
    """
    Generator that yields node attributes on a 3x3 grid.
    """
    
    metadata = {
                "base": 0, 
                "destination": 9,
                "max_energy":6030, 
                "campaign_time": 100, 
                "speed_min": 20, 
                "speed_max": 100,
                }
    
    tour_nodes = [0,3,6] # must be sorted?? frozen???
    tour_edges = []
    for i in tour_nodes[0:-1]:
        idx = tour_nodes.index(i)
        tour_edges.append((i, tour_nodes[idx+1]))
    tour_edges.append((tour_nodes[-1], tour_nodes[0]))
    cycle = nx.DiGraph()
    cycle.add_edges_from(tour_edges)
    
    nodes = {} 

    for i in range(3):
        for j in range(3):
            
            position = (i, j)
            node = i + j * 3  # unique node id

            if i==0 and j==0:
                time_window=[0,0]
                slope=0
                info_at_lowest=0
            
            else:    
                if i == 1: 
                    time_window = [1000, 2000]
                    slope = 20
                    info_at_lowest = 100

                elif i == 2:  # rightmost column
                    time_window = [0, 2000]
                    slope = 100
                    info_at_lowest = 4000
                else:
                    time_window = [0, 2000]
                    slope = 20
                    info_at_lowest = 100
                    
            #slope = (info_at_highest - info_at_lowest) / (highest - lowest)
            nodes[node] = node_data = {"position":position,
                                        "info_at_lowest":info_at_lowest, 
                                        "info_slope":slope,
                                        "time_window":time_window
                                        }
            # final destination
            nodes[9] = {"position":(0,0),
                        "info_at_lowest":0, 
                        "info_slope":0,
                        "time_window":[0,100]
                        }

    return nodes, cycle, metadata



#Realistic spatial layout
#Greedy "snake" tour covering the whole grid
#Randomized node info/time parameters
def generate_grid_data(n_rows=100, n_cols=100, seed=0):
    random.seed(seed)

    # UAV and environment parameters
    v_min, v_max = 20, 100
    base = 0

    # --- Create nodes ---
    nodes = {}
    for i in range(n_rows):
        for j in range(n_cols):
            
            node_id = i * n_cols + j
            info_at_lowest = random.randint(-300, 300)
            info_slope = random.randint(-100, 100)
            # Time windows wide enough for any feasible path
            time_window = [0, 5000]
            nodes[node_id] = {"position":(i,j),
                              "info_at_lowest":info_at_lowest, 
                              "info_slope":info_slope,
                              "time_window":time_window
                              }

    # --- Define a simple snake-like tour (guaranteed connected) ---
    tour = []
    for i in range(n_rows):
        row = list(range(i * n_cols, (i + 1) * n_cols))
        if i % 2 == 1:
            row.reverse()
        tour.extend(row)

    # --- Compute distances and edges ---
    tour_edges = [(tour[k], tour[k+1]) for k in range(len(tour)-1)]
    distances = {
        (i, j): round(math.sqrt((nodes[i].position[0]-nodes[j].position[0])**2 +
                                (nodes[i].position[1]-nodes[j].position[1])**2) * 100, 2)
        for (i, j) in tour_edges
    }

    # --- Ensure feasible total energy ---
    # Rough estimate of minimum energy using simple model:
    total_dist = sum(distances.values())
    est_min_energy = 0.01 * total_dist * v_min + 0.02 * total_dist  # arbitrary conservative bound
    max_energy = est_min_energy * 5  # add margin for feasibility

    metadata = {
        "base": base,
        "max_energy": max_energy,
        "campaign_time": 10000,
        "speed_min": v_min,
        "speed_max": v_max,
    }

    return nodes, tour, metadata, tour_edges, distances


def greedy_feasible_tour(nodes):
    distances = {}
    nodes = list(nodes.keys())
    for u,v in zip(nodes, nodes):
        point1, point2 = nodes[u].position, nodes[v].position
        distances[(u,v)] = math.dist(point1, point2)
        
    feasible_starts = {node_id: node for node_id, node in nodes.items() if node.time_window[0]<= node.time_window[1]}
    # 
    #
    #
    return

# Random Graph-Based Generator 
# Feasibility guarenteed
def generate_random_graph(num_nodes=10, edge_prob=0.3, seed=0):
    random.seed(seed)

    metadata = {
        "base": 0,
        "max_energy": 10000,
        "campaign_time": 200,
        "speed_min": 20,
        "speed_max": 100,
    }

    # Random node positions
    nodes = {
        i: Node(
            position=(random.random()*10, random.random()*10),
            info_at_lowest=random.randint(100, 500),
            info_slope=random.randint(10, 100),
            time_window=[0, random.randint(1000, 3000)]
        )
        for i in range(num_nodes)
    }

    # Random edges with probability edge_prob
    edges = [(i, j) for i, j in itertools.permutations(range(num_nodes), 2) if random.random() < edge_prob]
    distances = {
        (i, j): math.dist(nodes[i].position, nodes[j].position)*100 for (i, j) in edges
    }

    # Random tour covering subset of nodes
    tour = random.sample(range(num_nodes), min(num_nodes, 6))
    tour_edges = [(tour[k], tour[k+1]) for k in range(len(tour)-1)]

    return nodes, tour, metadata, tour_edges, distances, edges




def plot_graph_with_positions(nodes, edges):
    """
    Plots a graph using NetworkX with specified node positions.

    Args:
        nodes (dict): A dictionary where keys are node IDs and values are (x, y) tuples
                      representing the position of each node.
        edges (list): A list of tuples, where each tuple represents an edge (pair of node IDs).
    """
    G = nx.Graph()

    # Add nodes to the graph
    G.add_nodes_from(nodes.keys())

    # Add edges to the graph
    G.add_edges_from(edges)

    # Draw the graph using the provided positions
    plt.figure(figsize=(8, 6)) # Optional: Adjust figure size
    nx.draw(G, pos=nodes, with_labels=True, node_color='skyblue', node_size=700, font_size=10, font_weight='bold')
    plt.title("Graph with Fixed Node Positions")
    plt.show()




import os 
def plot_interactive_graph(nodes_metadata, edges):
    """
    Creates an interactive graph visualization with hover tooltips using Pyvis.

    Args:
        nodes_metadata (dict): A dictionary keyed by node ID, 
                               mapped to a dictionary of metadata (including 'position').
        edges (list): A list of tuples representing pairs of connected node IDs.
        filename (str): The output HTML filename.
    """
    filename = "/Users/kirtisoglu/Documents/GitHub/UAV-Routing/graph.html"
    # Create a Pyvis network object
    # enable_physics=False allows manual positioning
    net = Network(height="750px", width="100%", bgcolor="#222222", 
                  font_color="white", notebook=False)

    # Add nodes with their metadata as "title" for the tooltip
    
    for node_id, node in nodes_metadata.items():
        # The 'title' attribute in pyvis becomes the hover tooltip text
        tooltip_text = f"Node {node_id}<br>"
        tooltip_text += f"position: {node.position}<br>"
        tooltip_text += f"info_at_lowest: {node.info_at_lowest}<br>"
        tooltip_text += f"info_slope: {node.info_slope}<br>"
        tooltip_text += f"time_window: {node.time_window}<br>"
        
        # Extract the (x, y) position
        # We need to scale or adjust the positions for Pyvis if they're extreme
        x_pos = node.position[0]
        y_pos = node.position[1]
        
        net.add_node(
            node_id, 
            title=tooltip_text, 
            x=x_pos * 20,  # Scale position for better Pyvis layout
            y=y_pos * 20,  # Scale position for better Pyvis layout
            label=str(node_id), # Display label on the node
            size=8,            # Node size
            color="#0008ff"     # Node color
        )

    # Add edges
    net.add_edges(edges)

    # Set initial view options (optional)
    net.show_buttons(filter_=['physics']) # Allows user to toggle physics simulation

    # Save the visualization to an HTML file
    net.write_html(filename)
    print(f"Interactive graph saved to {os.path.abspath(filename)}")



