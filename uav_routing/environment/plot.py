
import networkx as nx
import matplotlib.pyplot as plt
from pyvis.network import Network

import os 


def plot_graph_with_positions(nodes, edges, info=None):
    """
    Plots a graph using NetworkX with specified node positions.

    Args:
        nodes (dict): A dictionary where keys are node IDs and values are (x, y) tuples
                      representing the position of each node.
        edges (list): A list of tuples, where each tuple represents an edge (pair of node IDs).
    """
    G = nx.DiGraph()

    # Add nodes to the graph
    G.add_nodes_from(nodes.keys())

    # Add edges to the graph
    G.add_edges_from(edges)
    
    pos = {node: nodes[node]["position"] for node in nodes.keys()}
    # Draw the graph using the provided positions
    plt.figure(figsize=(8, 6)) # Optional: Adjust figure size
    nx.draw(G, pos=pos, with_labels=True, node_color='skyblue', node_size=300, font_size=10, font_weight='bold')
    plt.title("Graph with Fixed Node Positions")
    plt.show()
    

def plot_uav_tour(nodes, tour_edges, obj_value, energy_used=None):
    """
    Plots a specific UAV tour with objective and energy data.
    """
    G = nx.DiGraph()
    G.add_edges_from(tour_edges)
    
    # Extract positions
    pos = {node: nodes[node]["position"] for node in nodes.keys()}
    
    plt.figure(figsize=(10, 7))
    
    # 1. Draw all nodes in the background (light gray)
    nx.draw_networkx_nodes(G, pos, nodelist=nodes.keys(), 
                           node_color='lightgray', node_size=200, alpha=0.5)
    
    # 2. Highlight the Depot (Node 0)
    nx.draw_networkx_nodes(G, pos, nodelist=[0], 
                           node_color='red', node_size=400, label='Depot')
    
    # 3. Draw the tour path with arrows
    nx.draw_networkx_edges(G, pos, edgelist=tour_edges, 
                           edge_color='blue', width=2, arrowsize=20)
    
    # 4. Highlight visited nodes
    visited = [v for u, v in tour_edges if v != 0]
    nx.draw_networkx_nodes(G, pos, nodelist=visited, 
                           node_color='skyblue', node_size=300)
    
    # 5. Add Labels
    nx.draw_networkx_labels(G, pos, font_size=10)
    
    # Title with Metadata
    title_str = f"Feasible Tour | Objective: {obj_value:.2f}"
    if energy_used:
        title_str += f" | Energy: {energy_used:.1f}"
    
    plt.title(title_str)
    plt.legend(scatterpoints=1)
    plt.axis('off')
    plt.show()
    
    

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