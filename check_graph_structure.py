"""
Check graph structure to understand the infeas issue.
"""

import pickle
import networkx as nx

# Load environment
with open("uav_routing/notebooks/calibrated_env.pkl", "rb") as f:
    data = pickle.load(f)

graph = data['graph']
drone = data['drone']

print("="*80)
print("GRAPH STRUCTURE ANALYSIS")
print("="*80)

print(f"\nNumber of nodes: {len(graph.nodes)}")
print(f"Number of edges: {len(graph.edges)}")
print(f"Base node: {drone.base}")

# Check if it's a complete graph
n_nodes = len(graph.nodes)
max_edges = n_nodes * (n_nodes - 1)  # Directed, no self-loops
print(f"\nMax possible edges (directed, no self-loops): {max_edges}")
print(f"Actual edges: {len(graph.edges)}")
print(f"Is complete graph? {len(graph.edges) == max_edges}")

# Check if there's a self-loop at depot
print(f"\nSelf-loop at depot (0,0)? {(0, 0) in graph.edges}")

# Check edges from and to depot
edges_from_depot = [(i, j) for (i, j) in graph.edges if i == 0]
edges_to_depot = [(i, j) for (i, j) in graph.edges if j == 0]

print(f"\nEdges from depot: {len(edges_from_depot)}")
print(f"Edges to depot: {len(edges_to_depot)}")

# Sample a few edges to see their attributes
print("\nSample edge attributes:")
sample_edges = list(graph.edges)[:3]
for edge in sample_edges:
    print(f"  Edge {edge}:")
    print(f"    Distance: {graph.edges[edge]['distance']:.2f}")
    if 'time' in graph.edges[edge]:
        print(f"    Time: {graph.edges[edge]['time']:.2f}")

# Check node attributes
print("\nSample node attributes:")
for node in list(graph.nodes)[:3]:
    print(f"\n  Node {node}:")
    print(f"    Time window: {graph.nodes[node]['time_window']}")
    print(f"    Info at lowest: {graph.nodes[node]['info_at_lowest']:.2f}")
    print(f"    Info slope: {graph.nodes[node]['info_slope']:.6f}")

# Check depot specifically
print(f"\n  Depot (node {drone.base}):")
print(f"    Time window: {graph.nodes[drone.base]['time_window']}")
print(f"    Info at lowest: {graph.nodes[drone.base]['info_at_lowest']:.2f}")
print(f"    Info slope: {graph.nodes[drone.base]['info_slope']:.6f}")

print("\n" + "="*80)
print("KEY INSIGHT:")
print("="*80)
print("The graph is a NetworkX graph that represents the ORIGINAL problem.")
print("It likely does NOT have self-loops.")
print("When we create variables in Gurobi, we need to ensure ALL edges exist,")
print("including the self-loop at depot, even if not in the original graph.")
