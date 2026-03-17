"""
data_analysis.py
================
Dataset analysis and visualization tools for Solomon-format UAV routing instances.
Each plot function accepts a list of inputs, where each input is either:
  - a file path (str) to a Solomon-format .txt file, or
  - a Graph object (nx.Graph with node attributes: position, time_window).
"""

import math
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import spearmanr
import networkx as nx

from uav_routing.environment.data import data_to_dict


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _extract(source):
    """
    Normalize a source into (nodes_dict, depot_id, campaign_time, label).

    source can be:
      - str  : file path -> parsed via data_to_dict
      - Graph : nx.Graph with 'base' in graph attrs and node attrs
                position, time_window
    """
    if isinstance(source, str):
        nodes, depot = data_to_dict(source)
        depot = int(depot)
        campaign_time = nodes[depot]['time_window'][1]
        label = os.path.splitext(os.path.basename(source))[0]
        return nodes, depot, campaign_time, label

    # Assume nx.Graph-like object
    graph = source
    depot = graph.graph.get('base', 0)
    nodes = {}
    for nid, data in graph.nodes(data=True):
        nodes[nid] = data
    tw = nodes[depot]['time_window']
    campaign_time = tw[1] if hasattr(tw, '__getitem__') else tw
    name = getattr(graph, 'name', '') or 'graph'
    label = name if name != '' else 'graph'
    return nodes, depot, campaign_time, label


def _normalize(sources):
    """Ensure sources is a list. Allows passing a single path or graph."""
    if isinstance(sources, (list, tuple)):
        return list(sources)
    return [sources]


def _depot_distances(nodes, depot):
    """Return arrays (node_ids, distances, tw_opens, tw_closes, tw_widths)
    for all customer nodes (excludes depot)."""
    dx, dy = nodes[depot]['position']
    ids, dists, opens, closes, widths = [], [], [], [], []
    for nid, data in nodes.items():
        if nid == depot:
            continue
        x, y = data['position']
        d = math.sqrt((x - dx) ** 2 + (y - dy) ** 2)
        tw_o, tw_c = data['time_window'][0], data['time_window'][1]
        ids.append(nid)
        dists.append(d)
        opens.append(tw_o)
        closes.append(tw_c)
        widths.append(tw_c - tw_o)
    return (np.array(ids), np.array(dists), np.array(opens),
            np.array(closes), np.array(widths))


# ---------------------------------------------------------------------------
# 1. Scatter plot: distance from depot vs. TW opening time
# ---------------------------------------------------------------------------

def plot_distance_vs_tw_open(sources, figsize=(5, 4)):
    """
    One subplot per source. x = Euclidean distance from depot,
    y = TW opening time. Spearman rho printed in title.
    """
    sources = _normalize(sources)
    n = len(sources)
    fig, axes = plt.subplots(1, n, figsize=(figsize[0] * n, figsize[1]), squeeze=False)

    for ax, src in zip(axes[0], sources):
        nodes, depot, _, label = _extract(src)
        _, dists, opens, _, _ = _depot_distances(nodes, depot)

        rho, _ = spearmanr(dists, opens)

        ax.scatter(dists, opens, s=20, alpha=0.7, edgecolors='none')

        ax.set_title(f'{label}  (ρ = {rho:.2f})', fontsize=10)
        ax.set_xlabel('Distance from depot')
        ax.set_ylabel('TW opening time')

    fig.suptitle('Distance from Depot vs. TW Opening Time', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    return fig


# ---------------------------------------------------------------------------
# 2. Binned box plot: distance quartile vs. TW opening time
# ---------------------------------------------------------------------------

def plot_binned_tw_open(sources, n_bins=4, figsize=(5, 4)):
    """
    Nodes are binned into n_bins distance quartiles from the depot.
    A box plot of TW opening times is shown per bin.
    """
    sources = _normalize(sources)
    n = len(sources)
    fig, axes = plt.subplots(1, n, figsize=(figsize[0] * n, figsize[1]), squeeze=False)

    for ax, src in zip(axes[0], sources):
        nodes, depot, _, label = _extract(src)
        _, dists, opens, _, _ = _depot_distances(nodes, depot)

        quantiles = np.percentile(dists, np.linspace(0, 100, n_bins + 1))
        bin_data = []
        bin_labels = []
        for i in range(n_bins):
            lo, hi = quantiles[i], quantiles[i + 1]
            mask = (dists >= lo) & (dists <= hi) if i < n_bins - 1 else (dists >= lo)
            bin_data.append(opens[mask])
            bin_labels.append(f'Q{i+1}\n[{lo:.1f},{hi:.1f}]')

        ax.boxplot(bin_data, tick_labels=bin_labels, patch_artist=True,
                   boxprops=dict(facecolor='steelblue', alpha=0.6),
                   medianprops=dict(color='tomato', linewidth=2))
        ax.set_title(label, fontsize=10)
        ax.set_xlabel('Distance quartile from depot')
        ax.set_ylabel('TW opening time')

    fig.suptitle('TW Opening Time by Distance Quartile', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    return fig


# ---------------------------------------------------------------------------
# 3. Temporal reachability check
# ---------------------------------------------------------------------------

def plot_temporal_reachability(sources, figsize=(5, 4)):
    """
    For each node, earliest_arrival = distance_from_depot (1 unit dist = 1 unit time).
    A node is reachable if earliest_arrival <= tw_open.
    Scatter of earliest_arrival vs tw_open; unreachable nodes highlighted.
    Fraction of unreachable nodes printed on each panel.
    """
    sources = _normalize(sources)
    n = len(sources)
    fig, axes = plt.subplots(1, n, figsize=(figsize[0] * n, figsize[1]), squeeze=False)

    for ax, src in zip(axes[0], sources):
        nodes, depot, _, label = _extract(src)
        _, dists, opens, _, _ = _depot_distances(nodes, depot)

        reachable = dists <= opens
        frac_unreachable = (~reachable).sum() / len(reachable)

        ax.scatter(dists[reachable], opens[reachable],
                   s=20, alpha=0.7, label='Reachable', color='steelblue', edgecolors='none')
        ax.scatter(dists[~reachable], opens[~reachable],
                   s=25, alpha=0.9, label='Unreachable', color='tomato',
                   marker='x', linewidths=1.5)

        # diagonal: earliest_arrival == tw_open
        lim = max(dists.max(), opens.max()) * 1.05
        ax.plot([0, lim], [0, lim], 'k--', linewidth=1, alpha=0.5, label='arrival = open')

        ax.set_xlim(0, lim)
        ax.set_ylim(0, lim)
        ax.set_title(label, fontsize=10)
        ax.set_xlabel('Earliest arrival (= dist from depot)')
        ax.set_ylabel('TW opening time')
        ax.text(0.05, 0.93, f'Unreachable: {frac_unreachable:.0%}',
                transform=ax.transAxes, fontsize=8,
                verticalalignment='top',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='lightyellow', alpha=0.8))
        if src == sources[0]:
            ax.legend(fontsize=7, loc='lower right')

    fig.suptitle('Temporal Reachability (dist = travel time assumption)', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    return fig


# ---------------------------------------------------------------------------
# 4. Time window timeline (horizontal bar) plot
# ---------------------------------------------------------------------------

def plot_tw_timeline(sources, figsize=(6, 5)):
    """
    Horizontal bars [tw_open, tw_close] for each node, sorted by tw_open.
    One subplot per source.
    """
    sources = _normalize(sources)
    n = len(sources)
    fig, axes = plt.subplots(1, n, figsize=(figsize[0] * n, figsize[1]), squeeze=False)

    for ax, src in zip(axes[0], sources):
        nodes, depot, campaign_time, label = _extract(src)
        _, _, opens, closes, _ = _depot_distances(nodes, depot)

        order = np.argsort(opens)
        opens_s = opens[order]
        closes_s = closes[order]
        y = np.arange(len(opens_s))

        ax.barh(y, closes_s - opens_s, left=opens_s,
                height=0.8, color='steelblue', alpha=0.7)
        ax.axvline(campaign_time, color='tomato', linestyle='--',
                   linewidth=1.2, label=f'Campaign end ({campaign_time:.0f})')
        ax.set_title(label, fontsize=10)
        ax.set_xlabel('Time')
        ax.set_ylabel('Nodes (sorted by TW open)')
        ax.legend(fontsize=7)
        ax.set_yticks([])

    fig.suptitle('Time Window Timeline', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    return fig


# ---------------------------------------------------------------------------
# 5. TW width histogram
# ---------------------------------------------------------------------------

def plot_tw_width_histogram(sources, bins=15, figsize=(4, 3)):
    """
    Distribution of TW widths (tw_close - tw_open) per source.
    """
    sources = _normalize(sources)
    n = len(sources)
    fig, axes = plt.subplots(1, n, figsize=(figsize[0] * n, figsize[1]), squeeze=False)

    for ax, src in zip(axes[0], sources):
        nodes, depot, _, label = _extract(src)
        _, _, _, _, widths = _depot_distances(nodes, depot)

        ax.hist(widths, bins=bins, color='steelblue', alpha=0.8, edgecolor='white')
        ax.axvline(widths.mean(), color='tomato', linestyle='--',
                   linewidth=1.5, label=f'Mean = {widths.mean():.1f}')
        ax.set_title(label, fontsize=10)
        ax.set_xlabel('TW width')
        ax.set_ylabel('Count')
        ax.legend(fontsize=7)

    fig.suptitle('Time Window Width Distribution', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    return fig


# ---------------------------------------------------------------------------
# 6. Tightness ratio bar chart
# ---------------------------------------------------------------------------

def plot_tightness_ratio(sources, figsize=(6, 4)):
    """
    Tightness ratio = mean(tw_width) / campaign_time per source.
    Lower ratio = tighter scheduling constraints.
    """
    sources = _normalize(sources)
    labels, ratios, mean_widths, campaigns = [], [], [], []

    for src in sources:
        nodes, depot, campaign_time, label = _extract(src)
        _, _, _, _, widths = _depot_distances(nodes, depot)
        ratio = widths.mean() / campaign_time
        labels.append(label)
        ratios.append(ratio)
        mean_widths.append(widths.mean())
        campaigns.append(campaign_time)

    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=figsize)
    bars = ax.bar(x, ratios, color='steelblue', alpha=0.8, edgecolor='white', width=0.5)

    for bar, r, mw, ct in zip(bars, ratios, mean_widths, campaigns):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.003,
                f'{r:.3f}\n(μ={mw:.0f}/{ct:.0f})',
                ha='center', va='bottom', fontsize=7.5)

    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=15, ha='right')
    ax.set_ylabel('Tightness ratio  (mean TW width / campaign time)')
    ax.set_title('TW Tightness Ratio per Dataset')
    fig.tight_layout()
    return fig




# ---------------------------------------------------------------------------
# 7. Plot data points
# ---------------------------------------------------------------------------

def plot_graph_with_positions(nodes, edges, name=None):
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
    plt.figure(figsize=(6, 4)) # Optional: Adjust figure size
    nx.draw(G, pos=pos, with_labels=True, node_color='skyblue', node_size=200, font_size=7, font_weight='bold')
    plt.title(f"Graph with Fixed Node Positions for {name}")
    plt.show()
    

# ---------------------------------------------------------------------------
# Convenience: run all plots
# ---------------------------------------------------------------------------

def analyse_datasets(sources, show=True):
    """
    Run all six analyses for the given list of sources.
    Each source can be a file path (str) or a Graph object.
    Returns a dict of {name: figure}.
    """
    sources = _normalize(sources)
    figs = {
        'distance_vs_tw_open':   plot_distance_vs_tw_open(sources),
        'binned_tw_open':        plot_binned_tw_open(sources),
        'temporal_reachability': plot_temporal_reachability(sources),
        'tw_timeline':           plot_tw_timeline(sources),
        'tw_width_histogram':    plot_tw_width_histogram(sources),
        'tightness_ratio':       plot_tightness_ratio(sources),
    }
    if show:
        plt.show()
    return figs


def main():
    _BASE = os.path.join(os.path.dirname(__file__),
                         '..', '..', 'datasets', 'data')

    PATHS = [
        os.path.join(_BASE, '50_c101.txt'),
        os.path.join(_BASE, '50_r101.txt'),
        os.path.join(_BASE, 'r101.txt'),
        os.path.join(_BASE, 'rc101.txt'),
        os.path.join(_BASE, 'c101.txt'),
    ]

    analyse_datasets(PATHS, show=True)


if __name__ == '__main__':
    main()
