"""Regenerate the dataset figures for the paper (Section 6.1).

Figure 2 (spatial layout): four representative geometries -- R101 (100)
random, C101 (100) clustered, RC104 (100) mixed, PR15 (240) Cordeau map.
Figure 3 (TW timeline): one instance per window regime -- R101 (100)
co-monotone, R102 (100) transition, RC104 (100) inverted, PR15 (240)
overlapping. Sorted by opening time, the pattern of the closing times
makes the regime visible at a glance.

Reuses the parsing in data_to_dict; styling follows the original figures.
"""
import os, sys, shutil

_repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo not in sys.path:
    sys.path.insert(0, _repo)
os.chdir(_repo)

import numpy as np
import matplotlib.pyplot as plt

from uav_routing.environment.data import data_to_dict


SPATIAL = [
    ("R101 (100)",  "datasets/data/r101.txt"),
    ("C101 (100)",  "datasets/data/c101.txt"),
    ("RC104 (100)", "datasets/c_r_rc_100_100_Vansteen/rc104.txt"),
    ("PR15 (240)",  "datasets/pr11_20/pr15.txt"),
]
TW = [
    ("R101 (100), co-monotone",  "datasets/data/r101.txt"),
    ("R102 (100), transition",   "datasets/c_r_rc_100_100_Vansteen/r102.txt"),
    ("RC104 (100), inverted",    "datasets/c_r_rc_100_100_Vansteen/rc104.txt"),
    ("PR15 (240), overlapping",  "datasets/pr11_20/pr15.txt"),
]

OUT_DIR = "fig"
PAPER_DIR = "/tmp/UAV-Paper/fig"
BRAIN_DIR = "/Users/kirtisoglu/GitHub/Brain/40-Papers/uav-routing/fig"


# ---- Figure 2: spatial layout (2x2 panels) ----
fig, axes = plt.subplots(2, 2, figsize=(11, 9))
for ax, (name, path) in zip(axes.flat, SPATIAL):
    nodes, depot = data_to_dict(path)
    depot = int(depot)
    cust_pos = np.array([v['position'] for k, v in nodes.items() if k != depot])
    depot_pos = np.array(nodes[depot]['position'])
    infos = np.array([v['info_at_lowest'] for k, v in nodes.items() if k != depot])
    sc = ax.scatter(cust_pos[:, 0], cust_pos[:, 1], c=infos, cmap='viridis',
                    s=22, alpha=0.85, edgecolors='none')
    ax.scatter(*depot_pos, c='red', s=140, marker='*', zorder=5,
               edgecolors='black', linewidths=0.6, label='Depot')
    ax.set_title(name, fontsize=12)
    ax.set_aspect('equal')
    ax.set_xlabel('x'); ax.set_ylabel('y')
    ax.legend(loc='upper right', fontsize=8)
    plt.colorbar(sc, ax=ax, label='$I_{e_i}$', fraction=0.046, pad=0.04)
fig.tight_layout()
out2 = os.path.join(OUT_DIR, "datasets_spatial.png")
os.makedirs(OUT_DIR, exist_ok=True)
fig.savefig(out2, dpi=180, bbox_inches='tight')
print(f"Saved {out2}")
plt.close(fig)

# ---- Figure 3: TW timeline (2x2 panels, one per window regime) ----
fig, axes = plt.subplots(2, 2, figsize=(11, 9))
for ax, (name, path) in zip(axes.flat, TW):
    nodes, depot = data_to_dict(path)
    depot = int(depot)
    cust = [(k, v['time_window'][0], v['time_window'][1])
            for k, v in nodes.items() if k != depot]
    cust.sort(key=lambda x: x[1])
    ys = np.arange(len(cust))
    for y, (k, e, l) in zip(ys, cust):
        ax.plot([e, l], [y, y], color='steelblue', linewidth=1.2, alpha=0.8)
    T_max = nodes[depot]['time_window'][1]
    ax.axvline(T_max, color='red', linestyle='--', linewidth=0.8,
               label=f'$T_{{\\max}}$ = {T_max:.0f}')
    ax.set_title(name, fontsize=12)
    ax.set_xlabel('Time (benchmark units)')
    ax.set_ylabel('Target (sorted by $e_i$)')
    ax.legend(loc='lower right', fontsize=9)
    ax.grid(True, alpha=0.3)
fig.tight_layout()
out3 = os.path.join(OUT_DIR, "tw_timeline.png")
fig.savefig(out3, dpi=180, bbox_inches='tight')
print(f"Saved {out3}")
plt.close(fig)


# Copy to paper + brain
for dst_dir in [PAPER_DIR, BRAIN_DIR]:
    if os.path.exists(dst_dir):
        for src in [out2, out3]:
            shutil.copy(src, os.path.join(dst_dir, os.path.basename(src)))
            print(f"  -> {os.path.join(dst_dir, os.path.basename(src))}")
