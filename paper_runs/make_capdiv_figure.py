"""Perturbation-dynamics figure for the shake cap divisor (paper Figure 2, next to Table 9).

One 600 s single-start ILS run from R4 on PR15 (240) per cap divisor
D in {3, 6, 12}, each recorded step by step with
run_ils_time_matched.py --dynamics-out. Style of the earlier figure: one panel
per D, the current objective in grey, the best-seen objective in color, every
shake step as a vertical line, and the exact solver's one-hour incumbent as a
dashed line.

Inputs:  paper_runs/results/details/dynamics/pr15_d<D>.csv
Output:  paper_runs/results/figures/ils_capdiv.png (copied to both papers)
"""
import os, csv, shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm

HERE = os.path.dirname(os.path.abspath(__file__))
DYN = os.path.join(HERE, "results", "details", "dynamics")
OUT = os.path.join(HERE, "results", "figures", "ils_capdiv.png")
PAPER_FIGS = [
    "/Users/kirtisoglu/GitHub/Brain/40-Papers/uav-routing/fig",
]   # the arXiv paper only; the submission keeps the earlier design
INSTANCE = "PR15 (240)"
MAX_IT = 15000        # the window the figure shows
DIVS = (3, 6, 12)
MISOCP_CSV = os.path.join(HERE, "results", "misocp_s1.csv")


def load(path):
    it, curr, best, kick = [], [], [], []
    for row in csv.DictReader(open(path)):
        it.append(int(row["iter"])); curr.append(float(row["f_curr"]))
        best.append(float(row["f_best"])); kick.append(int(row["kick"]))
    return it, curr, best, kick


def main():
    target = next(float(r["Objective"]) for r in csv.DictReader(open(MISOCP_CSV))
                  if r["Instance"] == INSTANCE)
    fig, axes = plt.subplots(1, len(DIVS), figsize=(6 * len(DIVS), 4.8),
                             sharey=True, squeeze=False)
    colors = cm.viridis([0.1, 0.5, 0.9])
    for c, D in enumerate(DIVS):
        ax = axes[0][c]; color = colors[c]
        p = os.path.join(DYN, f"pr15_d{D}.csv")
        if not os.path.exists(p):
            ax.set_title(f"$c=\\lceil k/{D} \\rceil$ (missing)"); continue
        it, curr, best, kick = load(p)
        n = sum(1 for v in it if v <= MAX_IT) or len(it)
        it, curr, best, kick = it[:n], curr[:n], best[:n], kick[:n]
        step = max(1, len(it) // 20000)
        first = True
        for i, k in enumerate(kick):
            if k:
                ax.axvline(it[i], color="orange", linewidth=0.8, alpha=0.5, zorder=1,
                           label="perturbation" if first else None)
                first = False
        ax.plot(it[::step], curr[::step], color="grey", linewidth=0.7, alpha=0.8,
                label=r"$f_{\mathrm{curr}}$", zorder=2)
        ax.plot(it[::step], best[::step], color=color, linewidth=2.2,
                label=r"$f_{\mathrm{best}}$", zorder=3)
        ax.axhline(target, color="red", linestyle="--", linewidth=1.2,
                   label="MISOCP incumbent", zorder=4)
        ax.set_title(f"$c=\\lceil k/{D} \\rceil$  ({sum(kick)} perturbations)")
        ax.set_xlim(0, MAX_IT)
        ax.set_xlabel("ILS iteration")
        ax.set_ylabel("Objective")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="lower right", fontsize=9)
    fig.suptitle(f"ILS perturbation dynamics on {INSTANCE}")
    fig.tight_layout()
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, dpi=150, bbox_inches="tight")
    for d in PAPER_FIGS:
        if os.path.isdir(d):
            shutil.copy(OUT, os.path.join(d, os.path.basename(OUT)))
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
