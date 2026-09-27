"""Perturbation-dynamics figure (fig:ils-capdiv): PR15 (240), one panel per cap divisor.

Inputs are the per-iteration traces the campaign driver writes with DESIGN_DYNAMICS=1:

    for D in 3 6 12; do
      DESIGN_INSTANCES="PR15 (240)" DESIGN_DYNAMICS=1 DESIGN_CAPDIV=$D DESIGN_OUT=_D${D}dyn \\
          python3 paper_runs/run_design.py new
    done
    python3 paper_runs/make_capdiv_figure.py

  paper_runs/results/details/dynamics/pr15_240_new_D{3,6,12}dyn.csv   (iter, wall_s, f_curr, f_best, kick)
  paper_runs/results/misocp_s1.csv                                   (the dashed incumbent)
Output: paper/fig/ils_capdiv.png, plus copies to the directories in PAPER_FIG_DIRS
(colon separated) if set. FIG_MAX_IT (default 15000) is the iteration window shown.
The run is deterministic given the seed, so the D = 3 panel is the run of Table 5.9.
"""
import os, csv, shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
DYN = os.path.join(HERE, "results", "details", "dynamics")
OUT = os.path.join(REPO, "paper", "fig", "ils_capdiv.png")
EXTRA_DIRS = [d for d in os.environ.get("PAPER_FIG_DIRS", "").split(":") if d]
INSTANCE, STEM = "PR15 (240)", "pr15_240"
MAX_IT = int(os.environ.get("FIG_MAX_IT", 15000))
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
    fig, axes = plt.subplots(1, len(DIVS), figsize=(6 * len(DIVS), 4.8), sharey=True, squeeze=False)
    colors = cm.viridis([0.1, 0.5, 0.9])
    missing = []
    for c, D in enumerate(DIVS):
        ax = axes[0][c]; color = colors[c]
        p = os.path.join(DYN, f"{STEM}_new_D{D}dyn.csv")
        if not os.path.exists(p):
            ax.set_title(f"$c=\\lceil k/{D} \\rceil$ (trace missing)"); missing.append(p); continue
        it, curr, best, kick = load(p)
        total_kicks = sum(kick)
        n = sum(1 for v in it if v <= MAX_IT) or len(it)
        it, curr, best, kick = it[:n], curr[:n], best[:n], kick[:n]
        step = max(1, len(it) // 20000)
        first = True
        for i, k in enumerate(kick):
            if k:
                ax.axvline(it[i], color="orange", linewidth=0.8, alpha=0.5, zorder=1,
                           label="shake" if first else None)
                first = False
        ax.plot(it[::step], curr[::step], color="grey", linewidth=0.7, alpha=0.8,
                label=r"$f(R_{\mathrm{curr}})$", zorder=2)
        ax.plot(it[::step], best[::step], color=color, linewidth=2.2,
                label=r"$f(R_{\mathrm{best}})$", zorder=3)
        ax.axhline(target, color="red", linestyle="--", linewidth=1.2, label="MISOCP incumbent", zorder=4)
        ax.set_title(f"$c=\\lceil k/{D} \\rceil$  ({sum(kick)} shakes shown, {total_kicks} in the run)")
        ax.set_xlim(0, MAX_IT); ax.set_xlabel("iteration"); ax.set_ylabel("objective")
        ax.grid(True, alpha=0.3); ax.legend(loc="lower right", fontsize=9)
    fig.suptitle(f"Perturbation dynamics on {INSTANCE}")
    fig.tight_layout()
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, dpi=150, bbox_inches="tight")
    for d in EXTRA_DIRS:
        if os.path.isdir(d):
            shutil.copy(OUT, os.path.join(d, os.path.basename(OUT)))
    print(f"wrote {OUT}" + (f"; missing traces: {missing}" if missing else ""))


if __name__ == "__main__":
    main()
