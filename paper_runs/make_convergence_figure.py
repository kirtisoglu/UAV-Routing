"""ILS convergence figure (paper Figure 3), one panel per instance.

Best-seen objective against wall-clock time for the single-start run of
Table 10 — the design of Section 4 at c = ceil(k/3), ended by the iteration
limit rather than a time budget — with the MISOCP value at its one-hour cap
as a dashed line. The step ends where the run ended, so the width of a panel
is the time that instance actually took.

Inputs:  paper_runs/results/details/capdiv_traces/<instance>_d3.csv
         paper_runs/results/capdiv.csv, paper_runs/results/misocp_s1.csv
Output:  paper_runs/results/figures/meta_convergence_timematched.png
         copied into the paper's fig/ directory
"""
import os, re, csv, shutil
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
# the design of run_design.py new writes its best-seen traces here
TDIR = os.path.join(RES, "details", "design_traces")
OUT = os.path.join(RES, "figures", "meta_convergence_timematched.png")
FIG_DIRS = ["/Users/kirtisoglu/GitHub/Brain/40-Papers/uav-routing/fig"]
ORDER = ["R101 (50)", "R101 (100)", "R1_2_1 (200)", "C101 (50)", "C101 (100)",
         "C1_2_1 (200)", "RC1_2_1 (200)", "R102 (100)", "R104 (100)",
         "C104 (100)", "RC104 (100)", "PR11 (48)", "PR15 (240)", "PR10 (288)"]
rd = lambda p: list(csv.DictReader(open(p)))
stem_of = lambda n: re.sub(r"[^a-z0-9]+", "_", n.lower()).strip("_")

mis = {r["Instance"]: (float(r["Objective"]), float(r["Gap (%)"])) for r in rd(os.path.join(RES, "misocp_s1.csv"))}
ends = {r["Instance"]: float(r["Wall (s)"])
        for r in rd(os.path.join(RES, "design_new.csv"))}   # one row per instance

fig, axes = plt.subplots(3, 5, figsize=(22, 11), squeeze=False)
for ax, name in zip(axes.flat, ORDER):
    p = os.path.join(TDIR, f"{stem_of(name)}_new.csv")
    if os.path.exists(p):
        rows = rd(p)
        w = [float(r["wall_s"]) for r in rows]
        b = [float(r["best_obj"]) for r in rows]
        end = max(w[-1], ends.get(name, w[-1]))
        w.append(end); b.append(b[-1])          # hold the best to the end of the run
        ax.step(w, b, where="post", color="tab:blue", linewidth=1.8, label="ILS best")
        ax.set_xlim(0, end * 1.02)
    f, g = mis[name]
    ax.axhline(f, color="tab:red", linestyle="--", linewidth=1.2,
               label="MISOCP optimum" if g <= 0.5 else "MISOCP incumbent")
    ax.set_title(name); ax.set_xlabel("wall-clock (s)"); ax.set_ylabel("Objective")
    ax.grid(True, alpha=0.3); ax.legend(loc="lower right", fontsize=8)
for ax in list(axes.flat)[len(ORDER):]:
    ax.axis("off")
fig.tight_layout()
os.makedirs(os.path.dirname(OUT), exist_ok=True)
fig.savefig(OUT, dpi=150, bbox_inches="tight"); plt.close(fig)
for d in FIG_DIRS:
    shutil.copy(OUT, os.path.join(d, os.path.basename(OUT)))
print(f"wrote {OUT} and copied to {len(FIG_DIRS)} paper fig dir(s)")
