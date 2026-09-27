"""ILS convergence figure (fig:ils-convergence), one panel per instance.

Best-seen objective against wall-clock time for the reference run of Table 5.9
(paper_runs/run_design.py new), with the MISOCP value at its one-hour cap as a
dashed line. A trace records improvements only, so the step is held flat from
the last improvement to the end of the run, read from design_new.csv.

Inputs:  paper_runs/results/details/design_traces/<stem>_new.csv   (wall_s, iter, best_obj, shakes)
         paper_runs/results/design_new.csv, paper_runs/results/misocp_s1.csv
Output:  paper/fig/meta_convergence_timematched.png, plus copies to PAPER_FIG_DIRS (colon separated)

    python3 paper_runs/make_convergence_figure.py
"""
import os, re, csv, shutil
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
RES = os.path.join(HERE, "results")
TDIR = os.path.join(RES, "details", "design_traces")
OUT = os.path.join(REPO, "paper", "fig", "meta_convergence_timematched.png")
EXTRA_DIRS = [d for d in os.environ.get("PAPER_FIG_DIRS", "").split(":") if d]
ORDER = ["R101 (50)", "R101 (100)", "R1_2_1 (200)", "C101 (50)", "C101 (100)",
         "C1_2_1 (200)", "RC1_2_1 (200)", "R102 (100)", "R104 (100)",
         "C104 (100)", "RC104 (100)", "PR11 (48)", "PR15 (240)", "PR10 (288)"]
rd = lambda p: list(csv.DictReader(open(p)))
stem_of = lambda n: re.sub(r"[^a-z0-9]+", "_", n.lower()).strip("_")

mis = {r["Instance"]: (float(r["Objective"]), float(r["Gap (%)"])) for r in rd(os.path.join(RES, "misocp_s1.csv"))}
ends = {r["Instance"]: float(r["Wall (s)"]) for r in rd(os.path.join(RES, "design_new.csv"))}

fig, axes = plt.subplots(3, 5, figsize=(22, 11), squeeze=False)
missing = []
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
    else:
        missing.append(p)
    f, g = mis[name]
    ax.axhline(f, color="tab:red", linestyle="--", linewidth=1.2,
               label="MISOCP optimum" if g <= 0.5 else "MISOCP incumbent")
    ax.set_title(name); ax.set_xlabel("wall-clock (s)"); ax.set_ylabel("objective")
    ax.grid(True, alpha=0.3); ax.legend(loc="lower right", fontsize=8)
for ax in list(axes.flat)[len(ORDER):]:
    ax.axis("off")
fig.tight_layout()
os.makedirs(os.path.dirname(OUT), exist_ok=True)
fig.savefig(OUT, dpi=150, bbox_inches="tight"); plt.close(fig)
for d in EXTRA_DIRS:
    if os.path.isdir(d):
        shutil.copy(OUT, os.path.join(d, os.path.basename(OUT)))
print(f"wrote {OUT}" + (f"; missing traces: {missing}" if missing else ""))
