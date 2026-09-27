"""Figure 3: ILS convergence, one panel per instance of Table 10.

The panels, their order and the dashed MISOCP reference are read straight out of
Table 10 in the paper, so the figure cannot drift from the table it shares a
protocol with. The curves come from the stage-1 traces, which are the runs the table reports:
every instance reproduces the table's objective to the cent. The animation
traces reproduce the objectives too, but three of them were recorded under load
and their clocks run 3.5x to 78x slow, so they cannot carry a wall-clock axis.
A trace records improvements only, so it stops at t_best and the curve is held
flat from there to the run time, both read from the table.

    python3 experiments/run_fig_convergence_tm.py
"""
import csv, os, re, shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
PAPER = "/Users/kirtisoglu/GitHub/Brain/40-Papers/uav-routing"
TEX = os.path.join(PAPER, "ArXiv-version.tex")
OUT = "fig/meta_convergence_timematched.png"

COL_ILS = "#1E6FBE"
COL_REF = "#C93B2C"


def stem(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def num(cell):
    """11\\,921.13 or \\textbf{11\\,921.13} -> 11921.13"""
    return float(re.sub(r"\\textbf\{|\}|\\,|\s", "", cell))


def panels_from_table():
    """(instance, f*, proven) per row of Table 10, in the table's own order."""
    tex = open(TEX, encoding="utf-8").read()
    body = re.search(r"\\label\{tab:matheuristic-vs-exact\}.*?\\midrule\n(.*?)\\bottomrule",
                     tex, re.S).group(1)
    out = []
    for line in body.strip().split("\n"):
        c = [x.strip() for x in line.rstrip(" \\\\").split("&")]
        if len(c) < 9:
            continue
        name = c[0].replace("\\_", "_").strip()
        out.append((name, num(c[2]), num(c[3]), num(c[4]),
                    num(c[5]), num(c[7]) == 0.0))   # f_ILS, t_best, run, f*, gap==0
    return out


def read_trace(path):
    xs, ys = [], []
    for row in csv.DictReader(open(path)):
        xs.append(float(row["wall_s"]))
        ys.append(float(row["best_obj"]))
    return xs, ys


def main():
    panels = panels_from_table()
    cols = 3
    rows = -(-len(panels) // cols)
    fig, axes = plt.subplots(rows, cols, figsize=(12, 2.35 * rows), squeeze=False)
    for ax, (name, f_ils, t_best, run, ref, proven) in zip(axes.flat, panels):
        s = stem(name)
        path = f"experiments/tm_ils_{s}_final_{s}_trace.csv"
        if not os.path.exists(path):
            ax.set_visible(False)
            print(f"  no trace for {name}")
            continue
        xs, ys = read_trace(path)
        if abs(ys[-1] - f_ils) > 0.005:
            print(f"  {name}: trace ends at {ys[-1]:,.2f}, table says {f_ils:,.2f}")
        xs, ys = xs + [run], ys + [ys[-1]]        # hold the best to the run's end
        ax.step(xs, ys, where="post", color=COL_ILS, linewidth=1.5, zorder=4)
        ax.axhline(ref, color=COL_REF, linestyle=(0, (6, 3)), linewidth=1.3, zorder=3)
        lo, hi = min(min(ys), ref), max(max(ys), ref)
        pad = 0.06 * (hi - lo) or 1.0
        ax.set_ylim(lo - pad, hi + pad)
        ax.set_xlim(0, run)
        ax.set_title(f"{name}  ($t_{{best}}$ {t_best:,.0f} s, run {run:,.0f} s)", fontsize=10)
        ax.tick_params(labelsize=8)
        ax.grid(True, alpha=0.3)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    for ax in axes.flat[len(panels):]:
        ax.set_visible(False)
    for ax in axes[-1, :]:
        ax.set_xlabel("wall-clock time (s)", fontsize=9)
    for ax in axes[:, 0]:
        ax.set_ylabel("best objective", fontsize=9)
    handles = [
        plt.Line2D([], [], color=COL_ILS, lw=1.5, label="ILS best-seen objective"),
        plt.Line2D([], [], color=COL_REF, lw=1.3, linestyle=(0, (6, 3)),
                   label="MISOCP at the $3{,}600$ s cap (optimum, or incumbent where the gap is open)"),
    ]
    fig.legend(handles=handles, loc="upper center", ncol=2, frameon=False,
               bbox_to_anchor=(0.5, 1.005), fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    os.makedirs("fig", exist_ok=True)
    fig.savefig(OUT, dpi=200, bbox_inches="tight")
    plt.close(fig)
    n_opt = sum(1 for *_, p in panels if p)
    print(f"Wrote {OUT}  ({len(panels)} panels, {n_opt} against a proven optimum)")
    if os.path.isdir(os.path.join(PAPER, "fig")):
        shutil.copy(OUT, os.path.join(PAPER, "fig", os.path.basename(OUT)))
        print(f"Copied to {PAPER}/fig/")


if __name__ == "__main__":
    main()
