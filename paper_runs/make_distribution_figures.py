"""Speed and arrival-time distributions (paper Section 5, distributions).

Reads the per-leg records saved by the s = 1 campaign, so it needs no
solver. One panel per instance, in the style of the earlier arrival-time
figure: filled histograms with black edges, counts on the vertical axis,
bars colored by the sign of the target's slope gamma (negative, positive,
zero). Rows group the instances by class: R-type, C-type, then RC-type and
Cordeau.

  arrival_time_distr.png  normalized arrival position (a_j - e_j)/(l_j - e_j)
                          of every visited target
  speed_distr.png         speed of every flown leg, colored by the slope sign
                          of the leg's destination target, with v_mp, v_mr
                          and v_max marked

Outputs go to paper_runs/results/figures/ and are copied to both papers.
"""
import json, os, glob, shutil, statistics as st

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
DET  = os.path.join(HERE, "results", "details")
OUT  = os.path.join(HERE, "results", "figures")
PAPER_FIGS = [
    "/Users/kirtisoglu/GitHub/Brain/40-Papers/uav-routing/fig",
    "/Users/kirtisoglu/GitHub/Brain/40-Papers/uav-routing/Submission-Comp. & Ind. Eng/fig",
]

ROWS = [
    ("R-type",           ["R101 (50)", "R101 (100)", "R1_2_1 (200)", "R102 (100)", "R104 (100)"]),
    ("C-type",           ["C101 (50)", "C101 (100)", "C1_2_1 (200)", "C104 (100)"]),
    ("RC-type, Cordeau", ["RC1_2_1 (200)", "RC104 (100)", "PR11 (48)", "PR15 (240)", "PR10 (288)"]),
]
SIGNS = [("negative", "tab:blue"), ("positive", "tab:orange"), ("zero", "tab:green"),
         ("depot return", "tab:green")]
V_MP, V_MR, V_MAX = 33.67, 44.31, 61.0
NCOL = max(len(r[1]) for r in ROWS)

plt.rcParams.update({"font.size": 15, "axes.titlesize": 17, "legend.fontsize": 13})


def sign_of(gamma):
    if gamma is None or abs(gamma) < 1e-12:
        return "zero"
    return "negative" if gamma < 0 else "positive"


def load():
    """instance -> (arrival positions by sign, leg speeds by sign)"""
    data = {}
    for p in sorted(glob.glob(os.path.join(DET, "s1_*.json"))):
        d = json.load(open(p))
        horizon = d["T_max_s"]
        pos = {s: [] for s, _ in SIGNS}
        spd = {s: [] for s, _ in SIGNS}
        for l in d["legs"]:
            e, u = l["tw"]
            s = sign_of(l.get("gamma"))
            if l["v_ms"]:
                spd["depot return" if l.get("gamma") is None else s].append(l["v_ms"])
            if u > e and l["arrival_s"] is not None and (e, u) != (0.0, horizon):
                pos[s].append((l["arrival_s"] - e) / (u - e))
        data[d["instance"]] = (pos, spd)
    return data


def panel_grid(data, which, bins, rng, xlabel, ylabel, title, fname, marks=False):
    fig, axes = plt.subplots(len(ROWS), NCOL, figsize=(5 * NCOL, 4.6 * len(ROWS)),
                             squeeze=False)
    for r, (group, names) in enumerate(ROWS):
        for c in range(NCOL):
            ax = axes[r][c]
            if c >= len(names) or names[c] not in data:
                ax.axis("off")
                continue
            name = names[c]
            series = data[name][which]
            for s, color in SIGNS:
                if series[s]:
                    ax.hist(series[s], bins=bins, range=rng, alpha=0.6,
                            edgecolor="black", color=color, label=s)
            if marks:
                for v, lab in ((V_MP, "$v_{mp}$"), (V_MR, "$v_{mr}$"), (V_MAX, "$v_{\\max}$")):
                    ax.axvline(v, color="black", linestyle=":", linewidth=1.2)
                    ax.text(v, ax.get_ylim()[1] * 0.97, lab, ha="center", va="top", fontsize=11,
                            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85))
            ax.set_title(name)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.legend()
        axes[r][0].annotate(group, xy=(0, 0.5), xytext=(-75, 0), xycoords="axes fraction",
                            textcoords="offset points", rotation=90, ha="center", va="center",
                            fontsize=17, fontweight="bold")
    fig.suptitle(title, y=1.0, fontsize=19)
    fig.tight_layout(w_pad=2.5)
    out = os.path.join(OUT, fname)
    fig.savefig(out, dpi=160, bbox_inches="tight")
    plt.close(fig)
    for d in PAPER_FIGS:
        if os.path.isdir(d):
            shutil.copy(out, os.path.join(d, fname))
    print(f"wrote {out}")


def main():
    os.makedirs(OUT, exist_ok=True)
    data = load()
    panel_grid(data, 0, 20, (0, 1),
               "Normalized arrival position", "Number of targets",
               "Arrival Time Distribution by Slope", "arrival_time_distr.png")
    panel_grid(data, 1, 20, (30, 62), "Leg speed (m/s)", "Number of legs",
               "Speed Distribution by Slope of the Destination Target", "speed_distr.png", marks=True)
    for name, (pos, spd) in data.items():
        allp = sum(pos.values(), []); alls = sum(spd.values(), [])
        late = 100 * sum(1 for x in allp if x > 0.5) / max(1, len(allp))
        print(f"  {name:14s} targets {len(allp):3d} late {late:5.1f}%  "
              f"speed median {st.median(alls):5.2f}  "
              + "  ".join(f"{s}:{len(pos[s])}" for s, _ in SIGNS))


if __name__ == "__main__":
    main()
