"""Perturbation-dynamics figure for a single time-budgeted ILS run.

Reads the per-step trace written by run_ils_time_matched.py --dynamics-out
and plots it in the style of fig/ils_perturb_dynamics.png: the current
objective, the best-so-far objective, the perturbation events, and a
reference line for the exact solver's incumbent.

The top panel shows the whole run against wall-clock time. Because an
hour-long run fires far too many kicks to draw individually, the bottom
panel zooms into a window where the sawtooth and the kick markers are
legible.

Usage:
  python3 experiments/run_fig_pr15_dynamics.py TRACE.csv OUT.png \
      --target 7981.34 --title "PR15 (240)" --zoom 1800 1900
"""
import os, sys, csv, argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(path):
    it, wall, curr, best, kick = [], [], [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                it.append(int(row["iter"]));   wall.append(float(row["wall_s"]))
                curr.append(float(row["f_curr"])); best.append(float(row["f_best"]))
                kick.append(int(row["kick"]))
            except (ValueError, KeyError):
                continue
    return it, wall, curr, best, kick


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("out")
    ap.add_argument("--target", type=float, default=None)
    ap.add_argument("--title", default="ILS run")
    ap.add_argument("--zoom", nargs=2, type=float, default=None,
                    help="wall-clock window (s) for the lower panel")
    ap.add_argument("--max-points", type=int, default=25000)
    args = ap.parse_args()

    it, wall, curr, best, kick = load(args.trace)
    if not it:
        raise SystemExit(f"no rows in {args.trace}")
    n = len(it)
    kicks = sum(kick)
    print(f"{n} steps, {kicks} perturbations, {wall[-1]:.0f} s, "
          f"best {best[-1]:.2f}")

    step = max(1, n // args.max_points)
    W, C, B = wall[::step], curr[::step], best[::step]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8))

    # ---- top: whole run ----
    ax1.plot(W, C, color="grey", linewidth=0.6, alpha=0.7,
             label=r"$f_{\mathrm{curr}}$", zorder=2)
    ax1.plot(W, B, color="tab:blue", linewidth=1.8,
             label=r"$f_{\mathrm{best}}$", zorder=3)
    if args.target is not None:
        ax1.axhline(args.target, color="red", linestyle="--", linewidth=1.2,
                    label=f"MISOCP incumbent {args.target:,.0f}", zorder=4)
    ax1.set_xlabel("wall-clock (s)")
    ax1.set_ylabel("Objective")
    ax1.set_title(f"{args.title}: full run "
                  f"({n:,} steps, {kicks:,} perturbations)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="lower right", fontsize=9)

    # ---- bottom: zoom with individual kick markers ----
    if args.zoom:
        lo, hi = args.zoom
    else:                       # default: a 100 s window in the middle
        mid = wall[-1] * 0.5
        lo, hi = mid, min(mid + 100.0, wall[-1])
    idx = [i for i, w in enumerate(wall) if lo <= w <= hi]
    if idx:
        zw = [wall[i] for i in idx]
        ax2.plot(zw, [curr[i] for i in idx], color="grey", linewidth=0.9,
                 alpha=0.85, label=r"$f_{\mathrm{curr}}$", zorder=2)
        ax2.plot(zw, [best[i] for i in idx], color="tab:blue", linewidth=1.8,
                 label=r"$f_{\mathrm{best}}$", zorder=3)
        first = True
        for i in idx:
            if kick[i]:
                ax2.axvline(wall[i], color="orange", linewidth=0.8, alpha=0.55,
                            zorder=1, label="perturbation" if first else None)
                first = False
        if args.target is not None:
            ax2.axhline(args.target, color="red", linestyle="--",
                        linewidth=1.2, zorder=4)
        nk = sum(kick[i] for i in idx)
        ax2.set_title(f"detail, {lo:.0f} to {hi:.0f} s ({nk} perturbations)")
    ax2.set_xlabel("wall-clock (s)")
    ax2.set_ylabel("Objective")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="lower right", fontsize=9)

    fig.tight_layout()
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    fig.savefig(args.out, dpi=150, bbox_inches="tight")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
