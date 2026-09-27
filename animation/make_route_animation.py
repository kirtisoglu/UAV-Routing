"""Animation of the route as the local search changes it.

Reads a route trace written by
    experiments/run_ils_time_matched.py --route-trace FILE
which records one row per accepted move and per shake, and renders a frame for
each row: the targets as points, the scheduled ones filled, the route as a
closed tour from the depot, and a panel title giving the event, the elapsed
time, the current objective and the best seen so far. Unscheduled targets are
grey, so a frame shows at a glance which regions the route is ignoring.

    python3 animation/make_route_animation.py TRACE.csv "R104 (100)" OUT.gif

Traces live in animation/traces and the rendered files in animation/out.

Options through the environment:
    ANIM_STRIDE   keep every n-th frame (default: at most 600 frames)
    ANIM_FPS      frames per second (default 12)
"""
import csv, os, sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "paper_runs"))
sys.path.insert(0, os.path.join(REPO, "experiments"))
os.chdir(REPO)
from common import make_instance
from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES

COLOR = {"add": "tab:green", "replace": "tab:orange",
         "swap": "tab:purple", "two_opt": "tab:blue", "shake": "tab:red"}


def main():
    trace, name = sys.argv[1], sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, "out", "route_animation.gif")
    rows = list(csv.DictReader(open(trace)))
    if not rows:
        raise SystemExit("empty trace")
    stride = int(os.environ.get("ANIM_STRIDE", max(1, len(rows) // 600)))
    rows = rows[::stride]

    instance, graph, drone = make_instance(dict(ALL_INSTANCES + EXPANSION_INSTANCES)[name])
    depot = drone.base
    pos = {n: graph.nodes[n]["position"] for n in graph.nodes}
    xs = [p[0] for p in pos.values()]; ys = [p[1] for p in pos.values()]

    fig, ax = plt.subplots(figsize=(7.2, 7.2))
    pad = 0.04 * max(max(xs) - min(xs), max(ys) - min(ys))
    ax.set_xlim(min(xs) - pad, max(xs) + pad); ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
    ax.scatter(xs, ys, s=14, color="0.82", zorder=1)
    ax.scatter([pos[depot][0]], [pos[depot][1]], s=90, marker="s", color="black", zorder=4)
    line, = ax.plot([], [], lw=1.5, zorder=2)
    pts = ax.scatter([], [], s=26, zorder=3)
    title = ax.set_title("", fontsize=11)

    def frame(i):
        r = rows[i]
        route = [int(x) for x in r["route"].split("-")]
        cyc = route + [depot]
        line.set_data([pos[n][0] for n in cyc], [pos[n][1] for n in cyc])
        c = COLOR.get(r["event"], "tab:blue")
        line.set_color(c)
        pts.set_offsets([[pos[n][0], pos[n][1]] for n in route if n != depot] or [[0, 0]])
        pts.set_color(c)
        title.set_text(f"{name}   {r['event']}   t = {float(r['wall_s']):.0f} s   "
                       f"targets {len(route) - 1}   f = {float(r['obj']):,.0f}   "
                       f"best {float(r['best']):,.0f}")
        return line, pts, title

    anim = FuncAnimation(fig, frame, frames=len(rows), blit=False)
    anim.save(out, writer=PillowWriter(fps=int(os.environ.get("ANIM_FPS", 12))))
    print(f"wrote {out}  ({len(rows)} frames, stride {stride})")


if __name__ == "__main__":
    main()
