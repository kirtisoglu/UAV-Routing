"""Evidence for Section 5.8 from the route and move traces of a run.

  python3 paper_runs/analyze_traces.py ranks  [trace_dir]   acceptance rate by rank of the drawn move
  python3 paper_runs/analyze_traces.py shakes [trace_dir]   shake index of every improvement of R_best
  python3 paper_runs/analyze_traces.py stop   [trace_dir]   objective and stopping shake for candidate S

Traces are the files <stem>_moves.csv and <stem>_route.csv written by
run_ils_time_matched.py --route-trace / --move-trace (default directory
animation/traces).  A move trace lists, for every recorded event, the drawn
move with its rank among the moves of its set at the draw and the verdict, so
the acceptance rate by rank bucket is accepted / drawn within the bucket; this
is the statistic behind the restricted candidate list of Section 4.3 (the
rate decays with rank for Swap and 2-opt, not for Insert and Replace).  A
route trace lists every accepted move and every shake with the best value, so
the shake index of every improvement, the gaps between improvements and the
idle shakes after the last one follow, which calibrate S of Section 4.4:
"stop" reports, for each candidate S, the objective the run would have
returned and the shake at which it would have stopped.
"""
import csv, glob, os, sys
from collections import defaultdict

BUCKETS = [(1, 1), (2, 5), (6, 10), (11, 20), (21, 50), (51, 10 ** 9)]
OPS = ("add", "replace", "swap", "two_opt")


def bucket(r):
    for lo, hi in BUCKETS:
        if lo <= r <= hi:
            return f"{lo}-{hi if hi < 10 ** 9 else 'inf'}"


def ranks(trace_dir):
    for f in sorted(glob.glob(os.path.join(trace_dir, "*_moves.csv"))):
        events = {}
        with open(f) as fh:
            for r in csv.DictReader(fh):
                key = (r["iter"], r["op"])
                if key not in events:
                    events[key] = (r["verdict"], r["op"], int(r["rank"]))
        stats = defaultdict(lambda: [0, 0])
        for (v, o, rk) in events.values():
            b = bucket(rk)
            stats[(o, b)][1] += 1
            if v == "accepted":
                stats[(o, b)][0] += 1
        print(f"\n{os.path.basename(f).replace('_moves.csv', '')}: accepted/drawn by rank of the drawn move")
        for o in OPS:
            line = f"  {o:8s}"
            for lo, hi in BUCKETS:
                b = f"{lo}-{hi if hi < 10 ** 9 else 'inf'}"
                a, t = stats[(o, b)]
                line += f" | {b:>6s}: {a:5d}/{t:6d}" + (f" {100 * a / t:3.0f}%" if t else "     ")
            print(line)


def improvements(f):
    """(shake index, best value) at every improvement of R_best, and the total shakes.
    Reads a route trace (<stem>_route.csv: verdict, best) or a best-seen trace of the
    runner (wall_s, iter, best_obj, shakes); in the latter the total is unknown and
    taken as the shake of the last improvement."""
    with open(f) as fh:
        rows = list(csv.DictReader(fh))
    if rows and "shakes" in rows[0]:
        out = [(int(r["shakes"]), float(r["best_obj"])) for r in rows if r["shakes"] != ""]
        return out, (out[-1][0] if out else 0)
    shakes = 0; best = -1.0; out = []
    for r in rows:
        if r["verdict"] == "shake":
            shakes += 1
            continue
        if r["verdict"] == "accepted":
            b = float(r["best"])
            if b > best + 1e-9:
                best = b; out.append((shakes, b))
    return out, shakes


def shakes(trace_dir):
    print(f"{'run':16s} {'shakes':>6s} {'#impr':>5s} {'last impr':>9s} {'idle after':>10s} {'max gap':>7s}  last gaps")
    for f in sorted(glob.glob(os.path.join(trace_dir, "*_route.csv"))):
        imp, total = improvements(f)
        idx = [s for s, _ in imp]
        gaps = [b - a for a, b in zip(idx, idx[1:])]
        print(f"{os.path.basename(f).replace('_route.csv', ''):16s} {total:6d} {len(imp):5d} {idx[-1]:9d} "
              f"{total - idx[-1]:10d} {max(gaps) if gaps else 0:7d}  {gaps[-10:]}")


def stop(trace_dir, candidates=(25, 50, 100, 150, 200, 300)):
    print(f"{'run':16s} " + " ".join(f"{'S=' + str(S):>18s}" for S in candidates) + "   (objective @ stopping shake)")
    files = sorted(glob.glob(os.path.join(trace_dir, "*_route.csv"))) or sorted(glob.glob(os.path.join(trace_dir, "*.csv")))
    for f in files:
        imp, total = improvements(f)
        if not imp:
            continue
        cells = []
        for S in candidates:
            last_s, val = imp[0]
            stop_at = None
            for s, b in imp[1:]:
                if s - last_s >= S:
                    stop_at = last_s + S
                    break
                last_s, val = s, b
            if stop_at is None:
                stop_at = last_s + S if last_s + S <= total else total
            cells.append(f"{val:10.2f} @{stop_at:5d}")
        print(f"{os.path.basename(f).replace('_route.csv', '').replace('.csv', ''):16s} " + " ".join(f"{c:>18s}" for c in cells))


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "ranks"
    d = sys.argv[2] if len(sys.argv) > 2 else "animation/traces"
    {"ranks": ranks, "shakes": shakes, "stop": stop}[what](d)
