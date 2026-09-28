"""Share of position pairs of the best routes whose Swap or 2-opt keeps every time
window reachable (the chain test at v_max), per instance. Quoted in Section 5.1;
rerun it after the reference campaign and update the sentence if the shares moved.

    python3 paper_runs/reorder_share.py                 # reads results/design_new.csv
    python3 paper_runs/reorder_share.py results/design_new_D6.csv

No solver is needed. The test is the time-window part of the feasibility test of
Section 4.3 (the energy budget is left out), applied to every pair 1 <= p < q <= k
of the best route reported for the instance.
"""
import os, sys, csv

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "experiments"))
os.chdir(REPO)
from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES
from run_ils_fb_cascade_demo import make_instance

PATHS = dict(ALL_INSTANCES + EXPANSION_INSTANCES)
ORDER = ["R101 (50)", "R101 (100)", "R1_2_1 (200)", "C101 (50)", "C101 (100)",
         "C1_2_1 (200)", "RC1_2_1 (200)", "R102 (100)", "R104 (100)",
         "C104 (100)", "RC104 (100)", "PR11 (48)", "PR15 (240)", "PR10 (288)"]


def windows_ok(route, G, depot, T_max, v_max):
    """Earliest arrivals at v_max stay inside every window and the return inside the horizon."""
    t, prev = 0.0, depot
    for n in route[1:]:
        t = max(G.nodes[n]["time_window"][0], t + G[prev][n]["distance"] / v_max)
        if t > G.nodes[n]["time_window"][1] + 1e-9:
            return False
        prev = n
    return t + G[prev][depot]["distance"] / v_max <= T_max + 1e-9


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "results", "design_new.csv")
    if not os.path.exists(path) and os.path.exists(os.path.join(HERE, path)):
        path = os.path.join(HERE, path)
    rows = {r["Instance"]: r for r in csv.DictReader(open(path))}
    print(f"{'instance':15s} {'k':>4s} {'pairs':>6s} {'swap ok':>8s} {'%':>6s} {'2-opt ok':>9s} {'%':>6s}")
    for name in ORDER:
        if name not in rows or not rows[name].get("Route"):
            print(f"{name:15s}  (no route in {os.path.basename(path)})"); continue
        inst, G, drone = make_instance(PATHS[name])
        depot, T_max = drone.base, inst.time_horizon
        v_max = getattr(drone, "max_speed", None) or getattr(drone, "speed_max")
        route = [int(x) for x in rows[name]["Route"].split("-")]
        if route[0] != depot or not windows_ok(route, G, depot, T_max, v_max):
            print(f"{name:15s}  route not window-feasible, check the CSV"); continue
        k = len(route) - 1; pairs = k * (k - 1) // 2; sw = to = 0
        for p in range(1, k):
            for q in range(p + 1, k + 1):
                r = list(route); r[p], r[q] = r[q], r[p]
                sw += windows_ok(r, G, depot, T_max, v_max)
                to += windows_ok(route[:p] + route[p:q + 1][::-1] + route[q + 1:], G, depot, T_max, v_max)
        print(f"{name:15s} {k:4d} {pairs:6d} {sw:8d} {100 * sw / pairs:6.1f} {to:9d} {100 * to / pairs:6.1f}")


if __name__ == "__main__":
    main()
