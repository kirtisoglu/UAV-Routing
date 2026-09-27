"""Slope-regime sensitivity (paper Table 5): MISOCP under the four regimes.

Ten instances x four regimes is forty cells, but the "mixed" column is the
paper's default sampling and is already solved by the s = 1 campaign, so
only growth, decay and static are run here: 30 solves, one at a time, each
row fsynced before the next starts. Re-running resumes from the CSV.

Regime definitions follow Section 5.2 and are asserted on the sampled
slopes before each solve, so a mislabelled run stops immediately.

Outputs:
  paper_runs/results/slope_regimes.csv
  paper_runs/results/details/regime_<instance>_<regime>.json
"""
import os, sys, csv, json, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import make_instance, leg_records, GUROBI_SEED, SORTIE_TIME

import gurobipy as gp
from uav_routing.solver.exact import solve_model_gurobi

# The ten instances of Table 5. All R-class instances are finished across
# every regime before the C-class starts, then RC and the Cordeau instance.
INSTANCES = {
    "R101 (50)":    "datasets/data/50_r101.txt",
    "R101 (100)":   "datasets/data/r101.txt",
    "R1_2_1 (200)": "datasets/homberger_200/r1_2_1.txt",
    "R104 (100)":   "datasets/c_r_rc_100_100_Vansteen/r104.txt",
    "C101 (50)":    "datasets/data/50_c101.txt",
    "C101 (100)":   "datasets/data/c101.txt",
    "C1_2_1 (200)": "datasets/homberger_200/c1_2_1.txt",
    "C104 (100)":   "datasets/c_r_rc_100_100_Vansteen/c104.txt",
    "RC104 (100)":  "datasets/c_r_rc_100_100_Vansteen/rc104.txt",
    "PR15 (240)":   "datasets/pr11_20/pr15.txt",
}
REGIMES    = ("growth", "decay", "static")     # "mixed" comes from the campaign
TIME_LIMIT = float(os.environ.get("REGIME_TIME_LIMIT", 3600.0))
THREADS    = 0
HERE       = os.path.dirname(os.path.abspath(__file__))
CSV_PATH   = os.path.join(HERE, "results", "slope_regimes.csv")
DET_DIR    = os.path.join(HERE, "results", "details")
LOG_DIR    = os.environ.get("REGIME_LOGDIR", "/private/tmp/claude-501/regime_logs")
CSV_COLS   = ["Instance", "Regime", "Nodes", "Objective", "Bound", "Gap (%)",
              "Time (s)", "Tour", "Energy (%)", "Reward", "Loiter (m)",
              "Wall (s)", "Status", "Model"]


def safe(s):
    return s.replace(" ", "_").replace("(", "").replace(")", "").lower()


def check_regime(graph, regime):
    """The sampled slopes must match the regime the row will be labelled with."""
    base = graph.graph["base"]
    s = [graph.nodes[n]["info_slope"] for n in graph.nodes if n != base]
    if regime == "growth":
        assert all(x >= 0 for x in s) and any(x > 0 for x in s), "growth: slopes not all non-negative"
    elif regime == "decay":
        assert all(x <= 0 for x in s) and any(x < 0 for x in s), "decay: slopes not all non-positive"
    elif regime == "static":
        assert all(x == 0 for x in s), "static: slopes are not all zero"


def main():
    os.makedirs(DET_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)
    if os.path.exists(CSV_PATH):
        with open(CSV_PATH) as f:
            done = {(r["Instance"], r["Regime"]) for r in csv.DictReader(f)}
        print(f"[resume] {len(done)} rows already on disk", flush=True)
    else:
        done = set()
        with open(CSV_PATH, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=CSV_COLS).writeheader()

    env = gp.Env(params={"OutputFlag": 0})
    # Instance-major: every regime of one instance is finished before the
    # next instance starts, so each row of Table 5 completes in one go.
    todo = [(n, r) for n in INSTANCES for r in REGIMES if (n, r) not in done]
    print(f"[plan] {len(todo)} solves to run", flush=True)
    for name, regime in todo:
        print(f"\n{'='*62}\n[start] {name} {regime} at {time.strftime('%H:%M:%S')}",
              flush=True)
        t0 = time.time()
        inst, g, drone = make_instance(INSTANCES[name], eta=1.0, regime=regime)
        check_regime(g, regime)
        tag = f"regime_{safe(name)}_{regime}"
        res = solve_model_gurobi(inst, seed=GUROBI_SEED, time_limit=TIME_LIMIT,
                                 env=env, threads=THREADS,
                                 log_file=os.path.join(LOG_DIR, tag + ".log"))
        wall = time.time() - t0
        if not res or not res.get("arc_data"):
            print(f"[warn] {name} {regime}: no solution in the budget", flush=True)
            continue
        legs = leg_records(res, drone, inst)
        energy = sum(l["energy_J"] for l in legs)
        loiter = sum(l["loiter_m"] for l in legs)
        for l in legs:
            assert l["L_m"] >= l["d_m"] - 1e-6 * max(l["d_m"], 1.0), \
                f"{tag}: flown length below straight-line distance"
        assert energy <= inst.max_energy * (1 + 1e-6), f"{tag}: energy over budget"

        detail = {
            "tag": tag, "instance": name, "regime": regime,
            "nodes": len(g.nodes), "graph_seed": 1, "gurobi_seed": GUROBI_SEED,
            "sortie_time_h": SORTIE_TIME, "eta": inst.eta,
            "time_limit_s": TIME_LIMIT,
            "energy_tiebreak": res.get("energy_tiebreak"),
            "obj": res["obj"], "reward": res.get("reward"),
            "bound": res["objbound"], "gap": res["gap"], "status": res["status"],
            "solve_time_s": res["solve_time"], "tour": res["tour"],
            "energy_total_J": energy, "max_energy_J": inst.max_energy,
            "energy_pct": 100.0 * energy / inst.max_energy,
            "loiter_total_m": loiter, "legs": legs,
        }
        with open(os.path.join(DET_DIR, tag + ".json"), "w") as f:
            json.dump(detail, f, indent=1); f.flush(); os.fsync(f.fileno())

        row = {
            "Instance": name, "Regime": regime, "Nodes": len(g.nodes),
            "Objective": round(res["obj"], 2), "Bound": round(res["objbound"], 2),
            "Gap (%)": f"{100*res['gap']:.2f}",
            "Time (s)": round(res["solve_time"], 1),
            "Tour": len(res["tour"]) - 2,
            "Energy (%)": f"{100.0*energy/inst.max_energy:.2f}",
            "Reward": round(res.get("reward"), 2) if res.get("reward") is not None else "-",
            "Loiter (m)": f"{loiter:.1f}", "Wall (s)": round(wall, 1),
            "Status": res["status"], "Model": "tightM",
        }
        with open(CSV_PATH, "a", newline="") as f:
            csv.DictWriter(f, fieldnames=CSV_COLS).writerow(row)
            f.flush(); os.fsync(f.fileno())
        print(f"[saved] {name} {regime}: obj={row['Objective']} "
              f"gap={row['Gap (%)']}% tour={row['Tour']} wall={wall:.0f}s", flush=True)
    print(f"\n{'='*62}\nDONE -> {CSV_PATH}", flush=True)


if __name__ == "__main__":
    main()
