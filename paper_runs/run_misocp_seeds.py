"""Slope-realization sweep for paper Table 3, columns s = 2, 3, 4.

Only the instances that close quickly at s = 1 (R101 (50), R101 (100),
R1_2_1 (200)); s = 1 itself comes from the main campaign. 3 instances x
3 seeds = 9 solves, one at a time, each row fsynced before the next starts.
Re-running resumes from the CSV.

Outputs:
  paper_runs/results/misocp_seeds.csv
  paper_runs/results/details/seeds_<instance>_s<seed>.json
"""
import os, sys, csv, json, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "experiments"))
from common import (make_instance, leg_records, R_CLASS,
                    GUROBI_SEED, SORTIE_TIME)

import gurobipy as gp
from uav_routing.solver.exact import solve_model_gurobi
from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES

# Which instances and which slope realizations, overridable from the
# environment so one driver covers every block of Table 3.
PATHS = dict(ALL_INSTANCES + EXPANSION_INSTANCES)
_i = os.environ.get("SEEDS_INSTANCES")
INSTANCES = ({n: PATHS[n] for n in
              (x.strip() for x in _i.split(";")) if n in PATHS}
             if _i else dict(R_CLASS))
_s = os.environ.get("SEEDS_LIST")
SEEDS = tuple(int(x) for x in _s.split(",")) if _s else (2, 3, 4)
TIME_LIMIT = float(os.environ.get("SEEDS_TIME_LIMIT", 3600.0))
THREADS    = 0
HERE       = os.path.dirname(os.path.abspath(__file__))
CSV_PATH   = os.path.join(HERE, "results", "misocp_seeds.csv")
DET_DIR    = os.path.join(HERE, "results", "details")
LOG_DIR    = os.environ.get("SEEDS_LOGDIR", "/private/tmp/claude-501/seeds_logs")
CSV_COLS   = ["Instance", "s", "Nodes", "Objective", "Bound", "Gap (%)",
              "Time (s)", "Tour", "Energy (%)", "Reward", "Loiter (m)",
              "Wall (s)", "Status", "Model"]


def safe(s):
    return s.replace(" ", "_").replace("(", "").replace(")", "").lower()


def main():
    os.makedirs(DET_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)
    if os.path.exists(CSV_PATH):
        with open(CSV_PATH) as f:
            done = {(r["Instance"], int(r["s"])) for r in csv.DictReader(f)}
        print(f"[resume] {len(done)} rows already on disk", flush=True)
    else:
        done = set()
        with open(CSV_PATH, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=CSV_COLS).writeheader()

    env = gp.Env(params={"OutputFlag": 0})
    for name, path in INSTANCES.items():
        for s in SEEDS:
            if (name, s) in done:
                print(f"[skip] {name} s={s}", flush=True)
                continue
            print(f"\n{'='*62}\n[start] {name} s={s} at {time.strftime('%H:%M:%S')}",
                  flush=True)
            t0 = time.time()
            inst, g, drone = make_instance(path, eta=1.0, seed=s)
            tag = f"seeds_{safe(name)}_s{s}"
            res = solve_model_gurobi(inst, seed=GUROBI_SEED, time_limit=TIME_LIMIT,
                                     env=env, threads=THREADS,
                                     log_file=os.path.join(LOG_DIR, tag + ".log"))
            wall = time.time() - t0
            if not res or not res.get("arc_data"):
                raise SystemExit(f"[abort] {name} s={s}: no solution returned")
            legs = leg_records(res, drone, inst)
            energy = sum(l["energy_J"] for l in legs)
            loiter = sum(l["loiter_m"] for l in legs)
            for l in legs:
                assert l["L_m"] >= l["d_m"] - 1e-6 * max(l["d_m"], 1.0), \
                    f"{tag}: flown length below straight-line distance"
            assert energy <= inst.max_energy * (1 + 1e-6), f"{tag}: energy over budget"

            detail = {
                "tag": tag, "instance": name, "nodes": len(g.nodes),
                "graph_seed": s, "gurobi_seed": GUROBI_SEED,
                "sortie_time_h": SORTIE_TIME, "eta": inst.eta,
                "time_limit_s": TIME_LIMIT,
                "energy_tiebreak": res.get("energy_tiebreak"),
                "obj": res["obj"], "reward": res.get("reward"),
                "bound": res["objbound"], "gap": res["gap"],
                "status": res["status"], "solve_time_s": res["solve_time"],
                "tour": res["tour"], "energy_total_J": energy,
                "max_energy_J": inst.max_energy,
                "energy_pct": 100.0 * energy / inst.max_energy,
                "loiter_total_m": loiter, "legs": legs,
            }
            with open(os.path.join(DET_DIR, tag + ".json"), "w") as f:
                json.dump(detail, f, indent=1)
                f.flush(); os.fsync(f.fileno())

            row = {
                "Instance": name, "s": s, "Nodes": len(g.nodes),
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
            print(f"[saved] {name} s={s}: obj={row['Objective']} "
                  f"gap={row['Gap (%)']}% tour={row['Tour']} wall={wall:.0f}s",
                  flush=True)
    print(f"\n{'='*62}\nDONE -> {CSV_PATH}", flush=True)


if __name__ == "__main__":
    main()
