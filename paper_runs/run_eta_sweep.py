"""Energy sensitivity sweep (paper Table 9): MISOCP at eta in {0.5,...,1.5}.

The six instances of that table at five energy levels is thirty solves, but
most are already paid for and are skipped rather than repeated:
  * eta = 1.0 for all six comes from the s = 1 campaign;
  * eta = 0.75 and 1.25 for the R-class come from the loitering campaign,
    whose loiter-allowed model is this same standard MISOCP.
Only the missing cells are solved, one at a time, each row fsynced before
the next starts. Re-running resumes from the CSV.

Outputs:
  paper_runs/results/eta_sweep.csv
  paper_runs/results/details/eta_<instance>_eta<eta>.json
"""
import os, sys, csv, json, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import make_instance, leg_records, GUROBI_SEED, SORTIE_TIME

import gurobipy as gp
from uav_routing.solver.exact import solve_model_gurobi

# The six instances of Table 9, R-class first so the fast results land early.
INSTANCES = {
    "R101 (50)":    "datasets/data/50_r101.txt",
    "R101 (100)":   "datasets/data/r101.txt",
    "R1_2_1 (200)": "datasets/homberger_200/r1_2_1.txt",
    "C101 (50)":    "datasets/data/50_c101.txt",
    "C101 (100)":   "datasets/data/c101.txt",
    "C1_2_1 (200)": "datasets/homberger_200/c1_2_1.txt",
}
ETAS       = (0.5, 1.5, 0.75, 1.25)      # 1.0 already exists for every row
TIME_LIMIT = float(os.environ.get("ETA_TIME_LIMIT", 3600.0))
THREADS    = 0
HERE       = os.path.dirname(os.path.abspath(__file__))
CSV_PATH   = os.path.join(HERE, "results", "eta_sweep.csv")
DET_DIR    = os.path.join(HERE, "results", "details")
LOG_DIR    = os.environ.get("ETA_LOGDIR", "/private/tmp/claude-501/eta_logs")
LOIT_CSV   = os.path.join(HERE, "results", "loitering.csv")
CSV_COLS   = ["Instance", "eta", "Nodes", "Objective", "Bound", "Gap (%)",
              "Time (s)", "Tour", "Energy (%)", "Reward", "Loiter (m)",
              "Wall (s)", "Status", "Model"]


def safe(s):
    return s.replace(" ", "_").replace("(", "").replace(")", "").lower()


def already_elsewhere():
    """(instance, eta) pairs whose solve exists in another campaign."""
    have = set()
    if os.path.exists(LOIT_CSV):
        for r in csv.DictReader(open(LOIT_CSV)):
            have.add((r["Instance"], float(r["eta"])))
    return have


def main():
    os.makedirs(DET_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)
    if os.path.exists(CSV_PATH):
        with open(CSV_PATH) as f:
            done = {(r["Instance"], float(r["eta"])) for r in csv.DictReader(f)}
        print(f"[resume] {len(done)} rows already on disk", flush=True)
    else:
        done = set()
        with open(CSV_PATH, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=CSV_COLS).writeheader()
    have = already_elsewhere()

    env = gp.Env(params={"OutputFlag": 0})
    todo = [(n, e) for e in ETAS for n in INSTANCES
            if (n, e) not in done and (n, e) not in have]
    print(f"[plan] {len(todo)} solves to run", flush=True)
    for name, eta in todo:
        print(f"\n{'='*62}\n[start] {name} eta={eta} at {time.strftime('%H:%M:%S')}",
              flush=True)
        t0 = time.time()
        inst, g, drone = make_instance(INSTANCES[name], eta=eta)
        tag = f"eta_{safe(name)}_eta{eta}"
        res = solve_model_gurobi(inst, seed=GUROBI_SEED, time_limit=TIME_LIMIT,
                                 env=env, threads=THREADS,
                                 log_file=os.path.join(LOG_DIR, tag + ".log"))
        wall = time.time() - t0
        if not res or not res.get("arc_data"):
            print(f"[warn] {name} eta={eta}: no solution in the budget", flush=True)
            continue
        legs = leg_records(res, drone, inst)
        energy = sum(l["energy_J"] for l in legs)
        loiter = sum(l["loiter_m"] for l in legs)
        for l in legs:
            assert l["L_m"] >= l["d_m"] - 1e-6 * max(l["d_m"], 1.0), \
                f"{tag}: flown length below straight-line distance"
        assert energy <= inst.max_energy * (1 + 1e-6), \
            f"{tag}: energy {energy:.1f} over the eta-scaled budget {inst.max_energy:.1f}"

        detail = {
            "tag": tag, "instance": name, "nodes": len(g.nodes),
            "graph_seed": 1, "gurobi_seed": GUROBI_SEED,
            "sortie_time_h": SORTIE_TIME, "eta": eta,
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
            "Instance": name, "eta": eta, "Nodes": len(g.nodes),
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
        print(f"[saved] {name} eta={eta}: obj={row['Objective']} "
              f"gap={row['Gap (%)']}% tour={row['Tour']} wall={wall:.0f}s", flush=True)
    print(f"\n{'='*62}\nDONE -> {CSV_PATH}", flush=True)


if __name__ == "__main__":
    main()
