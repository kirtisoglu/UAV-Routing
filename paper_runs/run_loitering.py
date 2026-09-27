"""Value of loitering (paper Table 11): exact MISOCP with and without loitering.

R-class only, eta in {0.75, 1.0, 1.25}: 3 instances x 3 etas x 2 models
= 18 solves, one at a time. The row for an (instance, eta) pair is appended
and fsynced only after BOTH of its solves finish, so an interrupted run
never leaves a half-built row. Re-running resumes from the CSV.

The no-loiter model adds L_ij <= d_ij x_ij to the standing L_ij >= d_ij x_ij,
so L_ij = d_ij exactly on every flown leg. That identity is asserted on the
returned solution, and the energy budget is asserted against eta, so a
misconfigured run stops at the first solve instead of after hours.

Outputs:
  paper_runs/results/loitering.csv
  paper_runs/results/details/loiter_<instance>_eta<eta>_<full|nol>.json
"""
import os, sys, csv, json, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (make_instance, leg_records, R_CLASS,
                    GRAPH_SEED, GUROBI_SEED, SORTIE_TIME)

import gurobipy as gp
from uav_routing.solver.exact import solve_model_gurobi

ETAS       = (0.75, 1.0, 1.25)
TIME_LIMIT = float(os.environ.get("LOITER_TIME_LIMIT", 3600.0))
THREADS    = 0
HERE       = os.path.dirname(os.path.abspath(__file__))
CSV_PATH   = os.path.join(HERE, "results", "loitering.csv")
DET_DIR    = os.path.join(HERE, "results", "details")
LOG_DIR    = os.environ.get("LOITER_LOGDIR",
                            "/private/tmp/claude-501/loiter_logs")
CSV_COLS = ["Instance", "eta",
            "Obj (full)", "Bound (full)", "Gap full (%)", "Tour (full)",
            "Energy full (%)", "Loiter (m)", "Loiter legs", "Time full (s)",
            "Obj (no-loit)", "Bound (no-loit)", "Gap nol (%)", "Tour (no-loit)",
            "Energy nol (%)", "Time nol (s)",
            "Gain (%)", "Wall (s)"]


def safe(s):
    return s.replace(" ", "_").replace("(", "").replace(")", "").replace("_2_1", "121").lower()


def solve(instance, graph, drone, no_loiter, tag, env):
    res = solve_model_gurobi(instance, seed=GUROBI_SEED, time_limit=TIME_LIMIT,
                             env=env, threads=THREADS, no_loiter=no_loiter,
                             log_file=os.path.join(LOG_DIR, tag + ".log"))
    if not res or not res.get("arc_data"):
        raise SystemExit(f"[abort] {tag}: no solution returned")
    legs = leg_records(res, drone, instance)
    energy = sum(l["energy_J"] for l in legs)
    loiter = sum(l["loiter_m"] for l in legs)
    nloit = sum(1 for l in legs if l["loiter_m"] > 1e-6 * max(l["d_m"], 1.0))

    # --- self-validation: stop now rather than after hours of bad runs ---
    for l in legs:
        assert l["L_m"] >= l["d_m"] - 1e-6 * max(l["d_m"], 1.0), \
            f"{tag}: flown length below straight-line distance on leg {l['i']}->{l['j']}"
        if no_loiter:
            assert abs(l["L_m"] - l["d_m"]) <= 1e-6 * max(l["d_m"], 1.0), \
                f"{tag}: no-loiter model still loiters on leg {l['i']}->{l['j']}"
    assert energy <= instance.max_energy * (1 + 1e-6), \
        f"{tag}: energy {energy:.1f} exceeds the eta-scaled budget {instance.max_energy:.1f}"

    detail = {
        "tag": tag, "instance_nodes": len(graph.nodes),
        "graph_seed": GRAPH_SEED, "gurobi_seed": GUROBI_SEED,
        "sortie_time_h": SORTIE_TIME, "eta": instance.eta,
        "no_loiter": no_loiter, "time_limit_s": TIME_LIMIT,
        "energy_tiebreak": res.get("energy_tiebreak"),
        "obj": res["obj"], "reward": res.get("reward"),
        "bound": res["objbound"], "gap": res["gap"], "status": res["status"],
        "solve_time_s": res["solve_time"], "tour": res["tour"],
        "energy_total_J": energy, "max_energy_J": instance.max_energy,
        "energy_pct": 100.0 * energy / instance.max_energy,
        "loiter_total_m": loiter, "loiter_legs": nloit,
        "legs": legs,
    }
    with open(os.path.join(DET_DIR, tag + ".json"), "w") as f:
        json.dump(detail, f, indent=1)
        f.flush(); os.fsync(f.fileno())
    return res, detail


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

    env = gp.Env(params={"OutputFlag": 0})
    for name, path in R_CLASS.items():
        for eta in ETAS:
            if (name, eta) in done:
                print(f"[skip] {name} eta={eta}", flush=True)
                continue
            print(f"\n{'='*62}\n[start] {name} eta={eta} at "
                  f"{time.strftime('%H:%M:%S')}", flush=True)
            t0 = time.time()
            base = f"loiter_{safe(name)}_eta{eta}"
            inst, g, drone = make_instance(path, eta=eta)
            rf, df = solve(inst, g, drone, False, base + "_full", env)
            inst2, g2, drone2 = make_instance(path, eta=eta)
            rn, dn = solve(inst2, g2, drone2, True, base + "_nol", env)
            gain = 100.0 * (df["obj"] - dn["obj"]) / dn["obj"] if dn["obj"] else float("nan")
            row = {
                "Instance": name, "eta": eta,
                "Obj (full)": round(df["obj"], 2), "Bound (full)": round(df["bound"], 2),
                "Gap full (%)": f"{100*df['gap']:.2f}", "Tour (full)": len(df["tour"]) - 2,
                "Energy full (%)": f"{df['energy_pct']:.2f}",
                "Loiter (m)": f"{df['loiter_total_m']:.1f}",
                "Loiter legs": df["loiter_legs"],
                "Time full (s)": round(df["solve_time_s"], 1),
                "Obj (no-loit)": round(dn["obj"], 2), "Bound (no-loit)": round(dn["bound"], 2),
                "Gap nol (%)": f"{100*dn['gap']:.2f}", "Tour (no-loit)": len(dn["tour"]) - 2,
                "Energy nol (%)": f"{dn['energy_pct']:.2f}",
                "Time nol (s)": round(dn["solve_time_s"], 1),
                "Gain (%)": f"{gain:.2f}", "Wall (s)": round(time.time() - t0, 1),
            }
            with open(CSV_PATH, "a", newline="") as f:
                csv.DictWriter(f, fieldnames=CSV_COLS).writerow(row)
                f.flush(); os.fsync(f.fileno())
            print(f"[saved] {name} eta={eta}: full={row['Obj (full)']} "
                  f"nol={row['Obj (no-loit)']} gain={row['Gain (%)']}% "
                  f"loiter={row['Loiter (m)']} m", flush=True)
    print(f"\n{'='*62}\nDONE -> {CSV_PATH}", flush=True)


if __name__ == "__main__":
    main()
