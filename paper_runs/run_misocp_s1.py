"""MISOCP campaign on the expanded benchmark, one hour each.

Solves the MISOCP of Section 3.4 (time links with the tightened per-leg
big-M constants of Section 3.3) at the paper's slope realization (graph
seed 1), eta = 1.0, a 3600 s cap and all cores (Threads = 0).

Instances are solved ONE AT A TIME and the CSV row is appended (and
flushed) immediately after each solve, so an interrupted run keeps every
result already obtained. Re-running skips instances already in the CSV,
so the campaign resumes where it stopped. Delete the CSV to start over.

Ordered by size, so the cheap instances report first.

Outputs:
  paper_runs/results/misocp_s1.csv
  per-instance Gurobi logs in $MISOCP_LOGDIR (default: experiments)
"""
import os, sys, csv, time, json

_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
os.chdir(_repo_root)

import gurobipy as gp

from uav_routing.solver.exact import solve_model_gurobi
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import make_instance, leg_records, GUROBI_SEED as _GS
sys.path.insert(0, os.path.join(_repo_root, 'experiments'))
from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES

# MISOCP_TIME_LIMIT and MISOCP_TARGETS (';'-separated names) are read from
# the environment so subsets can be run without editing the script.
TIME_LIMIT  = float(os.environ.get("MISOCP_TIME_LIMIT", 3600.0))
GUROBI_SEED = 42
THREADS     = 0               # all cores, as in the paper's exact runs

PATHS = dict(ALL_INSTANCES + EXPANSION_INSTANCES)

# Clustered Solomon/Homberger instances plus the eight expansion instances,
# ordered by number of targets.
TARGETS = [
    "PR11 (48)",
    "R101 (50)",
    "C101 (50)",
    "R101 (100)",
    "C101 (100)",
    "R102 (100)",
    "R104 (100)",
    "C104 (100)",
    "RC104 (100)",
    "R1_2_1 (200)",
    "C1_2_1 (200)",
    "RC1_2_1 (200)",
    "PR15 (240)",
    "PR10 (288)",
]
_t = os.environ.get("MISOCP_TARGETS")
if _t:
    TARGETS = [x.strip() for x in _t.split(";") if x.strip()]

CSV_PATH = os.environ.get("MISOCP_CSV", "paper_runs/results/misocp_s1.csv")
CSV_COLS = ["Instance", "Nodes", "Objective", "Bound", "Gap (%)",
            "Time (s)", "Tour", "Energy (%)", "Reward", "Loiter (m)",
            "Build (s)", "Wall (s)",
            "Status", "Model"]


def safe(name):
    return name.lower().replace(" ", "_").replace("(", "").replace(")", "")


def main():
    logdir = os.environ.get("MISOCP_LOGDIR", "experiments")
    # Solver logs are scratch; the per-instance solution detail is a result and
    # is kept in the repo next to the campaign CSV.
    detail_dir = os.environ.get("MISOCP_DETAILS", "paper_runs/results/details")
    os.makedirs(detail_dir, exist_ok=True)
    os.makedirs(logdir, exist_ok=True)

    if os.path.exists(CSV_PATH):
        with open(CSV_PATH) as f:
            done = {row["Instance"] for row in csv.DictReader(f)}
        print(f"[resume] {len(done)} result(s) already on disk", flush=True)
    else:
        done = set()
        with open(CSV_PATH, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=CSV_COLS).writeheader()

    with gp.Env(empty=True) as env:
        env.setParam("OutputFlag", 0)
        env.start()

        for name in TARGETS:
            if name in done:
                print(f"[skip] {name} (already solved)", flush=True)
                continue
            if name not in PATHS:
                print(f"[warn] unknown instance {name}", flush=True)
                continue

            print(f"\n{'='*62}\n[start] {name} at {time.strftime('%H:%M:%S')}",
                  flush=True)
            t_build = time.time()
            instance, graph, drone = make_instance(PATHS[name])
            build = time.time() - t_build
            print(f"[built] {name}: {len(graph.nodes)} nodes in {build:.1f} s",
                  flush=True)

            log_file = os.path.join(logdir, f"misocp_{safe(name)}.log")
            t0 = time.time()
            res = solve_model_gurobi(instance, seed=GUROBI_SEED,
                                     time_limit=TIME_LIMIT, env=env,
                                     threads=THREADS,
                                     log_file=log_file)
            wall = time.time() - t0

            energy = "-"
            loiter_tot = "-"
            if res and res.get("arc_data"):
                tot = sum(drone.socp_energy_function(a["t"], a["y"], a["z"])
                          for a in res["arc_data"].values())
                energy = f"{tot / instance.max_energy * 100:.2f}"
                loiter_tot = f"{sum(a['L'] - a['d'] for a in res['arc_data'].values()):.1f}"
                # Full solution detail, one JSON per instance: the tour, the
                # per-leg schedule and the loitering, so Table 4 can be built
                # without re-solving.
                v_max = drone.speed_max
                legs = []
                for (i, j), ad in res["arc_data"].items():
                    e_leg = drone.socp_energy_function(ad["t"], ad["y"], ad["z"])
                    legs.append({
                        "i": i, "j": j,
                        "t_s": ad["t"],
                        "t_lo_s": ad["d"] / v_max,
                        "L_m": ad["L"],
                        "d_m": ad["d"],
                        "loiter_m": ad["L"] - ad["d"],
                        "v_ms": ad["L"] / ad["t"] if ad["t"] > 0 else None,
                        "energy_J": e_leg,
                        "energy_pct": 100.0 * e_leg / instance.max_energy,
                        "arrival_s": res["arrival_times"].get(j),
                        "tw": list(ad["tw"]),
                        "gamma": graph.nodes[j].get("info_slope"),
                    })
                detail = {
                    "instance": name, "nodes": len(graph.nodes),
                    "graph_seed": 1, "eta": instance.eta,
                    "energy_tiebreak": res.get("energy_tiebreak"),
                    "time_limit_s": TIME_LIMIT, "gurobi_seed": GUROBI_SEED,
                    "obj": res["obj"], "reward": res.get("reward"),
                    "bound": res["objbound"], "gap": res["gap"],
                    "status": res["status"], "solve_time_s": res["solve_time"],
                    "tour": res["tour"],
                    "energy_total_J": tot, "max_energy_J": instance.max_energy,
                    "energy_pct": 100.0 * tot / instance.max_energy,
                    "loiter_total_m": sum(a["L"] - a["d"] for a in res["arc_data"].values()),
                    "T_max_s": instance.T_max_s * instance._t_norm,
                    "v_min_ms": drone.speed_min, "v_max_ms": v_max,
                    "legs": sorted(legs, key=lambda r: (r["arrival_s"] is None,
                                                        r["arrival_s"] or 0.0)),
                }
                dpath = os.path.join(detail_dir, f"s1_{safe(name)}.json")
                with open(dpath, "w") as f:
                    json.dump(detail, f, indent=1)
                    f.flush(); os.fsync(f.fileno())
                print(f"[detail] {name} -> {dpath}", flush=True)

            row = {
                "Instance":   name,
                "Nodes":      len(graph.nodes),
                "Objective":  round(res["obj"], 2) if res and res.get("obj") is not None else "-",
                "Bound":      round(res["objbound"], 2) if res and res.get("objbound") is not None else "-",
                "Gap (%)":    f"{res['gap']*100:.2f}" if res and res.get("gap") is not None else "-",
                "Time (s)":   round(res["solve_time"], 1) if res and res.get("solve_time") else "-",
                "Tour":       len(res["tour"]) - 2 if res and res.get("tour") else 0,
                "Energy (%)": energy,
                "Reward":     round(res["reward"], 2) if res and res.get("reward") is not None else "-",
                "Loiter (m)": loiter_tot,
                "Build (s)":  round(build, 1),
                "Wall (s)":   round(wall, 1),
                "Status":     res.get("status", "-") if res else "no_solution",
                "Model":      "tightM",
            }
            # Append and flush immediately: a crash or sleep keeps this result.
            with open(CSV_PATH, "a", newline="") as f:
                csv.DictWriter(f, fieldnames=CSV_COLS).writerow(row)
                f.flush()
                os.fsync(f.fileno())
            print(f"[saved] {name}: obj={row['Objective']} bound={row['Bound']} "
                  f"gap={row['Gap (%)']}% tour={row['Tour']} "
                  f"wall={wall:.0f}s", flush=True)

    print(f"\n{'='*62}\nDONE -> {CSV_PATH}", flush=True)


if __name__ == "__main__":
    main()
