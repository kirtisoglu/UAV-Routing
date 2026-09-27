"""Section 4 design campaign: the matheuristic of Section 4 on all fourteen
instances, one single-start run from R4 per instance, and optionally the
previous design (paper_runs/campaign/final_run.py) for a same-machine
comparison.

Configurations
  new   --fast-sets --scaled-socp --reorder-rcl L_R --sweep enum
        --max-idle-shakes S            (the design of Section 4)
  old   the 2026-09-26 base: roulette over the full reorder sets, physical-unit
        SOCP, (post, cons) arithmetic sweep, stop after maxIter = 13000
        iterations without an improvement

Both use the env of final_run.py (ILS_INSERT_RATIO=1 ILS_SHAKE_KNAP=6
ILS_NO_RETURN=2 ILS_REORDER_W=exch) and the same seed, cap divisor and
initial tour.  The stall safeguard (ILS_SHAKE_STALL) is kept for the old
design as it was run, and set to 0 for the new one, whose reorder sets are
bounded so that exhaustion fires by itself.

  python3 paper_runs/run_design.py new            # -> results/design_new.csv
  python3 paper_runs/run_design.py old            # -> results/design_old.csv
  DESIGN_INSTANCES="R104 (100);PR15 (240)" python3 paper_runs/run_design.py new

Environment knobs: DESIGN_RCL (L_R, default 20), DESIGN_IDLE (S, default 100),
DESIGN_CAPDIV (D, default 3), DESIGN_BUDGET (wall-clock safeguard, default
14400 s), DESIGN_WORKERS (parallel runs; keep at 1 for timings that are
comparable with the MISOCP's), DESIGN_OUT (suffix of the CSV and trace names,
so that a parameter grid or a variant does not overwrite the main campaign,
e.g. DESIGN_OUT=_L10), DESIGN_EXTRA (flags appended to every run, e.g.
"--fixed-speed" for the value-of-speed table or "--no-loiter" for the
value-of-loitering table).

  DESIGN_RCL=10 DESIGN_OUT=_L10 DESIGN_INSTANCES="R104 (100);RC104 (100);PR15 (240);PR10 (288)" \
      python3 paper_runs/run_design.py new
  DESIGN_EXTRA="--fixed-speed" DESIGN_OUT=_fixed python3 paper_runs/run_design.py new

Finished runs are appended to the CSV at once
and a re-run resumes from it.  The best-seen trace of a run is kept for the
convergence figure and for reading the objective and stopping time of any
other S from one run (experiments/tm_ils_<stem>_<tag>_trace.csv).
"""
import os, sys, csv, re, ast, shutil, subprocess, time
from concurrent.futures import ThreadPoolExecutor, as_completed

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "experiments"))
os.chdir(REPO)

from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES
PATHS = dict(ALL_INSTANCES + EXPANSION_INSTANCES)
ORDER = ["R101 (50)", "R101 (100)", "R1_2_1 (200)", "C101 (50)", "C101 (100)",
         "C1_2_1 (200)", "RC1_2_1 (200)", "R102 (100)", "R104 (100)",
         "C104 (100)", "RC104 (100)", "PR11 (48)", "PR15 (240)", "PR10 (288)"]
BUDGET = float(os.environ.get("DESIGN_BUDGET", 14400.0))
WORKERS = int(os.environ.get("DESIGN_WORKERS", 1))
RCL = int(os.environ.get("DESIGN_RCL", 20))
IDLE = int(os.environ.get("DESIGN_IDLE", 100))
CAPDIV = int(os.environ.get("DESIGN_CAPDIV", 3))
OUT = os.environ.get("DESIGN_OUT", "")
EXTRA = os.environ.get("DESIGN_EXTRA", "").split()
TRACE_DIR = os.path.join(HERE, "results", "details", "design_traces")
COLS = ["Instance", "design", "Objective", "Tour", "t_best (s)", "Wall (s)", "stop", "iterations",
        "shakes", "iter_of_best", "accepted", "socp_calls", "socp_ms", "reorder_trimmed",
        "socp_numeric_infeasible", "Route"]


def stem_of(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def run_one(name, design):
    env = dict(os.environ, ILS_INSERT_RATIO="1", ILS_SHAKE_KNAP="6", ILS_NO_RETURN="2",
               ILS_REORDER_W="exch")
    tag = f"design_{design}{OUT}"
    args = [sys.executable, "experiments/run_ils_time_matched.py", "--instance", name,
            "--budget", str(BUDGET), "--init", "R4", "--shake-return", "--shake-backtrack",
            "--sweep-cap-div", str(CAPDIV), "--tag", tag]
    if design == "new":
        env["ILS_SHAKE_STALL"] = "0"
        args += ["--fast-sets", "--scaled-socp", "--reorder-rcl", str(RCL), "--sweep", "enum",
                 "--max-idle-shakes", str(IDLE)]
    else:
        env["ILS_SHAKE_STALL"] = "1000"
        args += ["--max-iter", "13000"]
    args += EXTRA
    t0 = time.time()
    out = subprocess.run(args, capture_output=True, text=True, env=env).stdout
    wall = time.time() - t0
    stem = stem_of(name)
    prefix = f"experiments/tm_ils_{stem}_{tag}"
    if os.path.exists(prefix + "_trace.csv"):
        os.makedirs(TRACE_DIR, exist_ok=True)
        shutil.copy(prefix + "_trace.csv", os.path.join(TRACE_DIR, f"{stem}_{design}{OUT}.csv"))
    m = re.search(r"best obj ([\d.]+)\s+size (\d+)\s+found at ([\d.]+) s \(iter (\d+)\)", out)
    stop = re.search(r"stopped by (\w+)", out)
    itr = re.search(r"(\d+) iterations at", out)
    route = re.search(r"best route: \[([^\]]*)\]", out)
    c = re.search(r"counters: (\{.*\})", out)
    cnt = ast.literal_eval(c.group(1)) if c else {}
    if not m:
        print(out[-3000:])
        return None
    socp_ms = (cnt.get("socp_us", 0) / 1000.0 / cnt["socp_calls"]) if cnt.get("socp_calls") else ""
    return {"Instance": name, "design": design, "Objective": float(m.group(1)), "Tour": int(m.group(2)),
            "t_best (s)": round(float(m.group(3)), 1), "Wall (s)": round(wall, 1),
            "stop": stop.group(1) if stop else "", "iterations": int(itr.group(1)) if itr else "",
            "shakes": cnt.get("kicks", 0), "iter_of_best": int(m.group(4)),
            "accepted": cnt.get("accepted", 0), "socp_calls": cnt.get("socp_calls", 0),
            "socp_ms": round(socp_ms, 2) if socp_ms != "" else "",
            "reorder_trimmed": cnt.get("reorder_trimmed", 0),
            "socp_numeric_infeasible": cnt.get("socp_numeric_infeasible", 0),
            "Route": "-".join(x.strip() for x in route.group(1).split(",")) if route else ""}


def main():
    design = sys.argv[1] if len(sys.argv) > 1 else "new"
    assert design in ("new", "old")
    csv_path = os.path.join(HERE, "results", f"design_{design}{OUT}.csv")
    if os.path.exists(csv_path):
        done = {r["Instance"] for r in csv.DictReader(open(csv_path))}
        print(f"[resume] {len(done)} runs on disk", flush=True)
    else:
        done = set()
        with open(csv_path, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=COLS).writeheader()
    order = ([n.strip() for n in os.environ["DESIGN_INSTANCES"].split(";")]
             if os.environ.get("DESIGN_INSTANCES") else ORDER)
    jobs = [n for n in order if n not in done]
    print(f"[plan] {design}{OUT}: {len(jobs)} runs, {WORKERS} at a time; L_R={RCL} S={IDLE} D={CAPDIV} "
          f"budget {BUDGET:.0f}s extra={EXTRA}", flush=True)
    with ThreadPoolExecutor(max_workers=WORKERS) as ex:
        futs = {ex.submit(run_one, n, design): n for n in jobs}
        for fut in as_completed(futs):
            name = futs[fut]
            try:
                row = fut.result()
            except Exception as exc:
                print(f"[warn] {name}: {exc}", flush=True); continue
            if row is None:
                print(f"[warn] {name}: no result parsed", flush=True); continue
            with open(csv_path, "a", newline="") as f:
                csv.DictWriter(f, fieldnames=COLS).writerow(row); f.flush(); os.fsync(f.fileno())
            print(f"[saved] {name}: obj={row['Objective']:.2f} tour={row['Tour']} t_best={row['t_best (s)']} "
                  f"wall={row['Wall (s)']} shakes={row['shakes']} stop={row['stop']}", flush=True)
    print(f"\nDONE -> {csv_path}", flush=True)


if __name__ == "__main__":
    main()
