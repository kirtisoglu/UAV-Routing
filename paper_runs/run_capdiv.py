"""Shake cap divisor D, c = ceil(k/D) — paper Table 9.

One single-start run per cell at the design of Section 4: R4 start, the four
operators drawn uniformly, reference shaking with fallback, and termination on
maxIter iterations without an improvement rather than a wall-clock budget.
Seed 0 only, as the paper reports a single seed.

    D in {3, 6, 12}  x  fourteen instances  =  42 runs

Four at a time, since the fixed-tour SOCP is single threaded. Objectives and
iteration counts are deterministic given the seed, so the concurrency affects
only the wall clock, which this table does not report. Each finished run is
appended at once and re-running resumes from the CSV. The best-seen trace of
every run is kept for the cap-divisor figure.

Outputs:
  paper_runs/results/capdiv.csv
  paper_runs/results/details/capdiv_traces/<instance>_d<D>.csv
"""
import os, sys, csv, re, ast, shutil, subprocess, time
from concurrent.futures import ThreadPoolExecutor, as_completed

HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "experiments")); os.chdir(REPO)
from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES
PATHS = dict(ALL_INSTANCES + EXPANSION_INSTANCES)

ORDER = ["R101 (50)", "R101 (100)", "R1_2_1 (200)", "C101 (50)", "C101 (100)",
         "C1_2_1 (200)", "RC1_2_1 (200)", "R102 (100)", "R104 (100)",
         "C104 (100)", "RC104 (100)", "PR11 (48)", "PR15 (240)", "PR10 (288)"]
CAPDIVS = (3, 6, 12)   # D=6 is the default and is seeded from the main sweep
FIG_INSTANCE = "PR15 (240)"   # the instance Figure 2 plots
MAX_ITER = int(os.environ.get("T9_MAX_ITER", 15000))
WORKERS = int(os.environ.get("T9_WORKERS", 4))
CSV_PATH = os.path.join(HERE, "results", "capdiv.csv")
TRACE_DIR = os.path.join(HERE, "results", "details", "capdiv_traces")
COLS = ["Instance", "cap_div", "Objective", "Tour", "t_best (s)", "iterations",
        "shakes", "accepted", "socp_calls", "Wall (s)"]

stem_of = lambda n: re.sub(r"[^a-z0-9]+", "_", n.lower()).strip("_")


def run_one(name, div):
    tag = f"t9_d{div}"
    t0 = time.time()
    cmd = [sys.executable, "experiments/run_ils_time_matched.py",
           "--instance", name, "--max-iter", str(MAX_ITER), "--budget", "14400",
           "--init", "R4", "--shake-return", "--shake-backtrack",
           "--sweep-cap-div", str(div), "--tag", tag]
    # Figure 2 wants the perturbation dynamics of PR15 at each divisor, and these
    # are the runs that fill that row of Table 9 anyway, so take both at once
    if name == FIG_INSTANCE:
        cmd += ["--dynamics-out",
                f"paper_runs/results/details/dynamics/pr15_d{div}.csv"]
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    wall = time.time() - t0
    stem = stem_of(name); prefix = f"experiments/tm_ils_{stem}_{tag}"
    if os.path.exists(prefix + "_trace.csv"):
        os.makedirs(TRACE_DIR, exist_ok=True)
        shutil.move(prefix + "_trace.csv", os.path.join(TRACE_DIR, f"{stem}_d{div}.csv"))
    for suf in ("_summary.csv", "_trace.png"):
        if os.path.exists(prefix + suf):
            os.remove(prefix + suf)
    m = re.search(r"best obj ([\d.]+)\s+size (\d+)\s+found at ([\d.]+) s \(iter (\d+)\)", out)
    it = re.search(r"after (\d+) iterations", out)
    c = re.search(r"counters: (\{.*\})", out)
    if not m:
        return None
    cnt = ast.literal_eval(c.group(1)) if c else {}
    return {"Instance": name, "cap_div": div, "Objective": float(m.group(1)),
            "Tour": int(m.group(2)), "t_best (s)": round(float(m.group(3)), 1),
            "iterations": int(it.group(1)) if it else 0,
            "shakes": cnt.get("kicks", 0), "accepted": cnt.get("accepted", 0),
            "socp_calls": cnt.get("socp_calls", 0), "Wall (s)": round(wall, 1)}


def main():
    if os.path.exists(CSV_PATH):
        with open(CSV_PATH) as f:
            done = {(r["Instance"], int(r["cap_div"])) for r in csv.DictReader(f)}
        print(f"[resume] {len(done)} runs on disk", flush=True)
    else:
        done = set()
        os.makedirs(os.path.dirname(CSV_PATH), exist_ok=True)
        with open(CSV_PATH, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=COLS).writeheader()
    jobs = [(n, d) for n in ORDER for d in CAPDIVS if (n, d) not in done]
    # Figure 2 waits on this instance, so take its cells before the rest
    jobs.sort(key=lambda j: j[0] != FIG_INSTANCE)
    print(f"[plan] {len(jobs)} runs, {WORKERS} at a time, maxIter {MAX_ITER}", flush=True)
    k = 0
    with ThreadPoolExecutor(max_workers=WORKERS) as ex:
        futs = {ex.submit(run_one, *j): j for j in jobs}
        for fut in as_completed(futs):
            j = futs[fut]
            try:
                row = fut.result()
            except Exception as exc:
                print(f"[warn] {j}: {exc}", flush=True); continue
            if row is None:
                print(f"[warn] {j}: no result parsed", flush=True); continue
            with open(CSV_PATH, "a", newline="") as f:
                csv.DictWriter(f, fieldnames=COLS).writerow(row)
                f.flush(); os.fsync(f.fileno())
            k += 1
            print(f"[{k}/{len(jobs)}] {j[0]} D={j[1]}: {row['Objective']:,.2f} "
                  f"tour {row['Tour']} shakes {row['shakes']}", flush=True)
    print(f"\nDONE -> {CSV_PATH}", flush=True)


if __name__ == "__main__":
    main()
