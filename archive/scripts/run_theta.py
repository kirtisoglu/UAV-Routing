"""Parameter analysis of the shake threshold theta (paper Section 5, E2).

Single-start ILS from R4 at the chosen design of Section 4 (four operators
drawn uniformly, four checks, reward-weighted candidate order, shake with
cap ceil(k/6), strict acceptance), 600 s per run, on all fourteen instances:

    theta in {100, 300, 1000, 3000}  x  three ILS seeds (seed offsets 0, 1, 2)

168 runs, six at a time (the fixed-tour SOCP is single-threaded). Each
finished run is appended to the CSV at once and re-running resumes from it.
The best-seen trace of every run is kept for the convergence figure.

Outputs:
  paper_runs/results/theta.csv
  paper_runs/results/details/theta_traces/<instance>_th<theta>_s<seed>.csv
"""
import os, sys, csv, re, ast, shutil, subprocess, time
from concurrent.futures import ThreadPoolExecutor, as_completed

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "experiments"))
os.chdir(REPO)

from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES
PATHS = dict(ALL_INSTANCES + EXPANSION_INSTANCES)
ORDER = ["R101 (50)", "R101 (100)", "R1_2_1 (200)", "C101 (50)", "C101 (100)",
         "C1_2_1 (200)", "RC1_2_1 (200)", "R102 (100)", "R104 (100)",
         "C104 (100)", "RC104 (100)", "PR11 (48)", "PR15 (240)", "PR10 (288)"]
THETAS = (100, 300, 1000, 3000)
SEEDS = (0, 1, 2)
BUDGET = float(os.environ.get("E2_BUDGET", 600.0))
WORKERS = int(os.environ.get("E2_WORKERS", 6))
CSV_PATH = os.path.join(HERE, "results", "theta.csv")
TRACE_DIR = os.path.join(HERE, "results", "details", "theta_traces")
CSV_COLS = ["Instance", "theta", "seed", "Objective", "Tour", "t_best (s)",
            "proposals", "shakes", "accepted", "socp_calls", "budget (s)", "Wall (s)"]


def stem_of(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def run_one(name, theta, seed):
    tag = f"e2_th{theta}"
    t0 = time.time()
    out = subprocess.run(
        [sys.executable, "experiments/run_ils_time_matched.py",
         "--instance", name, "--budget", str(BUDGET), "--init", "R4",
         "--theta", str(theta), "--seed-offset", str(seed), "--tag", tag],
        capture_output=True, text=True).stdout
    wall = time.time() - t0
    stem = stem_of(name)
    full_tag = tag + (f"_w{seed}" if seed else "")
    prefix = f"experiments/tm_ils_{stem}_{full_tag}"
    trace = prefix + "_trace.csv"
    if os.path.exists(trace):
        os.makedirs(TRACE_DIR, exist_ok=True)
        shutil.move(trace, os.path.join(TRACE_DIR, f"{stem}_th{theta}_s{seed}.csv"))
    for suf in ("_summary.csv", "_trace.png"):
        if os.path.exists(prefix + suf):
            os.remove(prefix + suf)
    m = re.search(r"best obj ([\d.]+)\s+size (\d+)\s+found at (\d+) s \(iter (\d+)\)", out)
    c = re.search(r"counters: (\{.*\})", out)
    cnt = ast.literal_eval(c.group(1)) if c else {}
    if not m:
        return None
    props = sum(cnt.get(f"prop_{o}", 0) for o in ("add", "replace", "swap", "two_opt"))
    return {"Instance": name, "theta": theta, "seed": seed,
            "Objective": float(m.group(1)), "Tour": int(m.group(2)),
            "t_best (s)": int(m.group(3)), "proposals": props,
            "shakes": cnt.get("kicks", 0), "accepted": cnt.get("accepted", 0),
            "socp_calls": cnt.get("socp_calls", 0),
            "budget (s)": int(BUDGET), "Wall (s)": round(wall, 1)}


def main():
    if os.path.exists(CSV_PATH):
        with open(CSV_PATH) as f:
            done = {(r["Instance"], int(r["theta"]), int(r["seed"]))
                    for r in csv.DictReader(f)}
        print(f"[resume] {len(done)} runs on disk", flush=True)
    else:
        done = set()
        with open(CSV_PATH, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=CSV_COLS).writeheader()
    jobs = [(n, t, s) for n in ORDER for t in THETAS for s in SEEDS if (n, t, s) not in done]
    print(f"[plan] {len(jobs)} runs, {WORKERS} at a time, {BUDGET:.0f}s each", flush=True)
    k = 0
    with ThreadPoolExecutor(max_workers=WORKERS) as ex:
        futs = {ex.submit(run_one, *j): j for j in jobs}
        for fut in as_completed(futs):
            name, theta, seed = futs[fut]
            try:
                row = fut.result()
            except Exception as exc:
                print(f"[warn] {name} th{theta}/s{seed}: {exc}", flush=True)
                continue
            if row is None:
                print(f"[warn] {name} th{theta}/s{seed}: no result parsed", flush=True)
                continue
            with open(CSV_PATH, "a", newline="") as f:
                csv.DictWriter(f, fieldnames=CSV_COLS).writerow(row)
                f.flush(); os.fsync(f.fileno())
            k += 1
            print(f"[saved {k}/{len(jobs)}] {name} th{theta}/s{seed}: "
                  f"obj={row['Objective']:.2f} tour={row['Tour']} shakes={row['shakes']}", flush=True)
    print(f"\n{'=' * 62}\nDONE -> {CSV_PATH}", flush=True)


if __name__ == "__main__":
    main()
