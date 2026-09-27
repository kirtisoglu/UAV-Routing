"""Initial-tour selection (paper Table 8): does the starting tour matter?

Five single starts per instance — R1, R2 and three random draws of R3 — on
all fourteen instances: 70 runs. The R4 column is not re-run here; it is the
D=3 column of the cap-divisor sweep, which is a single start from R4 under
exactly this protocol, and is merged in when the table is written.

The ILS runs at the design of Section 4: four operators drawn uniformly,
reference shaking with fallback, cap ceil(k/3), and termination on maxIter
iterations without an improvement rather than a wall-clock budget. The table
reports objectives only, so the runs go four at a time; the fixed-tour SOCP
is single-threaded.

Each finished run is fsynced to the CSV immediately and re-running resumes
from it. Per-run trace files are deleted; only the objective is kept.

Outputs:
  paper_runs/results/initial_tours.csv
"""
import os, sys, csv, re, subprocess, time
from concurrent.futures import ThreadPoolExecutor, as_completed

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "experiments"))
os.chdir(REPO)

from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES
# (the driver is invoked as a subprocess; nothing is imported from it)
PATHS = dict(ALL_INSTANCES + EXPANSION_INSTANCES)
ORDER = ["R101 (50)", "R101 (100)", "R1_2_1 (200)", "C101 (50)", "C101 (100)",
         "C1_2_1 (200)", "RC1_2_1 (200)", "R102 (100)", "R104 (100)",
         "C104 (100)", "RC104 (100)", "PR11 (48)", "PR15 (240)", "PR10 (288)"]

MAX_ITER = int(os.environ.get("T8_MAX_ITER", 15000))
WORKERS = int(os.environ.get("T8_WORKERS", 4))
R3_SEEDS = (1, 2, 3)
CSV_PATH = os.path.join(HERE, "results", "initial_tours.csv")
CSV_COLS = ["Instance", "Start", "seed", "Objective", "Tour",
            "t_best (s)", "proposals", "max_iter", "Wall (s)"]

_lock_note = "one writer thread appends; each row is fsynced"


def jobs():
    out = []
    for name in ORDER:
        out.append((name, "R1", 0))
        out.append((name, "R2", 1))
        for s in R3_SEEDS:
            out.append((name, "R3", s))
    return out


def run_one(name, init, seed):
    tag = f"t8_{init}{seed}"
    t0 = time.time()
    out = subprocess.run(
        [sys.executable, "experiments/run_ils_time_matched.py",
         "--instance", name, "--max-iter", str(MAX_ITER), "--budget", "14400",
         "--init", init, "--init-seed", str(seed), "--tag", tag,
         "--shake-return", "--shake-backtrack"],
        capture_output=True, text=True).stdout
    wall = time.time() - t0
    stem = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
    for suf in ("_trace.csv", "_summary.csv", "_trace.png"):
        q = f"experiments/tm_ils_{stem}_{tag}{suf}"
        if os.path.exists(q):
            os.remove(q)
    m = re.search(r"best obj ([\d.]+)\s+size (\d+)\s+found at ([\d.]+) s "
                  r"\(iter (\d+)\)", out)
    if not m:
        return None
    return {"Instance": name, "Start": init, "seed": seed,
            "Objective": float(m.group(1)), "Tour": int(m.group(2)),
            "t_best (s)": round(float(m.group(3)), 1), "proposals": int(m.group(4)),
            "max_iter": MAX_ITER, "Wall (s)": round(wall, 1)}


def main():
    if os.path.exists(CSV_PATH):
        with open(CSV_PATH) as f:
            done = {(r["Instance"], r["Start"], int(r["seed"]))
                    for r in csv.DictReader(f)}
        print(f"[resume] {len(done)} runs on disk", flush=True)
    else:
        done = set()
        with open(CSV_PATH, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=CSV_COLS).writeheader()

    todo = [j for j in jobs() if j not in done]
    print(f"[plan] {len(todo)} runs, {WORKERS} at a time, "
          f"maxIter {MAX_ITER}", flush=True)
    n = 0
    with ThreadPoolExecutor(max_workers=WORKERS) as ex:
        futs = {ex.submit(run_one, *j): j for j in todo}
        for fut in as_completed(futs):
            name, init, seed = futs[fut]
            try:
                row = fut.result()
            except Exception as exc:
                print(f"[warn] {name} {init}/{seed}: {exc}", flush=True)
                continue
            if row is None:
                print(f"[warn] {name} {init}/{seed}: no result parsed", flush=True)
                continue
            with open(CSV_PATH, "a", newline="") as f:
                csv.DictWriter(f, fieldnames=CSV_COLS).writerow(row)
                f.flush(); os.fsync(f.fileno())
            n += 1
            print(f"[saved {n}/{len(todo)}] {name} {init}"
                  f"{'' if init in ('R1','R4') else '/'+str(seed)}: "
                  f"obj={row['Objective']:.2f} tour={row['Tour']}", flush=True)
    print(f"\n{'='*62}\nDONE -> {CSV_PATH}", flush=True)


if __name__ == "__main__":
    main()
