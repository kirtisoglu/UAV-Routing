"""Parameter analysis of the shake cap divisor jointly with theta (paper Section 5).

The shake removes at most c = max(2, ceil(k / D)) targets from a route of k
targets. Single-start ILS from R4 at the chosen design, 600 s per run, on
all fourteen instances, over the full grid theta in CAP_THETAS x D in
CAP_DIVS and the seeds in CAP_SEEDS. The column D = 6 is the E2 grid and is
read from paper_runs/results/theta.csv rather than rerun, so the two files
together form the 4 x 2 factorial over theta and D in {3, 6}.

  CAP_SEEDS=0      python3 paper_runs/run_cap.py     (default: seed 0 only)
  CAP_SEEDS=0,1,2  python3 paper_runs/run_cap.py     (later, if wanted)

Each finished run is appended to the CSV at once and re-running resumes.

Outputs:
  paper_runs/results/cap.csv
"""
import os, sys, csv, re, ast, subprocess, time
from concurrent.futures import ThreadPoolExecutor, as_completed

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "experiments"))
os.chdir(REPO)

from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES
from run_matheuristic import ORDER, stem_of
CAP_DIVS = tuple(int(x) for x in os.environ.get("CAP_DIVS", "3").split(","))
CAP_THETAS = tuple(int(x) for x in os.environ.get("CAP_THETAS", "100,300,1000,3000").split(","))
SEEDS = tuple(int(x) for x in os.environ.get("CAP_SEEDS", "0").split(","))
BUDGET = float(os.environ.get("CAP_BUDGET", 600.0))
WORKERS = int(os.environ.get("CAP_WORKERS", 3))
CSV_PATH = os.path.join(HERE, "results", "cap.csv")
CSV_COLS = ["Instance", "cap_div", "theta", "seed", "Objective", "Tour", "t_best (s)",
            "proposals", "shakes", "accepted", "socp_calls", "budget (s)", "Wall (s)"]


def run_one(name, div, theta, seed):
    tag = f"cap{div}_th{theta}"
    t0 = time.time()
    out = subprocess.run(
        [sys.executable, "experiments/run_ils_time_matched.py",
         "--instance", name, "--budget", str(BUDGET), "--init", "R4",
         "--theta", str(theta), "--seed-offset", str(seed),
         "--sweep-cap-div", str(div), "--tag", tag],
        capture_output=True, text=True).stdout
    wall = time.time() - t0
    prefix = f"experiments/tm_ils_{stem_of(name)}_{tag}" + (f"_w{seed}" if seed else "")
    for suf in ("_trace.csv", "_summary.csv", "_trace.png"):
        if os.path.exists(prefix + suf):
            os.remove(prefix + suf)
    m = re.search(r"best obj ([\d.]+)\s+size (\d+)\s+found at (\d+) s \(iter (\d+)\)", out)
    c = re.search(r"counters: (\{.*\})", out)
    cnt = ast.literal_eval(c.group(1)) if c else {}
    if not m:
        return None
    props = sum(cnt.get(f"prop_{o}", 0) for o in ("add", "replace", "swap", "two_opt"))
    return {"Instance": name, "cap_div": div, "theta": theta, "seed": seed,
            "Objective": float(m.group(1)), "Tour": int(m.group(2)),
            "t_best (s)": int(m.group(3)), "proposals": props,
            "shakes": cnt.get("kicks", 0), "accepted": cnt.get("accepted", 0),
            "socp_calls": cnt.get("socp_calls", 0),
            "budget (s)": int(BUDGET), "Wall (s)": round(wall, 1)}


def main():
    if os.path.exists(CSV_PATH):
        with open(CSV_PATH) as f:
            done = {(r["Instance"], int(r["theta"]), int(r["cap_div"]), int(r["seed"]))
                    for r in csv.DictReader(f)}
        print(f"[resume] {len(done)} runs on disk", flush=True)
    else:
        done = set()
        with open(CSV_PATH, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=CSV_COLS).writeheader()
    jobs = [(n, t, d, s) for n in ORDER for t in CAP_THETAS for d in CAP_DIVS for s in SEEDS
            if (n, t, d, s) not in done]
    print(f"[plan] {len(jobs)} runs, {WORKERS} at a time, {BUDGET:.0f}s each, "
          f"thetas {CAP_THETAS}, divisors {CAP_DIVS}, seeds {SEEDS}", flush=True)
    k = 0
    with ThreadPoolExecutor(max_workers=WORKERS) as ex:
        futs = {ex.submit(run_one, n, d, t, s): (n, t, d, s) for n, t, d, s in jobs}
        for fut in as_completed(futs):
            name, t, d, s = futs[fut]
            try:
                row = fut.result()
            except Exception as exc:
                print(f"[warn] {name} th{t}/D{d}/s{s}: {exc}", flush=True)
                continue
            if row is None:
                print(f"[warn] {name} th{t}/D{d}/s{s}: no result parsed", flush=True)
                continue
            with open(CSV_PATH, "a", newline="") as f:
                csv.DictWriter(f, fieldnames=CSV_COLS).writerow(row)
                f.flush(); os.fsync(f.fileno())
            k += 1
            print(f"[saved {k}/{len(jobs)}] {name} th{t}/D{d}/s{s}: obj={row['Objective']:.2f} "
                  f"tour={row['Tour']} shakes={row['shakes']}", flush=True)
    print(f"\n{'=' * 62}\nDONE -> {CSV_PATH}", flush=True)


if __name__ == "__main__":
    main()
