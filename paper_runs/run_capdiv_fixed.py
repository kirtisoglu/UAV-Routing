"""Shake cap divisor sweep with the speed fixed at v_mr (paper Table 12, right block).

The same grid as run_capdiv.py, D in {3, 6, 12} by three ILS seeds on all
fourteen instances, with --fixed-speed so the drone flies every leg at the
maximum-range speed and loitering is its only timing lever. Table 12 contrasts
the two blocks: with the speed fixed an insertion can no longer be absorbed by
flying faster, so both the number of scheduled targets and the correlation
between that number and the objective should fall.

Each finished run is appended to the CSV at once and re-running resumes from it.

Outputs:
  paper_runs/results/capdiv_fixed.csv
"""
import os, sys, csv, re, ast, subprocess, time
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
_only = os.environ.get("T12_INSTANCES")          # ';'-separated subset
if _only:
    ORDER = [x.strip() for x in _only.split(";") if x.strip()]
CAPDIVS = (3, 6, 12)
SEEDS_ENV = os.environ.get("T12_SEEDS")            # e.g. "3,4,5" to add seeds
SEEDS = tuple(int(x) for x in SEEDS_ENV.split(",")) if SEEDS_ENV else (0, 1, 2)
BUDGET = float(os.environ.get("T12_BUDGET", 600.0))
WORKERS = int(os.environ.get("T12_WORKERS", 6))
CSV_PATH = os.path.join(HERE, "results", "capdiv_fixed.csv")
CSV_COLS = ["Instance", "cap_div", "seed", "Objective", "Tour", "t_best (s)",
            "proposals", "shakes", "accepted", "socp_calls", "budget (s)", "Wall (s)"]


def stem_of(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def run_one(name, div, seed):
    tag = f"t12_d{div}"
    t0 = time.time()
    out = subprocess.run(
        [sys.executable, "experiments/run_ils_time_matched.py",
         "--instance", name, "--budget", str(BUDGET), "--init", "R4",
         "--sweep-cap-div", str(div), "--seed-offset", str(seed), "--tag", tag,
         "--fixed-speed"],
        capture_output=True, text=True).stdout
    wall = time.time() - t0
    prefix = f"experiments/tm_ils_{stem_of(name)}_{tag}" + (f"_w{seed}" if seed else "")
    for suf in ("_trace.csv", "_summary.csv", "_trace.png"):
        if os.path.exists(prefix + suf):
            os.remove(prefix + suf)
    m = re.search(r"best obj ([\d.]+)\s+size (\d+)\s+found at ([\d.]+) s \(iter (\d+)\)", out)
    c = re.search(r"counters: (\{.*\})", out)
    if not m:
        return None
    cnt = ast.literal_eval(c.group(1)) if c else {}
    props = sum(cnt.get(f"prop_{o}", 0) for o in ("add", "replace", "swap", "two_opt"))
    return {"Instance": name, "cap_div": div, "seed": seed,
            "Objective": float(m.group(1)), "Tour": int(m.group(2)),
            "t_best (s)": round(float(m.group(3)), 1), "proposals": props,
            "shakes": cnt.get("kicks", 0), "accepted": cnt.get("accepted", 0),
            "socp_calls": cnt.get("socp_calls", 0),
            "budget (s)": int(BUDGET), "Wall (s)": round(wall, 1)}


def main():
    if os.path.exists(CSV_PATH):
        with open(CSV_PATH) as f:
            done = {(r["Instance"], int(r["cap_div"]), int(r["seed"]))
                    for r in csv.DictReader(f)}
        print(f"[resume] {len(done)} runs on disk", flush=True)
    else:
        done = set()
        with open(CSV_PATH, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=CSV_COLS).writeheader()
    jobs = [(n, d, s) for n in ORDER for d in CAPDIVS for s in SEEDS
            if (n, d, s) not in done]
    print(f"[plan] {len(jobs)} runs, {WORKERS} at a time, {BUDGET:.0f}s each, fixed speed", flush=True)
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
                csv.DictWriter(f, fieldnames=CSV_COLS).writerow(row)
                f.flush(); os.fsync(f.fileno())
            k += 1
            print(f"[saved {k}/{len(jobs)}] {j[0]} d{j[1]}/s{j[2]}: "
                  f"obj={row['Objective']:.2f} tour={row['Tour']}", flush=True)
    print(f"\n{'=' * 62}\nDONE -> {CSV_PATH}", flush=True)


if __name__ == "__main__":
    main()
