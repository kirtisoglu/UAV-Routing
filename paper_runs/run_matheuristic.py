"""Matheuristic vs exact (paper Table 9, E3) and value of speed optimization
(Table 10, E4).

Single-start ILS from R4 at the chosen design of Section 4, one run of
3 600 s per instance on all fourteen instances, one run at a time so that
each run has the whole machine for one hour, as the MISOCP had. On the
instances the exact solver closed (gap 0 in misocp_s1.csv) the run ends as
soon as it reaches the proven optimum. The shake threshold theta is selected
from the E2 results (paper_runs/results/theta.csv): the value whose mean
objective over the seeds has the smallest mean relative deviation from the
per-instance best, ties going to the smaller value. The MISOCP side of
Table 9 comes from paper_runs/results/misocp_s1.csv.

  python3 paper_runs/run_matheuristic.py variable    E3, speed free in [v_min, v_max]
  python3 paper_runs/run_matheuristic.py fixed       E4, speed fixed at v_mr (loitering allowed)
  python3 paper_runs/run_matheuristic.py noloiter    Table 12, L_ij = d_ij (speed free, no loitering)

Each finished run is appended to the CSV at once and re-running resumes
from it. The best-seen trace of every run is kept for the convergence
figure.

Outputs:
  paper_runs/results/ils_variable.csv, paper_runs/results/ils_fixed.csv
  paper_runs/results/details/ils_traces/<instance>_<variable|fixed>.csv
"""
import os, sys, csv, re, ast, shutil, subprocess, time, statistics as st
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
BUDGET = float(os.environ.get("E3_BUDGET", 3600.0))
WORKERS = int(os.environ.get("E3_WORKERS", 1))   # one run at a time: the whole machine per run, like the MISOCP
THETA_CSV = os.path.join(HERE, "results", "theta.csv")
MISOCP_CSV = os.path.join(HERE, "results", "misocp_s1.csv")
TRACE_DIR = os.path.join(HERE, "results", "details", "ils_traces")
CSV_COLS = ["Instance", "mode", "theta", "cap_div", "design", "init_obj", "init_size", "Objective", "Tour",
            "t_best (s)", "proposals", "shakes", "accepted", "socp_calls",
            "budget (s)", "Wall (s)", "Route"]


def select_theta(path=THETA_CSV):
    """theta with the smallest mean relative deviation of its seed-mean
    objective from the per-instance best seed-mean; ties to the smaller."""
    rows = list(csv.DictReader(open(path)))
    by = {}
    for r in rows:
        by.setdefault((r["Instance"], int(r["theta"])), []).append(float(r["Objective"]))
    thetas = sorted({t for _, t in by})
    insts = sorted({i for i, _ in by})
    dev = {t: [] for t in thetas}
    for i in insts:
        means = {t: st.mean(by[(i, t)]) for t in thetas if (i, t) in by}
        if len(means) < len(thetas):
            continue
        best = max(means.values())
        for t in thetas:
            dev[t].append(100 * (best - means[t]) / best)
    score = {t: st.mean(dev[t]) for t in thetas if dev[t]}
    theta = min(score, key=lambda t: (round(score[t], 6), t))
    print("[theta] mean deviation from per-instance best (%): "
          + ", ".join(f"{t}: {score[t]:.2f}" for t in thetas) + f" -> theta = {theta}", flush=True)
    return theta


def stem_of(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def proven_optima(path=MISOCP_CSV):
    """instance -> MISOCP objective, for the instances the exact solver closed."""
    return {r["Instance"]: float(r["Objective"]) for r in csv.DictReader(open(path))
            if float(r["Gap (%)"]) == 0.0}


def run_one(name, mode, theta):
    tag = f"e3_{mode}{os.environ.get('E3_OUT', '')}"
    args = [sys.executable, "experiments/run_ils_time_matched.py",
            "--instance", name, "--budget", str(BUDGET), "--init", "R4",
            "--theta", str(theta), "--tag", tag]
    opt = proven_optima().get(name)
    if opt is not None and mode == "variable":
        args += ["--target", str(opt), "--stop-at-target"]
    if os.environ.get("E3_CAP_DIV"):
        args += ["--sweep-cap-div", os.environ["E3_CAP_DIV"]]
    if os.environ.get("E3_EXTRA_ARGS"):          # design switches, e.g. "--local-search phases --pair-weight ratio --rcl 5"
        args += os.environ["E3_EXTRA_ARGS"].split()
    if mode == "fixed":
        args.append("--fixed-speed")
    if mode == "noloiter":
        args.append("--no-loiter")
    t0 = time.time()
    out = subprocess.run(args, capture_output=True, text=True).stdout
    wall = time.time() - t0
    stem = stem_of(name)
    prefix = f"experiments/tm_ils_{stem}_{tag}"
    trace = prefix + "_trace.csv"
    if os.path.exists(trace):
        os.makedirs(TRACE_DIR, exist_ok=True)
        shutil.move(trace, os.path.join(TRACE_DIR, f"{stem}_{mode}{os.environ.get('E3_OUT', '')}.csv"))
    for suf in ("_summary.csv", "_trace.png"):
        if os.path.exists(prefix + suf):
            os.remove(prefix + suf)
    m = re.search(r"best obj ([\d.]+)\s+size (\d+)\s+found at ([\d.]+) s \(iter (\d+)\)", out)
    init = re.search(r"start 0 \(R4(?:, \w+)?\) init obj ([\d.]+) size (\d+)", out)
    route = re.search(r"best route: \[([^\]]*)\]", out)
    c = re.search(r"counters: (\{.*\})", out)
    cnt = ast.literal_eval(c.group(1)) if c else {}
    if not m:
        return None
    props = sum(cnt.get(f"prop_{o}", 0) for o in ("add", "replace", "swap", "two_opt"))
    return {"Instance": name, "mode": mode, "theta": theta,
            "cap_div": int(os.environ.get("E3_CAP_DIV", 6)),
            "design": os.environ.get("E3_EXTRA_ARGS", "random+omega"),
            "init_obj": float(init.group(1)) if init else None,
            "init_size": int(init.group(2)) if init else None,
            "Objective": float(m.group(1)), "Tour": int(m.group(2)),
            "t_best (s)": round(float(m.group(3)), 1), "proposals": props,
            "shakes": cnt.get("kicks", 0), "accepted": cnt.get("accepted", 0),
            "socp_calls": cnt.get("socp_calls", 0),
            "budget (s)": int(BUDGET), "Wall (s)": round(wall, 1),
            "Route": "-".join(x.strip() for x in route.group(1).split(",")) if route else ""}


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "variable"
    assert mode in ("variable", "fixed", "noloiter")
    # The shake fires when the four feasible sets of the route are exhausted, so the
    # threshold no longer exists in the design; the driver keeps the column for the CSV.
    theta = int(os.environ.get("E3_THETA", 10**12))
    csv_path = os.path.join(HERE, "results", f"ils_{mode}{os.environ.get('E3_OUT', '')}.csv")   # E3_OUT: suffix for a variant campaign
    if os.path.exists(csv_path):
        with open(csv_path) as f:
            done = {r["Instance"] for r in csv.DictReader(f)}
        print(f"[resume] {len(done)} runs on disk", flush=True)
    else:
        done = set()
        with open(csv_path, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=CSV_COLS).writeheader()
    order = [n.strip() for n in os.environ["E3_INSTANCES"].split(";")] if os.environ.get("E3_INSTANCES") else ORDER
    jobs = [n for n in order if n not in done]
    print(f"[plan] {mode}: {len(jobs)} runs, {WORKERS} at a time, {BUDGET:.0f}s each, theta={theta}", flush=True)
    k = 0
    with ThreadPoolExecutor(max_workers=WORKERS) as ex:
        futs = {ex.submit(run_one, n, mode, theta): n for n in jobs}
        for fut in as_completed(futs):
            name = futs[fut]
            try:
                row = fut.result()
            except Exception as exc:
                print(f"[warn] {name}: {exc}", flush=True)
                continue
            if row is None:
                print(f"[warn] {name}: no result parsed", flush=True)
                continue
            with open(csv_path, "a", newline="") as f:
                csv.DictWriter(f, fieldnames=CSV_COLS).writerow(row)
                f.flush(); os.fsync(f.fileno())
            k += 1
            print(f"[saved {k}/{len(jobs)}] {name}: obj={row['Objective']:.2f} "
                  f"tour={row['Tour']} t_best={row['t_best (s)']}", flush=True)
    print(f"\n{'=' * 62}\nDONE -> {csv_path}", flush=True)


if __name__ == "__main__":
    main()
