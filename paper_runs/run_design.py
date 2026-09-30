"""Campaign driver: every matheuristic number of Section 5 comes from this script.

One run of experiments/run_ils_time_matched.py per instance at the design of
Section 4 (--fast-sets --scaled-socp --reorder-rcl L_r --sweep enum
--max-idle-shakes S, cap divisor D, single start; every move weighted by the four
tiers of Section 4.3, ILS_REORDER_W=tiers, and the shake applying the next removal
of the enumeration, ILS_SHAKE_KNAP=0), the result appended to a CSV
under paper_runs/results and the best-seen trace copied next to it. The tables
are then written into the manuscript by paper_runs/fill_tables.py.

    python3 paper_runs/run_design.py new     -> results/design_new.csv   (the reference run)
    python3 paper_runs/run_design.py old     -> results/design_old.csv   (previous design, comparison only)

Knobs (environment variables; every one defaults to the paper's setting):
  DESIGN_INSTANCES  "R104 (100);PR15 (240)"   instances to run (default: all fourteen)
  DESIGN_OUT        suffix of the CSV and trace names so that a variant does not
                    overwrite the reference run: _fixed, _noloiter, _D6, _R1, ...
  DESIGN_EXTRA      flags appended to every run: "--fixed-speed" or "--no-loiter"
  DESIGN_CAPDIV     D   (default 3)          DESIGN_RCL   L_r (default 20)
  DESIGN_IDLE       S   (default 100)
  DESIGN_INIT       start heuristic R1|R2|R3|R4 (default R4)
  DESIGN_INIT_SEED  seed of the random start R3 (default 1)
  DESIGN_DYNAMICS   1 = also write the per-iteration trace (iter, wall_s, f_curr,
                    f_best, kick) to results/details/dynamics/<stem>_<design><OUT>.csv,
                    the input of the perturbation figure (fig:ils-capdiv)
  DESIGN_ETA        energy budget scaling eta (default 1); name the output by it, e.g.
                    DESIGN_ETA=0.75 DESIGN_OUT=_eta075 (the value is recorded in the extra column)
  DESIGN_REORDER_W  move weights (default tiers, the paper since 30 Sept 2026; exch is the
                    previous design; the test modes are in experiments/TEST_RUNBOOK.md)
  DESIGN_SHAKE_KNAP removals the shake looks ahead over (default 0 = the next removal only,
                    the paper; 6 is the previous design's look-ahead)
  DESIGN_SCALED     0 = the subproblem in physical units instead of the scaled units of
                    Section 4.1 (default 1); only for the solve-time comparison of the
                    Parameter analysis, recorded as "physical-units" in the extra column
  DESIGN_BUDGET     wall-clock safeguard per run, seconds (default 14400)
  DESIGN_WORKERS    parallel runs (default 1; keep 1 whenever t_best or Run is reported)

Which run fills which table (the exact commands are in experiments/RERUN_PLAN.md):
  design_new.csv                  tab:matheuristic-vs-exact, fig:ils-convergence, the
                                  D = 3 block of tab:theta, the R4 column of
                                  tab:initial-tour, the "variable speed" block of
                                  tab:fixed-speed, the "loitering allowed" block of tab:coverage
  design_new_rep2.csv             tab:matheuristic-vs-exact, replication 2 (DESIGN_EXTRA="--seed-offset 1" DESIGN_OUT=_rep2)
  design_new_fixed.csv            tab:fixed-speed   (DESIGN_EXTRA="--fixed-speed" DESIGN_OUT=_fixed)
  design_new_noloiter.csv         tab:coverage      (DESIGN_EXTRA="--no-loiter"   DESIGN_OUT=_noloiter)
  design_new_D6.csv, _D12.csv     tab:theta         (DESIGN_CAPDIV=6 DESIGN_OUT=_D6, DESIGN_CAPDIV=12 DESIGN_OUT=_D12)
  design_new_R1.csv, _R2.csv,     tab:initial-tour  (DESIGN_INIT=R1 DESIGN_OUT=_R1, ...;
  _R3s1.csv, _R3s2.csv, _R3s3.csv                    R3 with DESIGN_INIT_SEED=1, 2, 3 and DESIGN_OUT=_R3s1, ...)
  design_new_eta075.csv, _fixed_eta075.csv, _noloiter_eta075.csv, and the same at _eta125
                                  tab:levers-eta    (DESIGN_ETA=0.75 with DESIGN_OUT=_eta075, _fixed_eta075, _noloiter_eta075; 1.25 likewise)
  details/dynamics/pr15_240_new_D{3,6,12}dyn.csv    fig:ils-capdiv
                                  (DESIGN_INSTANCES="PR15 (240)" DESIGN_DYNAMICS=1 DESIGN_CAPDIV=D DESIGN_OUT=_D{D}dyn)

Finished runs are appended to the CSV at once and a re-run resumes from it, so an
interrupted campaign is simply restarted. A CSV written by an earlier version of
this script (other columns) is kept as <name>.v1.csv and a fresh file is started.
Every row records the git commit the run was made with. ILS_LICENSE_GUARD must not
be set: it was a workaround for a size-limited license and silently drops routes.
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
INIT = os.environ.get("DESIGN_INIT", "R4")
INIT_SEED = int(os.environ.get("DESIGN_INIT_SEED", 1))
DYNAMICS = os.environ.get("DESIGN_DYNAMICS", "") not in ("", "0")
ETA = float(os.environ.get("DESIGN_ETA", 1.0))
OUT = os.environ.get("DESIGN_OUT", "")
SHAKE_KNAP_ENV = os.environ.get("DESIGN_SHAKE_KNAP", "0")   # 0 = the next removal only (the paper), 6 = the previous look-ahead
SCALED = os.environ.get("DESIGN_SCALED", "1") != "0"      # 0 = physical units (solve-time comparison only)
REORDER_W = os.environ.get("DESIGN_REORDER_W", "tiers")  # tiers (the paper) | exch (previous design) | signs | signsm | mid | midall | boundall | route | routebest
EXTRA = os.environ.get("DESIGN_EXTRA", "").split()
TRACE_DIR = os.path.join(HERE, "results", "details", "design_traces")
DYN_DIR = os.path.join(HERE, "results", "details", "dynamics")
COLS = ["Instance", "design", "init", "init_seed", "D", "L_r", "S", "extra", "commit",
        "init_obj", "init_size", "Objective", "Tour", "flown_km", "energy_pct", "time_s",
        "t_best (s)", "Wall (s)", "stop", "iterations", "shakes", "iter_of_best", "accepted",
        "socp_calls", "socp_ms", "reorder_trimmed", "socp_numeric_infeasible", "Route",
        "sets_s", "sets_pct"]


def stem_of(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def git_commit():
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True,
                              text=True, cwd=REPO).stdout.strip()
    except Exception:
        return ""


COMMIT = git_commit()


def run_one(name, design):
    env = dict(os.environ, ILS_INSERT_RATIO="1", ILS_SHAKE_KNAP=SHAKE_KNAP_ENV, ILS_NO_RETURN="2",
               ILS_REORDER_W=REORDER_W)
    tag = f"design_{design}{OUT}"
    stem = stem_of(name)
    args = [sys.executable, "experiments/run_ils_time_matched.py", "--instance", name,
            "--budget", str(BUDGET), "--init", INIT, "--init-seed", str(INIT_SEED),
            "--shake-return", "--shake-backtrack", "--sweep-cap-div", str(CAPDIV), "--tag", tag]
    if design == "new":
        env["ILS_SHAKE_STALL"] = "0"
        args += ["--fast-sets"] + (["--scaled-socp"] if SCALED else []) + ["--reorder-rcl", str(RCL), "--sweep", "enum",
                 "--max-idle-shakes", str(IDLE)]
    else:
        env["ILS_SHAKE_STALL"] = "1000"
        env["ILS_REORDER_W"], env["ILS_SHAKE_KNAP"] = "exch", "6"   # as design_old.csv was run
        args += ["--max-iter", "13000"]
    if ETA != 1.0:
        args += ["--eta", str(ETA)]
    if DYNAMICS:
        os.makedirs(DYN_DIR, exist_ok=True)
        args += ["--dynamics-out", os.path.join(DYN_DIR, f"{stem}_{design}{OUT}.csv")]
    args += EXTRA
    t0 = time.time()
    out = subprocess.run(args, capture_output=True, text=True, env=env).stdout
    wall = time.time() - t0
    if "--seed-offset" in EXTRA:                # the runner suffixes its own tag with the offset
        off = int(EXTRA[EXTRA.index("--seed-offset") + 1])
        if off:
            tag = f"{tag}_w{off}"
    prefix = f"experiments/tm_ils_{stem}_{tag}"
    if os.path.exists(prefix + "_trace.csv"):
        os.makedirs(TRACE_DIR, exist_ok=True)
        shutil.copy(prefix + "_trace.csv", os.path.join(TRACE_DIR, f"{stem}_{design}{OUT}.csv"))
    m = re.search(r"best obj ([\d.]+)\s+size (\d+)\s+found at ([\d.]+) s \(iter (\d+)\)", out)
    init = re.search(r"init obj ([\d.]+) size (\d+)", out)
    phys = re.search(r"best flown ([\d.]+) m\s+energy ([\d.]+) J \(([\d.]+)% of budget\)\s+time ([\d.]+) s", out)
    stop = re.search(r"stopped by (\w+)", out)
    itr = re.search(r"(\d+) iterations at", out)
    route = re.search(r"best route: \[([^\]]*)\]", out)
    c = re.search(r"counters: (\{.*\})", out)
    cnt = ast.literal_eval(c.group(1)) if c else {}
    if not m:
        print(out[-3000:])
        return None
    socp_ms = (cnt.get("socp_us", 0) / 1000.0 / cnt["socp_calls"]) if cnt.get("socp_calls") else ""
    return {"Instance": name, "design": design, "init": INIT, "init_seed": INIT_SEED if INIT == "R3" else "",
            "D": CAPDIV, "L_r": RCL if design == "new" else "", "S": IDLE if design == "new" else "",
            "extra": " ".join(EXTRA + (["--eta", str(ETA)] if ETA != 1.0 else []) + ([f"reorder={REORDER_W}"] if REORDER_W != "tiers" else [])
                               + ([f"knap={SHAKE_KNAP_ENV}"] if SHAKE_KNAP_ENV != "0" else [])
                               + ([] if SCALED else ["physical-units"])), "commit": COMMIT,
            "init_obj": float(init.group(1)) if init else "", "init_size": int(init.group(2)) if init else "",
            "Objective": float(m.group(1)), "Tour": int(m.group(2)),
            "flown_km": round(float(phys.group(1)) / 1000.0, 2) if phys else "",
            "energy_pct": float(phys.group(3)) if phys else "",
            "time_s": round(float(phys.group(4)), 1) if phys else "",
            "t_best (s)": round(float(m.group(3)), 1), "Wall (s)": round(wall, 1),
            "stop": stop.group(1) if stop else "", "iterations": int(itr.group(1)) if itr else "",
            "shakes": cnt.get("kicks", 0), "iter_of_best": int(m.group(4)),
            "accepted": cnt.get("accepted", 0), "socp_calls": cnt.get("socp_calls", 0),
            "socp_ms": round(socp_ms, 2) if socp_ms != "" else "",
            "reorder_trimmed": cnt.get("reorder_trimmed", 0),
            "socp_numeric_infeasible": cnt.get("socp_numeric_infeasible", 0),
            "Route": "-".join(x.strip() for x in route.group(1).split(",")) if route else "",
            "sets_s": round(cnt.get("sets_us", 0) / 1e6, 2),
            "sets_pct": round(100.0 * cnt.get("sets_us", 0) / 1e6 / wall, 1) if wall > 0 else ""}


def open_csv(csv_path):
    """Rows already on disk, after moving aside a file with another column set."""
    if os.path.exists(csv_path):
        with open(csv_path, newline="") as f:
            header = next(csv.reader(f), [])
        if header == COLS or (header and header == COLS[:len(header)] and "Route" in header):
            # the current columns, or those of the driver before the set-construction
            # timer (sets_s, sets_pct): resume with the file's own columns
            done = {r["Instance"] for r in csv.DictReader(open(csv_path))}
            print(f"[resume] {len(done)} runs on disk", flush=True)
            return done, header
        k = 1
        while os.path.exists(csv_path.replace(".csv", f".v{k}.csv")):
            k += 1
        old = csv_path.replace(".csv", f".v{k}.csv")
        shutil.move(csv_path, old)
        print(f"[note] {os.path.basename(csv_path)} had the columns of an earlier driver; "
              f"kept as {os.path.basename(old)}, starting a fresh file", flush=True)
    with open(csv_path, "w", newline="") as f:
        csv.DictWriter(f, fieldnames=COLS).writeheader()
    return set(), COLS


def main():
    if os.environ.get("ILS_LICENSE_GUARD"):
        raise SystemExit("ILS_LICENSE_GUARD is set: it drops long routes silently. Unset it.")
    design = sys.argv[1] if len(sys.argv) > 1 else "new"
    assert design in ("new", "old"), "usage: run_design.py new|old"
    assert INIT in ("R1", "R2", "R3", "R4"), "DESIGN_INIT must be R1, R2, R3 or R4"
    csv_path = os.path.join(HERE, "results", f"design_{design}{OUT}.csv")
    done, header = open_csv(csv_path)
    order = ([n.strip() for n in os.environ["DESIGN_INSTANCES"].split(";")]
             if os.environ.get("DESIGN_INSTANCES") else ORDER)
    for n in order:
        if n not in PATHS:
            raise SystemExit(f"unknown instance {n!r}; known: {', '.join(ORDER)}")
    jobs = [n for n in order if n not in done]
    print(f"[plan] {design}{OUT}: {len(jobs)} runs, {WORKERS} at a time; start {INIT}"
          f"{' seed ' + str(INIT_SEED) if INIT == 'R3' else ''}, L_r={RCL} S={IDLE} D={CAPDIV} "
          f"eta={ETA:g} reorder={REORDER_W} knap={SHAKE_KNAP_ENV} units={'scaled' if SCALED else 'physical'} budget {BUDGET:.0f}s extra={EXTRA} dynamics={'on' if DYNAMICS else 'off'} commit {COMMIT}",
          flush=True)
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
                csv.DictWriter(f, fieldnames=header, extrasaction="ignore").writerow(row); f.flush(); os.fsync(f.fileno())
            print(f"[saved] {name}: obj={row['Objective']:.2f} tour={row['Tour']} t_best={row['t_best (s)']} "
                  f"wall={row['Wall (s)']} shakes={row['shakes']} stop={row['stop']}", flush=True)
    print(f"\nDONE -> {csv_path}", flush=True)


if __name__ == "__main__":
    main()
