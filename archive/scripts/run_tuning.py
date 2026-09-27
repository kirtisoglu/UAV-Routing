"""Tuning grids for the iterated local search (Section 5 parameter choices).

Six grids on five instances spanning the three window regimes, each run a
single start from R4 with a 600 s budget, six runs at a time (the SOCP
subproblem is single-threaded). Results: paper_runs/results/tuning/<grid>.csv.

  python3 paper_runs/run_tuning.py <grid>      grid in GRIDS below

All grids were rerun on 2026-09-11 after the ILS drivers were switched to
the calibrated graph (before that, the feasibility checks 2, 4 and 5 read
the raw benchmark distances). Grids and what they test:
  shake_design           cap 4 / cap k/6 / segment kick / no shake / no reordering
                         at the original threshold (theta = 10 rejected moves)
  threshold_old_counter  theta in {30,100,300} with the old counter
  operators              full pool vs without Swap / 2-opt / Replace, plus Relocate
  threshold_new_counter  theta in {100,300,1000} after the counter fix
  threshold_scaled       theta = 10n and 20n on the two largest instances
  default_check          the adopted default (theta = 20n, cap k/6) on the rest
"""
import os, sys, csv, re, ast, time, subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed

HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(HERE); os.chdir(REPO)
INST = {"R101 (100)": 101, "C101 (100)": 101, "R104 (100)": 101, "PR15 (240)": 241, "R1_2_1 (200)": 201}
SWEEP6 = ["--ruin-mode", "sweep", "--sweep-cap-div", "6"]
GRIDS = {
    "shake_design": {
        "cap4":      ["--ruin-mode", "sweep", "--sweep-cap-max", "4", "--theta", "10"],
        "capn6":     SWEEP6 + ["--theta", "10"],
        "segment":   ["--ruin-mode", "segment", "--theta", "10"],
        "noshake":   SWEEP6 + ["--theta", "1000000000"],
        "noreorder": ["--ruin-mode", "sweep", "--sweep-cap-max", "4", "--theta", "10", "--no-swap-2opt"],
    },
    "threshold_old_counter": {f"n6_th{t}": SWEEP6 + ["--theta", str(t)] for t in (30, 100, 300)},
    "operators": {
        "full":       SWEEP6 + ["--theta", "100"],
        "no_swap":    SWEEP6 + ["--theta", "100", "--disable", "swap"],
        "no_2opt":    SWEEP6 + ["--theta", "100", "--disable", "two_opt"],
        "no_replace": SWEEP6 + ["--theta", "100", "--disable", "replace"],
        "plus_reloc": SWEEP6 + ["--theta", "100", "--relocate"],
    },
    "threshold_new_counter": {f"new_th{t}": SWEEP6 + ["--theta", str(t)] for t in (100, 300, 1000)},
    "threshold_scaled": {"th10n": "10n", "th20n": "20n"},      # theta computed per instance
    "default_check": {"default": SWEEP6},
    "strict_accept": {"default": []},   # acceptance f(R') > f(R), chosen settings
    "checks": {                       # chosen ILS: count what each check rejects; weighting on vs off
        "full":    [],
        "uniform": ["--uniform-sampling"],
    },
}
# "threshold_old_counter" keeps its name from the first campaign; since the
# 2026-09-11 rerun it uses the current counter (every non-accepted proposal
# counts), so it differs from "threshold_new_counter" only in the theta values.


def run(name, conf, args):
    if isinstance(args, str):                       # "10n" / "20n"
        args = SWEEP6 + ["--theta", str(int(args[:-1]) * (INST[name] - 1))]
    tag = f"tune_{conf}"; t0 = time.time()
    out = subprocess.run([sys.executable, "experiments/run_ils_time_matched.py", "--instance", name,
                          "--budget", "600", "--tag", tag] + args,
                         capture_output=True, text=True).stdout
    stem = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
    for suf in ("_trace.csv", "_summary.csv", "_trace.png"):
        p = f"experiments/tm_ils_{stem}_{tag}{suf}"
        if os.path.exists(p): os.remove(p)
    m = re.search(r"best obj ([\d.]+)\s+size (\d+)\s+found at (\d+) s \(iter (\d+)\)", out)
    c = re.search(r"counters: (\{.*\})", out); cnt = ast.literal_eval(c.group(1)) if c else {}
    row = {"instance": name, "config": conf, "best": float(m.group(1)) if m else None,
           "size": int(m.group(2)) if m else None, "t_best": int(m.group(3)) if m else None,
           "iters": int(m.group(4)) if m else None, "wall": round(time.time() - t0)}
    for op in ("add", "replace", "swap", "two_opt", "relocate"):
        row[f"prop_{op}"] = cnt.get(f"prop_{op}", 0); row[f"acc_{op}"] = cnt.get(f"acc_{op}", 0)
    for k in ("kicks", "socp_calls", "socp_infeas", "saturated", "cache_infeas", "cache_feas", "lb_reject", "worse", "accepted",
              "chk1_fb_excluded", "chk2_cascade_rejected", "chk2_cascade_passed"):
        row[k] = cnt.get(k, 0)
    return row


def main():
    grid = sys.argv[1]; conf = GRIDS[grid]
    out = os.path.join(HERE, "results", "tuning", grid + ".csv")
    done = {(r["instance"], r["config"]) for r in csv.DictReader(open(out))} if os.path.exists(out) else set()
    insts = ["R1_2_1 (200)", "PR15 (240)"] if grid == "threshold_scaled" else \
            ["R101 (100)", "C101 (100)", "R104 (100)"] if grid == "default_check" else list(INST)
    jobs = [(n, c) for n in insts for c in conf if (n, c) not in done]
    with ThreadPoolExecutor(max_workers=6) as ex:
        futs = {ex.submit(run, n, c, conf[c]): (n, c) for n, c in jobs}
        for f in as_completed(futs):
            r = f.result(); new = not os.path.exists(out)
            with open(out, "a", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(r.keys()))
                if new: w.writeheader()
                w.writerow(r); fh.flush()
            print(f"[saved] {r['instance']:13s} {r['config']:12s} best={r['best']} kicks={r['kicks']}", flush=True)


if __name__ == "__main__":
    main()
