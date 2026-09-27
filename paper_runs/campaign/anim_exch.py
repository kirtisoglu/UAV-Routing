"""Exchange config at maxIter 10000, with animation traces, ten instances."""
import subprocess, re, csv, os, sys, time
os.chdir("/Users/kirtisoglu/GitHub/UAV-Routing")
INST = ["R101 (50)", "R101 (100)", "R1_2_1 (200)", "R102 (100)", "RC104 (100)",
        "C101 (50)", "C101 (100)", "C104 (100)", "PR11 (48)", "PR10 (288)",
        "R104 (100)", "PR15 (240)", "C1_2_1 (200)", "RC1_2_1 (200)"]   # all 14
ENV = dict(os.environ, ILS_INSERT_RATIO="1", ILS_SHAKE_KNAP="6", ILS_NO_RETURN="2",
           ILS_REORDER_W="exch", ILS_SHAKE_STALL="1000", MOVE_TRACE_TOP="20")
def stem(n): return re.sub(r"[^a-z0-9]+", "_", n.lower()).strip("_")
rows = []
for name in INST:
    st = stem(name)
    rt = f"animation/traces/{st}_route.csv"; mt = f"animation/traces/{st}_moves.csv"
    log = f"paper_runs/results/newdesign/anim_{st}.log"
    with open(log, "w") as fh:
        subprocess.run([sys.executable, "experiments/run_ils_time_matched.py",
                        "--instance", name, "--max-iter", os.environ.get("MAXITER", "13000"), "--budget", "14400",
                        "--init", "R4", "--shake-return", "--shake-backtrack",
                        "--route-trace", rt, "--move-trace", mt,
                        "--tag", f"anim_{st}"], stdout=fh, stderr=subprocess.STDOUT, env=ENV)
    t = open(log).read()
    g = lambda p, d="": (re.search(p, t).group(1) if re.search(p, t) else d)
    rows.append({"Instance": name, "Objective": g(r"best obj ([\d.]+)"),
                 "t_best (s)": g(r"found at ([\d.]+) s"), "shakes": g(r"'kicks': (\d+)"),
                 "Wall (s)": g(r"wall (\d+) s")})
    print(f"  ran {name}", flush=True)
    d = f"animation/datasets/{st}"
    os.makedirs(d, exist_ok=True)
    subprocess.run([sys.executable, "animation/build_viewer_data.py", rt, mt, name,
                    os.path.join(d, "data.json"), log], capture_output=True, text=True)
    print(f"  packed {name}", flush=True)
subprocess.run([sys.executable, "animation/export_site.py"], capture_output=True, text=True)
with open("paper_runs/results/anim_exch.csv", "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
print("ANIM_EXCH_COMPLETE")
