"""Record the animation datasets under the design of `run_design.py new`.

Same configuration as the main campaign -- the flags and environment are read
from `run_design.py` rather than restated here, so the viewer can never drift
from the tables -- plus the two trace files the packer needs. One run at a
time, as the protocol requires, and the log keeps the `anim_<stem>.log` name
that `animation/pack_all.py` looks for.

    python3 paper_runs/campaign/anim_exch.py            # all fourteen
    ANIM_INSTANCES="PR11 (48);C101 (100)" python3 paper_runs/campaign/anim_exch.py
"""
import os, re, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "paper_runs"))
os.chdir(REPO)

INST = ["R101 (50)", "R101 (100)", "R1_2_1 (200)", "C101 (50)", "C101 (100)",
        "C1_2_1 (200)", "RC1_2_1 (200)", "R102 (100)", "R104 (100)", "C104 (100)",
        "RC104 (100)", "PR11 (48)", "PR15 (240)", "PR10 (288)"]
if os.environ.get("ANIM_INSTANCES"):
    INST = [n.strip() for n in os.environ["ANIM_INSTANCES"].split(";") if n.strip()]

# the campaign's own settings, so the recording matches design_new.csv
RCL = os.environ.get("DESIGN_RCL", "20")
IDLE = os.environ.get("DESIGN_IDLE", "100")
CAPDIV = os.environ.get("DESIGN_CAPDIV", "3")
BUDGET = os.environ.get("DESIGN_BUDGET", "14400")

stem_of = lambda n: re.sub(r"[^a-z0-9]+", "_", n.lower()).strip("_")

for name in INST:
    st = stem_of(name)
    rt = f"animation/traces/{st}_route.csv"
    mt = f"animation/traces/{st}_moves.csv"
    log = f"paper_runs/results/newdesign/anim_{st}.log"
    env = dict(os.environ, ILS_INSERT_RATIO="1", ILS_SHAKE_KNAP="6", ILS_NO_RETURN="2",
               ILS_REORDER_W="exch", ILS_SHAKE_STALL="0", MOVE_TRACE_TOP="20")
    args = [sys.executable, "experiments/run_ils_time_matched.py", "--instance", name,
            "--budget", BUDGET, "--init", "R4", "--shake-return", "--shake-backtrack",
            "--sweep-cap-div", CAPDIV, "--tag", f"anim_{st}",
            "--fast-sets", "--scaled-socp", "--reorder-rcl", RCL, "--sweep", "enum",
            "--max-idle-shakes", IDLE,
            "--route-trace", rt, "--move-trace", mt]
    t0 = time.time()
    with open(log, "w") as fh:
        subprocess.run(args, stdout=fh, stderr=subprocess.STDOUT, env=env)
    t = open(log).read()
    g = lambda p: (re.search(p, t).group(1) if re.search(p, t) else "?")
    print(f"  {name:<15} obj {g(r'best obj ([0-9.]+)'):>11}  wall {time.time()-t0:6.0f} s  "
          f"shakes {g(chr(39) + 'kicks' + chr(39) + r': (\d+)'):>5}  "
          f"socp_us {g(chr(39) + 'socp_us' + chr(39) + r': (\d+)'):>11}", flush=True)

print("ANIM_COMPLETE")
