"""Pack every finished trace into a viewer dataset.

Skips instances whose run has not written a stop line yet, so it is safe to run
while the collection is still going and again when it finishes.
"""
import os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
os.chdir(REPO)
NAME = {"r101_50": "R101 (50)", "r101_100": "R101 (100)", "r1_2_1_200": "R1_2_1 (200)",
        "c101_50": "C101 (50)", "c101_100": "C101 (100)", "c1_2_1_200": "C1_2_1 (200)",
        "rc1_2_1_200": "RC1_2_1 (200)", "r102_100": "R102 (100)", "r104_100": "R104 (100)",
        "c104_100": "C104 (100)", "rc104_100": "RC104 (100)", "pr11_48": "PR11 (48)",
        "pr15_240": "PR15 (240)", "pr10_288": "PR10 (288)"}
STOP = re.compile(r"stopped by")

for stem, name in NAME.items():
    # the traces come from anim_exch.py, so its log is the one that matches them
    log = f"paper_runs/results/newdesign/anim_{stem}.log"
    rt, mt = f"animation/traces/{stem}_route.csv", f"animation/traces/{stem}_moves.csv"
    done = os.path.exists(log) and STOP.search(open(log).read())
    if not (done and os.path.exists(rt) and os.path.exists(mt)):
        print(f"  skip {name} (still running)" if os.path.exists(rt) else f"  skip {name} (no trace)")
        continue
    out = f"animation/datasets/{stem}"
    os.makedirs(out, exist_ok=True)
    if subprocess.run([sys.executable, "animation/build_viewer_data.py", rt, mt, name,
                       f"{out}/data.json", log]).returncode:
        print(f"  FAILED {name}"); continue
    subprocess.run([sys.executable, "animation/build_html.py", f"{out}/data.json",
                    f"{out}/index.html"], stdout=subprocess.DEVNULL)
    print(f"  packed {name}")
