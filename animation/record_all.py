"""Record a ten-minute run of every benchmark instance and build its replay page.

One run per instance at the design of Section 4, with the route trace and the
move trace, packed and built into a standalone HTML. Everything lands in
animation/datasets, one folder per instance, so a page can be embedded on its
own or the whole folder served as a set.

    python3 animation/record_all.py                # all fourteen
    python3 animation/record_all.py "PR15 (240)"   # one or more by name

Environment:
    REC_BUDGET   seconds per run (default 600)
    REC_TOP      candidates kept per event (default 20)
"""
import os, subprocess, sys, json, re, time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "experiments"))
os.chdir(REPO)
from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES

ORDER = ["R101 (50)", "R101 (100)", "R1_2_1 (200)", "C101 (50)", "C101 (100)",
         "C1_2_1 (200)", "RC1_2_1 (200)", "R102 (100)", "R104 (100)",
         "C104 (100)", "RC104 (100)", "PR11 (48)", "PR15 (240)", "PR10 (288)"]
OUT = os.path.join(HERE, "datasets")
BUDGET = os.environ.get("REC_BUDGET", "600")
TOP = os.environ.get("REC_TOP", "20")


def stem(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def record(name):
    s = stem(name)
    d = os.path.join(OUT, s)
    os.makedirs(d, exist_ok=True)
    rt, mt = os.path.join(d, "route.csv"), os.path.join(d, "moves.csv")
    js, html = os.path.join(d, "data.json"), os.path.join(d, "index.html")
    if os.path.exists(html):
        print(f"[skip] {name}: already recorded", flush=True)
        return True
    env = dict(os.environ, MOVE_TRACE_TOP=TOP)
    t0 = time.time()
    p = subprocess.run(
        [sys.executable, "experiments/run_ils_time_matched.py",
         "--instance", name, "--budget", BUDGET, "--init", "R4",
         "--route-trace", rt, "--move-trace", mt, "--tag", "anim_" + s],
        capture_output=True, text=True, env=env)
    for suf in ("_trace.csv", "_summary.csv", "_trace.png"):
        f = f"experiments/tm_ils_{s}_anim_{s}{suf}"
        if os.path.exists(f):
            os.remove(f)
    m = re.search(r"best obj ([\d.]+)\s+size (\d+)\s+found at (\d+) s", p.stdout)
    if not m or not os.path.exists(rt):
        print(f"[fail] {name}: no trace written", flush=True)
        print(p.stdout[-400:] or p.stderr[-400:], flush=True)
        return False
    for step in ([sys.executable, "animation/build_viewer_data.py", rt, mt, name, js],
                 [sys.executable, "animation/build_html.py", js, html]):
        q = subprocess.run(step, capture_output=True, text=True)
        if q.returncode:
            print(f"[fail] {name}: {' '.join(step[1:2])}\n{q.stderr[-300:]}", flush=True)
            return False
    size = os.path.getsize(html) / 1e6
    print(f"[done] {name}: obj {float(m.group(1)):,.2f} at {m.group(3)} s, "
          f"{size:.2f} MB page, {time.time() - t0:.0f} s", flush=True)
    return True


def main():
    names = sys.argv[1:] or ORDER
    known = dict(ALL_INSTANCES + EXPANSION_INSTANCES)
    for n in names:
        if n not in known:
            raise SystemExit(f"unknown instance: {n}")
    os.makedirs(OUT, exist_ok=True)
    ok = sum(record(n) for n in names)
    index = {"instances": [{"name": n, "dir": stem(n)} for n in names]}
    json.dump(index, open(os.path.join(OUT, "index.json"), "w"), indent=1)
    print(f"\n{ok} of {len(names)} recorded -> {OUT}", flush=True)


if __name__ == "__main__":
    main()
