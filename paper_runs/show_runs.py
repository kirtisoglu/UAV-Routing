"""Print a results table for a tagged batch against the D=3 baseline.

    python3 paper_runs/show_runs.py <tag>            all 14 instances
    python3 paper_runs/show_runs.py <tag> quad       the four-instance test set
    python3 paper_runs/show_runs.py <tag> quad5      the test set plus PR10

Reads paper_runs/results/newdesign/<tag>_<stem>.log for each instance and
compares with the D=3 column of paper_runs/results/capdiv.csv. t_best is taken
from the run's summary CSV and the baseline trace, which keep the tenths the
console line rounds away.
"""
import csv, os, re, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)

QUAD = ["R101 (100)", "RC104 (100)", "C101 (100)", "PR11 (48)"]   # fastest first, loop-fixed
QUAD5 = QUAD + ["PR10 (288)"]
SIX = QUAD + ["C104 (100)", "PR10 (288)"]
TEN = ["R101 (50)", "R101 (100)", "R1_2_1 (200)", "R102 (100)", "RC104 (100)",
       "C101 (50)", "C101 (100)", "C104 (100)", "PR11 (48)", "PR10 (288)"]


def stem(n):
    return re.sub(r"[^a-z0-9]+", "_", n.lower()).strip("_")


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    tag = sys.argv[1]
    scope = sys.argv[2] if len(sys.argv) > 2 else "all"
    base = {r["Instance"]: r for r in csv.DictReader(open("paper_runs/results/capdiv.csv"))
            if r["cap_div"] == "3"}
    # The A-B-A-B loop fix is the default, so the reference is the loop-fixed run
    # where one exists. The old D=3 column reached its best early only because the
    # cycle stopped the search, which flatters its t_best.
    BASE_TAG = os.environ.get("BASE_TAG", "nr2")
    for name in list(base):
        f = f"paper_runs/results/newdesign/{BASE_TAG}_{stem(name)}.log"
        if not os.path.exists(f):
            continue
        b = open(f).read()
        if "FINISHED" not in b:
            continue
        gg = lambda p, d="": (re.search(p, b).group(1) if re.search(p, b) else d)
        base[name] = dict(base[name], Objective=gg(r"best obj ([\d.]+)"),
                          **{"t_best (s)": gg(r"found at ([\d.]+) s"),
                             "Wall (s)": gg(r"wall (\d+) s"),
                             "shakes": gg(chr(39) + "kicks" + chr(39) + r": (\d+)"),
                             "_fixed": "1"})
    names = {"quad": QUAD, "quad5": QUAD5, "six": SIX, "ten": TEN}.get(scope, sorted(base))

    print(f"{'instance':<16}{'obj':>11}{'base obj':>11}{'D%':>7}"
          f"{'t_best':>8}{'base':>8}{'wall':>7}{'base':>7}{'shk':>6}{'base':>6}")
    nfix = sum(1 for n in names if base.get(n, {}).get("_fixed"))
    print(f"  baseline: {BASE_TAG} (loop-fixed) for {nfix}/{len(names)}, D=3 otherwise\n")
    deltas = []
    for name in names:
        b = base[name]
        log = f"paper_runs/results/newdesign/{tag}_{stem(name)}.log"
        if not os.path.exists(log):
            print(f"{name:<16}{'queued':>11}")
            continue
        txt = open(log).read()
        if "FINISHED" not in txt:
            print(f"{name:<16}{'running':>11}")
            continue

        def g(p, d=""):
            m = re.search(p, txt)
            return m.group(1) if m else d

        o = float(g(r"best obj ([\d.]+)", "0"))
        ob = float(b["Objective"])
        deltas.append(100 * (o - ob) / ob)
        # the summary CSV and the baseline trace keep t_best to a tenth
        tb = g(r"found at ([\d.]+) s")
        sc = f"experiments/tm_ils_{stem(name)}_{tag}_{stem(name)}_summary.csv"
        if os.path.exists(sc):
            try:
                tb = next(csv.DictReader(open(sc)))["best_wall_s"]
            except Exception:
                pass
        btb = b["t_best (s)"]
        bt = f"paper_runs/results/details/capdiv_traces/{stem(name)}_d3.csv"
        if not b.get("_fixed") and os.path.exists(bt):
            try:
                btb = f"{float(list(csv.DictReader(open(bt)))[-1]['wall_s']):.1f}"
            except Exception:
                pass
        print(f"{name:<16}{o:>11.2f}{ob:>11.2f}{deltas[-1]:>+7.2f}"
              f"{tb:>8}{btb:>8}{g(r'wall (\d+) s'):>7}{float(b['Wall (s)']):>7.0f}"
              f"{g(chr(39) + 'kicks' + chr(39) + r': (\d+)'):>6}{b['shakes']:>6}")
    if deltas:
        print(f"\nfinished {len(deltas)}/{len(names)}   mean {sum(deltas)/len(deltas):+.2f}%   "
              f"better {sum(1 for x in deltas if x > 0.005)}  "
              f"worse {sum(1 for x in deltas if x < -0.005)}")


if __name__ == "__main__":
    main()
