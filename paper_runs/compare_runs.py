"""Compare two campaign CSVs instance by instance.

    python3 paper_runs/compare_runs.py results/design_new.csv results/design_new.v1.csv
    python3 paper_runs/compare_runs.py results/design_new.csv results/design_new_fixed.csv

Prints Objective, Tour, t_best and Wall of both files and the objective change in
percent, then the totals. The last column reads `same` when the two rows found the
same best route after the same numbers of iterations and shakes, which is what a
fixed seed and an unchanged search must give. Use it after a rerun to confirm that
a deterministic configuration reproduced its earlier numbers, or to read a variant
against the reference run before the table is written.
"""
import csv, os, sys, signal

signal.signal(signal.SIGPIPE, signal.SIG_DFL)      # quiet when piped into head

ORDER = ["R101 (50)", "R101 (100)", "R1_2_1 (200)", "C101 (50)", "C101 (100)",
         "C1_2_1 (200)", "RC1_2_1 (200)", "R102 (100)", "R104 (100)",
         "C104 (100)", "RC104 (100)", "PR11 (48)", "PR15 (240)", "PR10 (288)"]
SAME = ("Route", "iterations", "shakes")     # the run itself, not its timing


def load(p):
    """A path relative to the working directory, or to paper_runs/ (so 'results/x.csv' works from anywhere)."""
    if not os.path.exists(p):
        alt = os.path.join(os.path.dirname(os.path.abspath(__file__)), p)
        if os.path.exists(alt):
            p = alt
    return {r["Instance"]: r for r in csv.DictReader(open(p))}


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    a, b = load(sys.argv[1]), load(sys.argv[2])
    la, lb = os.path.basename(sys.argv[1]), os.path.basename(sys.argv[2])
    print(f"{'Instance':15s} {'obj ' + la:>22s} {'obj ' + lb:>22s} {'change %':>9s} {'tour':>9s} {'t_best (s)':>15s} {'wall (s)':>15s}  run")
    ta = tb = wa = wb = 0.0
    for n in ORDER:
        ra, rb = a.get(n), b.get(n)
        if ra is None or rb is None:
            print(f"{n:15s} {'-' if ra is None else 'x':>22s} {'-' if rb is None else 'x':>22s}"); continue
        oa, ob = float(ra["Objective"]), float(rb["Objective"])
        ta += float(ra["t_best (s)"]); tb += float(rb["t_best (s)"])
        wa += float(ra["Wall (s)"]); wb += float(rb["Wall (s)"])
        print(f"{n:15s} {oa:22,.2f} {ob:22,.2f} {100 * (oa - ob) / ob:+9.2f} "
              f"{ra['Tour'] + '/' + rb['Tour']:>9s} {ra['t_best (s)'] + ' / ' + rb['t_best (s)']:>15s} "
              f"{ra['Wall (s)'] + ' / ' + rb['Wall (s)']:>15s}  "
              f"{'same' if all(ra.get(c) == rb.get(c) for c in SAME) else 'differs'}")
    print(f"{'total':15s} {'':22s} {'':22s} {'':9s} {'':9s} {f'{ta:.0f} / {tb:.0f}':>15s} {f'{wa:.0f} / {wb:.0f}':>15s}")


if __name__ == "__main__":
    main()
