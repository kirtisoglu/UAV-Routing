"""Re-record the three animation datasets whose clocks ran under load.

C1_2_1 (200), PR15 (240) and RC1_2_1 (200) were recorded while other runs held
the machine, so their traces carry wall times 3.5x to 78x the ones the same runs
take alone -- 14,637 s against the 187 s of Table 10 in the worst case. The
objectives are right, the clocks are not, so the viewer's run time, its
per-iteration time and its seconds axis are all wrong on those three. Run this
with the machine idle; it also picks up the socp_us timer, which did not exist
when these were first recorded.
"""
import os, re, subprocess, sys

os.chdir("/Users/kirtisoglu/GitHub/UAV-Routing")
INST = ["C1_2_1 (200)", "RC1_2_1 (200)", "PR15 (240)"]
ENV = dict(os.environ, ILS_INSERT_RATIO="1", ILS_SHAKE_KNAP="6", ILS_NO_RETURN="2",
           ILS_REORDER_W="exch", ILS_SHAKE_STALL="1000", MOVE_TRACE_TOP="20")
stem = lambda n: re.sub(r"[^a-z0-9]+", "_", n.lower()).strip("_")

for name in INST:
    st = stem(name)
    rt, mt = f"animation/traces/{st}_route.csv", f"animation/traces/{st}_moves.csv"
    log = f"paper_runs/results/newdesign/anim_{st}.log"
    print(f"  recording {name}", flush=True)
    with open(log, "w") as fh:
        subprocess.run([sys.executable, "experiments/run_ils_time_matched.py",
                        "--instance", name, "--max-iter", "13000", "--budget", "14400",
                        "--init", "R4", "--shake-return", "--shake-backtrack",
                        "--route-trace", rt, "--move-trace", mt,
                        "--tag", f"anim_{st}"], stdout=fh, stderr=subprocess.STDOUT, env=ENV)
    t = open(log).read()
    g = lambda p: (re.search(p, t).group(1) if re.search(p, t) else "?")
    print(f"    obj {g(r'best obj ([d.0-9]+)')}  wall {g(r'wall (\d+) s')} s  "
          f"socp_us {g(chr(39) + 'socp_us' + chr(39) + r': (\d+)')}", flush=True)

subprocess.run([sys.executable, "animation/pack_all.py"])
subprocess.run([sys.executable, "animation/export_site.py"])
print("RERECORD_COMPLETE")
