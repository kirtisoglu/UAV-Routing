#!/bin/zsh
# Full overnight campaign at maxIter = 13000 with the 2026-09-26 base.
cd /Users/kirtisoglu/GitHub/UAV-Routing
C=paper_runs/campaign
export ILS_INSERT_RATIO=1 ILS_SHAKE_KNAP=6 ILS_NO_RETURN=2 ILS_REORDER_W=exch ILS_SHAKE_STALL=1000
# capdiv.csv is seeded with the D=3 column, lifted from the stage-1 runs, whose
# protocol is identical (default --sweep-cap-div 3). run_capdiv.py resumes from it
# and runs only D=6 and D=12. The old-design baseline is in capdiv_old_design.csv.
T9_MAX_ITER=13000 T9_WORKERS=1 python3 paper_runs/run_capdiv.py > paper_runs/results/newdesign/t9.log 2>&1
# Figure 2: D=3 comes from the animation run of PR15 already on disk, and
# D=6 and D=12 are written by the Table 9 runs above. No separate stage.
T8_MAX_ITER=13000 T8_WORKERS=4 python3 paper_runs/run_initial_tours.py > paper_runs/results/newdesign/t8.log 2>&1
echo CAMPAIGN_COMPLETE
