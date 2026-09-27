#!/bin/zsh
# After the campaign: does the information bound cut SOCP calls without changing
# the solution? Four instances, with and without, serial.
cd /Users/kirtisoglu/GitHub/UAV-Routing
until grep -q CAMPAIGN_COMPLETE paper_runs/results/newdesign/campaign.log 2>/dev/null; do sleep 120; done
export ILS_INSERT_RATIO=1 ILS_SHAKE_KNAP=6 ILS_NO_RETURN=2 ILS_REORDER_W=exch ILS_SHAKE_STALL=1000
for spec in "R101 (100):r101_100" "RC104 (100):rc104_100" "C101 (100):c101_100" "PR11 (48):pr11_48"; do
  inst="${spec%%:*}"; st="${spec##*:}"
  ILS_INFO_BOUND=1 python3 experiments/run_ils_time_matched.py --instance "$inst" \
    --max-iter 13000 --budget 14400 --init R4 --shake-return --shake-backtrack \
    --tag bound_$st > paper_runs/results/newdesign/bound_$st.log 2>&1
done
echo BOUND_TEST_COMPLETE
