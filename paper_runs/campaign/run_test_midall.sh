#!/bin/zsh
# experiments/TEST_RUNBOOK.md, fourth and fifth commands: mid, then midall.
#
# An experiment, not a change of design. Nothing in the paper is touched and
# design_new.csv is not rerun. One run at a time. On a [warn] or a traceback,
# note it in experiments/DESIGN.md and stop.
cd /Users/kirtisoglu/GitHub/UAV-Routing
L=paper_runs/results/newdesign
G="PR11 (48);R104 (100);RC104 (100);C104 (100);PR15 (240);PR10 (288)"

note_and_stop() {
  {
    echo ""
    echo "## midall test halt, $(date '+%Y-%m-%d %H:%M') -- $1"
    echo ""
    echo "\`experiments/TEST_RUNBOOK.md\` stopped at $1. The step printed a warning or"
    echo "failed; nothing was retried and no result was reported from it. Console tail"
    echo "of \`$2\`:"
    echo ""
    echo '```'
    tail -25 "$2"
    echo '```'
  } >> experiments/DESIGN.md
  echo "HALTED at $1 -- written up in experiments/DESIGN.md"
  exit 1
}

check() {
  grep -q '^\[warn\]' "$2" && note_and_stop "$1" "$2"
  grep -qi 'Traceback' "$2" && note_and_stop "$1" "$2"
  grep -q 'DONE ->' "$2" || note_and_stop "$1 (no DONE line)" "$2"
  echo "  ok: $1"
}

DESIGN_INSTANCES="$G" DESIGN_REORDER_W=mid DESIGN_OUT=_mid \
    python3 paper_runs/run_design.py new > $L/test_mid.log 2>&1
check "mid" $L/test_mid.log

DESIGN_INSTANCES="$G" DESIGN_REORDER_W=midall DESIGN_OUT=_midall \
    python3 paper_runs/run_design.py new > $L/test_midall.log 2>&1
check "midall" $L/test_midall.log

echo "TEST_MIDALL_COMPLETE"
