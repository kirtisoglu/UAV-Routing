#!/bin/zsh
# experiments/TEST_RUNBOOK.md, "tiers on all fourteen instances", step 2.
#
# An experiment, not a change of design. Nothing in the paper is touched and
# design_new.csv is not rerun. One run at a time. On a [warn] or a traceback,
# note it in experiments/DESIGN.md and stop.
cd /Users/kirtisoglu/GitHub/UAV-Routing
L=paper_runs/results/newdesign

note_and_stop() {
  {
    echo ""
    echo "## tiers test halt, $(date '+%Y-%m-%d %H:%M') -- $1"
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

DESIGN_REORDER_W=tiers DESIGN_OUT=_tiers \
    python3 paper_runs/run_design.py new > $L/test_tiers.log 2>&1
check "tiers (weighted shake)" $L/test_tiers.log

DESIGN_REORDER_W=tiers DESIGN_SHAKE_KNAP=0 DESIGN_OUT=_tiers_knap0 \
    python3 paper_runs/run_design.py new > $L/test_tiers_knap0.log 2>&1
check "tiers knap=0 (unweighted shake)" $L/test_tiers_knap0.log

echo "TEST_TIERS_COMPLETE"
