#!/bin/zsh
# Step 8 of experiments/RERUN_PLAN.md: the two levers by energy budget.
#
# Six campaigns of fourteen runs, one run at a time, then the table and the
# compile. Rule 5: on a [warn] or a traceback, write it up under a dated heading
# in experiments/DESIGN.md and stop -- never retry with other settings.
cd /Users/kirtisoglu/GitHub/UAV-Routing
L=paper_runs/results/newdesign

note_and_stop() {
  {
    echo ""
    echo "## Campaign halt, $(date '+%Y-%m-%d %H:%M') -- $1"
    echo ""
    echo "Step 8 stopped at $1. Runbook rule 5: the step printed a warning or"
    echo "failed, so nothing was retried and no table was written from it."
    echo "Console tail of \`$2\`:"
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

for e in 0.75 1.25; do
  t=$(echo $e | tr -d .)
  DESIGN_ETA=$e DESIGN_OUT=_eta$t \
      python3 paper_runs/run_design.py new > $L/step8_eta$t.log 2>&1
  check "Step 8 eta=$e reference" $L/step8_eta$t.log

  DESIGN_ETA=$e DESIGN_EXTRA="--fixed-speed" DESIGN_OUT=_fixed_eta$t \
      python3 paper_runs/run_design.py new > $L/step8_fixed_eta$t.log 2>&1
  check "Step 8 eta=$e fixed-speed" $L/step8_fixed_eta$t.log

  DESIGN_ETA=$e DESIGN_EXTRA="--no-loiter" DESIGN_OUT=_noloiter_eta$t \
      python3 paper_runs/run_design.py new > $L/step8_noloiter_eta$t.log 2>&1
  check "Step 8 eta=$e no-loiter" $L/step8_noloiter_eta$t.log
done

python3 paper_runs/fill_tables.py --only tab:levers-eta || exit 1
echo "[written] tab:levers-eta"

cd paper
pdflatex -interaction=nonstopmode ArXiv-version > /dev/null 2>&1
bibtex ArXiv-version > /dev/null 2>&1
pdflatex -interaction=nonstopmode ArXiv-version > /dev/null 2>&1
pdflatex -interaction=nonstopmode ArXiv-version > /dev/null 2>&1
grep -c '^!' ArXiv-version.log | sed 's/^/  latex errors: /'
grep 'Output written' ArXiv-version.log | tail -1
cd ..
echo "STEP8_COMPLETE"
