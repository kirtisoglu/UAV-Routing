#!/bin/zsh
# Steps 2 to 6 of experiments/RERUN_PLAN.md, unattended.
#
# Waits for the Step 2 pair already running, then works through the remaining
# steps in order, writing each table and figure as soon as its runs are on disk.
# Rule 5 of the runbook: on a [warn] or a traceback, record it under a dated
# heading in experiments/DESIGN.md and stop -- never retry with other settings.
cd /Users/kirtisoglu/GitHub/UAV-Routing
L=paper_runs/results/newdesign
mkdir -p $L

note_and_stop() {   # $1 = step, $2 = logfile
  {
    echo ""
    echo "## Campaign halt, $(date '+%Y-%m-%d %H:%M') -- $1"
    echo ""
    echo "\`paper_runs/campaign/run_rest.sh\` stopped at $1. Runbook rule 5: the step"
    echo "printed a warning or failed, so nothing was retried and no table was written"
    echo "from it. Console tail of \`$2\`:"
    echo ""
    echo '```'
    tail -25 "$2"
    echo '```'
  } >> experiments/DESIGN.md
  echo "HALTED at $1 -- written up in experiments/DESIGN.md"
  exit 1
}

check() {           # $1 = step, $2 = logfile
  grep -q '^\[warn\]' "$2" && note_and_stop "$1" "$2"
  grep -qi 'Traceback' "$2" && note_and_stop "$1" "$2"
  grep -q 'DONE ->' "$2" || note_and_stop "$1 (no DONE line)" "$2"
  echo "  ok: $1"
}

# --- Step 2: the pair launched before this script; wait it out ---------------
echo "[waiting] Step 2 (fixed-speed, no-loiter)"
while pgrep -f "run_design.py new" > /dev/null; do sleep 30; done
check "Step 2 --fixed-speed"  $L/step2a.log
check "Step 2 --no-loiter"    $L/step2b.log
python3 paper_runs/fill_tables.py --only tab:fixed-speed tab:coverage || exit 1
echo "[done] Step 2"

# --- Step 3: cap divisor grid ------------------------------------------------
DESIGN_CAPDIV=6  DESIGN_OUT=_D6  python3 paper_runs/run_design.py new > $L/step3a.log 2>&1
check "Step 3 D=6"  $L/step3a.log
DESIGN_CAPDIV=12 DESIGN_OUT=_D12 python3 paper_runs/run_design.py new > $L/step3b.log 2>&1
check "Step 3 D=12" $L/step3b.log
python3 paper_runs/fill_tables.py --only tab:theta || exit 1
echo "[done] Step 3"

# --- Step 4: the perturbation figure ----------------------------------------
for D in 3 6 12; do
  DESIGN_INSTANCES="PR15 (240)" DESIGN_DYNAMICS=1 DESIGN_CAPDIV=$D DESIGN_OUT=_D${D}dyn \
      python3 paper_runs/run_design.py new > $L/step4_D$D.log 2>&1
  check "Step 4 D=$D" $L/step4_D$D.log
done
python3 paper_runs/make_capdiv_figure.py || exit 1
echo "[done] Step 4"

# --- Step 5: the initial-tour table -----------------------------------------
DESIGN_INIT=R1 DESIGN_OUT=_R1 python3 paper_runs/run_design.py new > $L/step5_R1.log 2>&1
check "Step 5 R1" $L/step5_R1.log
DESIGN_INIT=R2 DESIGN_OUT=_R2 python3 paper_runs/run_design.py new > $L/step5_R2.log 2>&1
check "Step 5 R2" $L/step5_R2.log
for s in 1 2 3; do
  DESIGN_INIT=R3 DESIGN_INIT_SEED=$s DESIGN_OUT=_R3s$s \
      python3 paper_runs/run_design.py new > $L/step5_R3s$s.log 2>&1
  check "Step 5 R3 seed $s" $L/step5_R3s$s.log
done
python3 paper_runs/fill_tables.py --only tab:initial-tour || exit 1
echo "[done] Step 5"

# --- Step 6: compile ---------------------------------------------------------
cd paper
pdflatex -interaction=nonstopmode ArXiv-version > /dev/null 2>&1
bibtex ArXiv-version > /dev/null 2>&1
pdflatex -interaction=nonstopmode ArXiv-version > /dev/null 2>&1
pdflatex -interaction=nonstopmode ArXiv-version > /dev/null 2>&1
grep -c '^!' ArXiv-version.log | sed 's/^/  latex errors: /'
grep 'Output written' ArXiv-version.log | tail -1
cd ..
echo "CAMPAIGN_REST_COMPLETE"
