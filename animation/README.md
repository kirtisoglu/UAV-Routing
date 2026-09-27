# Route replay

An interactive replay of a local-search run: the route on the map, the candidate
set that was on offer at each step, the proposals that were turned down, and a
timeline of the objective that doubles as a scrubber.

The output is one self-contained HTML file. Open it by double-clicking; it needs
no server and no network beyond the web font.

## 1. Record

    python3 experiments/run_ils_time_matched.py \
        --instance "R104 (100)" --budget 300 --init R4 \
        --theta 999999999 --sweep-cap-div 6 \
        --local-search paper --paper-label --exhaust-shake \
        --route-trace animation/traces/r104_300s.csv \
        --move-trace  animation/traces/r104_300s_moves.csv \
        --tag anim

`--route-trace` writes one row per evaluated candidate with its verdict
(accepted, refused, infeasible) and one per shake, each with the route and, for
rows whose route was solved, the arrival time at every visit and the speed and
loitering on every leg as the subproblem set them. `--move-trace` writes the
candidate set that was on offer at each of those events, ranked by weight;
`MOVE_TRACE_TOP` sets how many candidates are kept (default 40).

Pass `-` in place of the move trace when a run has none. Candidate sets from a
different run do not correspond to these frames and must never be substituted.

## 2. Pack

    python3 animation/build_viewer_data.py \
        animation/traces/r104_300s.csv animation/traces/r104_300s_moves.csv \
        "R104 (100)" animation/out/r104_300s.json

Frames are the events that change the route. Each carries the route, the
objective, the best so far, the candidate set and the proposals rejected since
the previous frame. `VIEWER_FRAMES` caps the frame count (default 900, every
shake kept), `VIEWER_PROPOSALS` how many rejected routes are drawn per frame,
`VIEWER_CANDS` how many candidates are listed.

## 3. Build the page

    python3 animation/build_html.py animation/out/r104_300s.json animation/out/r104_300s.html

`viewer_template.html` is the page; the builder inlines the trace into it.

## Reading it

Filled points are scheduled targets, hollow ones unscheduled, the square is the
depot. The tour takes the colour of the operator that produced the frame: green
Insert, orange Replace, purple Swap, blue 2-opt, red shake. Grey ghost routes are
proposals the search evaluated and rejected. Weights in the rail are the
candidate's information gain per metre of added distance.

On the timeline the dark line is the best objective, the grey line the current
one, and the red ticks are shakes. Click or drag it to jump. Space plays and
pauses, the arrow keys step.

Hovering a target gives its window, its reward slope, and for a scheduled target
its arrival and the speed on the legs either side. Those come from the solved
schedule when the trace carries one. A trace recorded before that was added
falls back to the earliest-arrival schedule, and the panel then says
“estimated”.

## Also here

`make_route_animation.py` renders the same trace as a GIF, for slides.

## Every dataset at once

    python3 animation/record_all.py                # all fourteen instances
    python3 animation/record_all.py "PR15 (240)"   # one or more by name

Records a ten-minute run per instance and writes `animation/datasets/<stem>/`
with `route.csv`, `moves.csv`, `data.json` and a standalone `index.html`. An
instance whose page already exists is skipped, so the command resumes. A
directory listing of every instance recorded is written to
`animation/datasets/index.json`, which is the file to read when embedding the
set on a site.

`REC_BUDGET` sets the seconds per run (default 600) and `REC_TOP` how many
candidates are kept per event (default 20).
