"""Pack a route trace and its move trace into one JSON for the interactive viewer.

Frames are the events that change the route (accepted moves and shakes). Each
frame carries the route, the objective, the best so far, and the proposals that
were evaluated and rejected since the previous frame, so the viewer can show
both what the search took and what it turned down.

    python3 animation/build_viewer_data.py TRACE.csv MOVES.csv "R104 (100)" OUT.json
"""
import csv, json, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "paper_runs"))
sys.path.insert(0, os.path.join(REPO, "experiments"))
os.chdir(REPO)
from common import make_instance
from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES

MAX_FRAMES = int(os.environ.get("VIEWER_FRAMES", 900))
MAX_PROPOSALS = int(os.environ.get("VIEWER_PROPOSALS", 6))
MAX_CANDS = int(os.environ.get("VIEWER_CANDS", 12))



def _clean_wall(name):
    """The run time the paper reports for this instance, or None."""
    f = "paper_runs/results/final.csv"
    if not os.path.exists(f):
        return None
    for r in csv.DictReader(open(f)):
        if r["Instance"] == name:
            try:
                return float(r["Wall (s)"])
            except (KeyError, ValueError):
                return None
    return None

def main():
    rt, mt, name, out = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
    log = sys.argv[5] if len(sys.argv) > 5 else None      # run log, for the counters
    instance, graph, drone = make_instance(dict(ALL_INSTANCES + EXPANSION_INSTANCES)[name])
    depot = drone.base
    nodes = sorted(graph.nodes)
    pos = {n: graph.nodes[n]["position"] for n in nodes}
    tw = {n: list(graph.nodes[n]["time_window"]) for n in nodes}
    slope = {n: float(graph.nodes[n].get("info_slope", 0.0)) for n in nodes}
    info0 = {n: float(graph.nodes[n].get("info_at_lowest", 0.0)) for n in nodes}

    # pass "-" when the run has no move trace of its own; candidate sets from a
    # different run would not correspond to these frames
    # set_size and w_sum describe the WHOLE set at the draw, while the rows list
    # only its top few, so the page needs them to turn weights into probabilities
    cands_by_iter, cand_meta, set_sizes = {}, {}, {}
    if mt != "-" and os.path.exists(mt):
        for r in csv.DictReader(open(mt)):
            it = int(r["iter"])
            cands_by_iter.setdefault(it, []).append(
                [r["cand"], float(r["weight"]), r["op"]])
            if it not in cand_meta:
                cand_meta[it] = (int(r.get("set_size") or 0),
                                 float(r.get("w_sum") or 0.0))
            if r.get("set_size"):
                set_sizes.setdefault((it, r["op"]), int(r["set_size"]))

    # cap = max(T/T_max, E/E_max) of the solved route, the quantity the
    # acceptance test of Swap and 2-opt compares. The trace carries the per-leg
    # speed and loitering of the subproblem's schedule, so both totals are read
    # back from it rather than re-solved.
    c0, c1, c2 = drone.c_0, drone.c_1, drone.c_2
    T_max, E_max = instance.time_horizon, instance.max_energy

    def cap_of(route, speeds, loiter):
        if not speeds:
            return None, None, None
        cyc = route + [depot]
        T = E = 0.0
        for (a, b), v, lo in zip(zip(cyc, cyc[1:]), speeds, loiter or []):
            if not v:
                continue
            L = graph[a][b]["distance"] + (lo or 0.0)
            T += L / v
            E += c0 * L / v + c1 * L * v ** 2 + c2 * L / v ** 2
        return T / T_max, E / E_max, max(T / T_max, E / E_max)

    rows = list(csv.DictReader(open(rt)))
    # A trace recorded while other runs held the machine carries wall times far
    # above what the same run takes alone -- C1_2_1 (200) ran 14,637 s against
    # the 187 s of the clean run. The search is deterministic, so the iterations
    # and the objectives are unaffected and only the clock has to be put back on
    # scale: stretch it onto the clean run time the paper reports.
    ref = _clean_wall(name)
    if ref and rows:
        seen = max(float(r["wall_s"]) for r in rows)
        if seen > 0 and abs(seen - ref) / ref > 0.05:
            k = ref / seen
            for r in rows:
                r["wall_s"] = str(float(r["wall_s"]) * k)
            print(f"  clock rescaled x{k:.4f}: {seen:,.0f} s -> {ref:,.0f} s", flush=True)
    totals = {"accepted": 0, "shake": 0, "refused": 0, "infeasible": 0}
    for r in rows:
        if r["verdict"] in totals:
            totals[r["verdict"]] += 1
    # An iteration whose operator had an empty feasible set proposes nothing and
    # writes no row, so the rows do not account for every iteration. The run
    # counts those as iterations and they count toward the stopping rule, so the
    # page reports them rather than leaving the arithmetic short.
    totals["iterations"] = max(int(r["iter"]) for r in rows) if rows else 0
    totals["no_move"] = max(0, totals["iterations"] - len(rows))
    frames, pending = [], []
    for r in rows:
        v = r["verdict"]
        if v in ("accepted", "shake"):
            def seq(col, cast=float):
                s = r.get(col) or ""
                return [cast(x) if x else None for x in s.split("-")] if s else None
            frames.append({
                "e": r["event"], "v": v, "it": int(r["iter"]),
                "t": float(r["wall_s"]),
                "f": float(r["obj"]) if r["obj"] else None,
                "b": float(r["best"]),
                "r": [int(x) for x in r["route"].split("-")],
                "p": pending[-MAX_PROPOSALS:],
                "np": len(pending),
                "c": [c[:2] for c in cands_by_iter.get(int(r["iter"]), [])[:MAX_CANDS]],
                "cn": cand_meta.get(int(r["iter"]), (0, 0.0))[0],
                "cw": cand_meta.get(int(r["iter"]), (0, 0.0))[1],
                "a": seq("arrivals"), "s": seq("speeds"), "lo": seq("loiter"),
            })
            # Prefer the solver's own totals when the trace carries them: the
            # acceptance rule compares those, and reconstructing from the leg
            # data differs by the slack in the SOCP's rotated cones (~0.6%).
            if r.get("cap"):
                _tr = float(r["T"]) / T_max
                _er = float(r["E"]) / E_max
                _cap = float(r["cap"])
            else:
                _tr, _er, _cap = cap_of(frames[-1]["r"], frames[-1]["s"], frames[-1]["lo"])
            frames[-1]["tr"], frames[-1]["er"], frames[-1]["cap"] = _tr, _er, _cap
            frames[-1]["wlo"] = seq("amin")     # realized window, lower bound
            frames[-1]["whi"] = seq("amax")     # realized window, upper bound
            pending = []
        else:
            pending.append({"e": r["event"], "v": v,
                            "f": float(r["obj"]) if r["obj"] else None,
                            "r": [int(x) for x in r["route"].split("-")]})
    # the two routes the window panel draws: the last accepted route before the
    # first shake, and the best route of the run. Marked before thinning so the
    # frames survive it, and resolved to indices after.
    pre_mark = best_mark = None
    for i, fr in enumerate(frames):
        if fr["v"] == "shake":
            break
        if fr["f"] is not None:
            pre_mark = i
    bi = max((i for i, fr in enumerate(frames) if fr["f"] is not None),
             key=lambda i: frames[i]["f"], default=None)
    best_mark = bi
    # two independent marks: on a run whose best route is also the last one
    # before the first shake, a single field would keep "pre" and lose "best"
    for i, fr in enumerate(frames):
        fr["_pre"], fr["_best"] = i == pre_mark, i == best_mark
    if len(frames) > MAX_FRAMES:      # thin uniformly, so the mix of events is preserved
        step = len(frames) / MAX_FRAMES
        keep = [int(k * step) for k in range(MAX_FRAMES)]
        for m in (pre_mark, best_mark):
            if m is not None and m not in keep:
                keep.append(m)
        keep = sorted(set(keep))
        frames = [frames[k] for k in keep]

    # metres per coordinate unit, so the page can turn positions into travel times
    ns = [n for n in nodes if n != depot][:2]
    import math
    du = math.dist(pos[ns[0]], pos[ns[1]])
    scale = graph[ns[0]][ns[1]]["distance"] / du if du else 1.0
    if not frames:
        raise SystemExit(f"{rt}: no route-changing events, nothing to show")
    # ---- statistics panel -------------------------------------------------
    import statistics as _st
    ops = ("add", "replace", "swap", "two_opt")
    by_op = {o: {"acc": 0, "ref": 0, "inf": 0} for o in ops}
    for r in rows:
        o = r["event"]
        if o not in by_op:
            continue
        v = r["verdict"]
        if v == "accepted": by_op[o]["acc"] += 1
        elif v == "infeasible": by_op[o]["inf"] += 1
        elif v == "refused": by_op[o]["ref"] += 1
    setsz = {}
    for o in ops:
        v = [n for (it, oo), n in set_sizes.items() if oo == o]
        if v:
            setsz[o] = {"min": min(v), "avg": round(_st.mean(v), 1), "max": max(v)}
    # a shake follows a local optimum, so the gap between shakes is the gap
    # between the local optima the search visits
    sh = [int(r["iter"]) for r in rows if r["verdict"] == "shake"]
    gaps = [sh[i + 1] - sh[i] for i in range(len(sh) - 1)]
    ev = [r for r in rows if r["obj"]]
    best_i = max(range(len(ev)), key=lambda i: float(ev[i]["obj"])) if ev else None
    solved = totals["accepted"] + totals["refused"] + totals["infeasible"]
    stats = {
        "sets": setsz,
        "ops": {o: dict(by_op[o],
                        rate=round(100 * by_op[o]["acc"] /
                                   max(1, sum(by_op[o].values())), 1)) for o in ops},
        "gap": {"max": max(gaps) if gaps else 0,
                "avg": round(_st.mean(gaps), 1) if gaps else 0, "n": len(sh)},
        "solved": solved,
        "infeas_pct": round(100 * totals["infeasible"] / max(1, solved), 1),
        "wall": round(max(float(r["wall_s"]) for r in rows), 1) if rows else 0,
        "ms_per_iter": round(1000 * max(float(r["wall_s"]) for r in rows) /
                             max(1, totals["iterations"]), 2) if rows else 0,
    }
    if log and os.path.exists(log):
        lt = open(log).read()
        cnt = lambda k: (int(re.search(rf"'{k}': (\d+)", lt).group(1))
                         if re.search(rf"'{k}': (\d+)", lt) else 0)
        us, calls = cnt("socp_us"), cnt("socp_calls")
        if calls:
            stats["socp"] = {"calls": calls}
            if us:            # socp_us is newer than some runs; 0 means untimed
                stats["socp"].update(
                    s=round(us / 1e6, 1),
                    pct=round(100 * (us / 1e6) / max(1e-9, stats["wall"]), 1),
                    ms_each=round(us / 1000 / calls, 2))
        stats["cache_hits"] = cnt("cache_feas") + cnt("cache_infeas")
        stats["ts_reject"] = cnt("ts_reject")
    stats["pre_i"] = next((i for i, fr in enumerate(frames) if fr.get("_pre")), None)
    stats["best_i"] = next((i for i, fr in enumerate(frames) if fr.get("_best")), None)
    for fr in frames:
        fr.pop("_pre", None); fr.pop("_best", None)
    data = {
        "instance": name, "depot": depot, "stats": stats,
        "vmax": drone.speed_max, "scale": scale, "horizon": instance.time_horizon,
        "nodes": [{"id": n, "x": pos[n][0], "y": pos[n][1],
                   "e": tw[n][0], "l": tw[n][1],
                   "g": slope[n], "i0": info0[n]} for n in nodes],
        "emax": E_max,
        "budget": max(f["t"] for f in frames),
        "totals": totals, "events": sum(totals.values()),
        "frames": frames,
    }
    json.dump(data, open(out, "w"), separators=(",", ":"))
    print(f"wrote {out}  frames {len(frames)}  size {os.path.getsize(out)/1e6:.2f} MB")


if __name__ == "__main__":
    main()
