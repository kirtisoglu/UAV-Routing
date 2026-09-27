"""Check fast_sets against the set construction of run_start_paper on routes taken
from the route traces, and time both.  No solver is needed."""
import sys, os, csv, time, random
sys.path.insert(0, 'experiments')
from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES
from run_ils_time_matched import TimedILS, chained_sets as chained_sets_old, route_distance
import fast_sets as fs
PATHS = dict(ALL_INSTANCES + EXPANSION_INSTANCES)

def load_routes(stem, n_sample=40, seed=0):
    rows = list(csv.DictReader(open(f'animation/traces/{stem}_route.csv')))
    routes = []
    seen = set()
    for r in rows:
        rt = tuple(int(x) for x in r['route'].split('-'))
        if rt not in seen:
            seen.add(rt); routes.append(list(rt))
    random.Random(seed).shuffle(routes)
    return routes[:n_sample]

def baseline_sets(run, route, Nprime):
    """The four sets exactly as run_start_paper builds them (INSERT_RATIO=1, exch)."""
    G, depot = run.graph, run.depot
    F, B = run.F_leg, run.B_leg
    DM = run.DM; d = lambda a, b: DM[a][b]
    EW, LW, I0, GAM = {}, {}, {}, {}
    for n in G.nodes:
        e, l = G.nodes[n]['time_window']; EW[n], LW[n] = float(e), float(l)
        I0[n] = float(G.nodes[n].get('info_at_lowest', 1.0)); GAM[n] = float(G.nodes[n].get('info_slope', 0.0))
    EW[depot], LW[depot], I0[depot], GAM[depot] = 0.0, run.T_max, 0.0, 0.0
    inv_v = 1.0 / run.v_max; T_max = run.T_max
    def realized(cr, chk=True):
        n = len(cr); amin = [0.0] * n; prev = 0.0
        for q in range(1, n):
            nq = cr[q]; row = DM[cr[q - 1]]
            prev = max(EW[nq], prev + row[nq] * inv_v)
            if chk and prev > LW[nq]: return False, 0.0
            amin[q] = prev
        back = DM[cr[-1]][depot] * inv_v
        if chk and prev + back > T_max: return False, 0.0
        nxt = T_max - back; tot = 0.0
        for q in range(n - 1, 0, -1):
            nq = cr[q]; hi = LW[nq] if LW[nq] < nxt else nxt; lo = amin[q]
            if chk and hi < lo: return False, 0.0
            tot += I0[nq] + GAM[nq] * (0.5 * (lo + hi) - EW[nq]); nxt = hi - DM[cr[q - 1]][nq] * inv_v
        return True, tot
    def chain_bounds(cr):
        n = len(cr); amin = [0.0] * (n + 1); amax = [0.0] * (n + 1); prev = 0.0
        for q in range(1, n):
            nq = cr[q]; prev = max(EW[nq], prev + DM[cr[q - 1]][nq] * inv_v); amin[q] = prev
        amin[n] = prev + DM[cr[-1]][depot] * inv_v; amax[n] = T_max
        nxt = T_max - DM[cr[-1]][depot] * inv_v
        for q in range(n - 1, 0, -1):
            nq = cr[q]; hi = LW[nq] if LW[nq] < nxt else nxt; amax[q] = hi; nxt = hi - DM[cr[q - 1]][nq] * inv_v
        amax[0] = nxt
        return amin, amax
    k = len(route) - 1; ext = route + [depot]
    _ok0, info0 = realized(route)
    amin0, amax0 = chain_bounds(route)
    d_room = run.E_max / run.e_per_m - route_distance(route, G, depot)
    out = {}
    # add
    FR, BR = chained_sets_old(route, F, B, depot); moves = []
    for p in range(1, k + 2):
        i, j = ext[p - 1], ext[p]; cand = FR[p] & BR[p] & Nprime; dij = d(i, j)
        for u in cand:
            dd = d(i, u) + d(u, j) - dij
            if dd > d_room: continue
            au = EW[u]; cand_a = amin0[p - 1] + d(i, u) * inv_v
            if cand_a > au: au = cand_a
            if au > LW[u] or au + d(u, j) * inv_v > amax0[p]: continue
            ok, info_new = realized(route[:p] + [u] + route[p:], False)
            if not ok: continue
            dI = info_new - info0
            moves.append((dI / dd if dd > 1e-9 else dI, ("add", u, p), dd))
    out["add"] = moves
    # replace
    moves = []
    for p in range(1, k + 1):
        i, v, j = ext[p - 1], ext[p], ext[p + 1]; cand = FR[p] & BR[p + 1] & Nprime
        base = d(i, v) + d(v, j); gap = d(i, j)
        for u in cand:
            dd = d(i, u) + d(u, j) - base; dd_w = d(i, u) + d(u, j) - gap
            if dd > d_room: continue
            au = EW[u]; cand_a = amin0[p - 1] + d(i, u) * inv_v
            if cand_a > au: au = cand_a
            if au > LW[u] or au + d(u, j) * inv_v > amax0[p + 1]: continue
            ok, info_new = realized(route[:p] + [u] + route[p + 1:], False)
            if not ok: continue
            dI = info_new - info0
            moves.append((dI / dd_w if dd_w > 1e-9 else dI, ("replace", u, p), dd))
    out["replace"] = moves
    # swap
    buf = list(route); moves = []
    for p in range(1, k):
        rp = ext[p]
        for q in range(p + 1, k + 1):
            rq = ext[q]
            if rp not in F[rq]: continue
            ok = True
            for m in range(p + 1, q):
                rm = ext[m]
                if rm not in F[rq] or rp not in F[rm]: ok = False; break
            if not ok: continue
            if q == p + 1:
                dd = d(ext[p - 1], rq) + d(rp, ext[q + 1]) - d(ext[p - 1], rp) - d(rq, ext[q + 1])
            else:
                dd = (d(ext[p - 1], rq) + d(rq, ext[p + 1]) + d(ext[q - 1], rp) + d(rp, ext[q + 1])
                      - d(ext[p - 1], rp) - d(rp, ext[p + 1]) - d(ext[q - 1], rq) - d(rq, ext[q + 1]))
            if dd > d_room: continue
            buf[p], buf[q] = buf[q], buf[p]; ok, info_new = realized(buf); buf[p], buf[q] = buf[q], buf[p]
            if not ok: continue
            moves.append(((GAM[rp] - GAM[rq]) * (amin0[q] - amin0[p]), ("swap", p, q), dd))
    out["swap"] = moves
    # two_opt
    buf = list(route); moves = []
    for p in range(1, k):
        for q in range(p + 1, k + 1):
            rq = ext[q]
            if any(ext[a] not in F[rq] for a in range(p, q)): break
            dd = d(ext[p - 1], rq) + d(ext[p], ext[q + 1]) - d(ext[p - 1], ext[p]) - d(rq, ext[q + 1])
            if dd > d_room: continue
            a2, b2 = p, q
            while a2 < b2: buf[a2], buf[b2] = buf[b2], buf[a2]; a2 += 1; b2 -= 1
            ok, info_new = realized(buf)
            a2, b2 = p, q
            while a2 < b2: buf[a2], buf[b2] = buf[b2], buf[a2]; a2 += 1; b2 -= 1
            if not ok: continue
            _s = 0.0; _a, _b = p, q
            while _a < _b:
                _s += (GAM[ext[_a]] - GAM[ext[_b]]) * (amin0[_b] - amin0[_a]); _a += 1; _b -= 1
            moves.append((_s, ("two_opt", p, q), dd))
    out["two_opt"] = moves
    return out, amin0, amax0, d_room

def compare(a, b, tol=1e-6):
    da = {m[1]: m for m in a}; db = {m[1]: m for m in b}
    missing = set(da) - set(db); extra = set(db) - set(da)
    worst = 0.0
    for key in set(da) & set(db):
        worst = max(worst, abs(da[key][0] - db[key][0]) / max(1.0, abs(da[key][0])), abs(da[key][2] - db[key][2]))
    return missing, extra, worst

if __name__ == "__main__":
    stems = sys.argv[1:] or ["r101_100", "c101_100", "rc104_100", "r104_100", "c104_100", "pr15_240", "pr10_288"]
    names = {"r101_100": "R101 (100)", "c101_100": "C101 (100)", "rc104_100": "RC104 (100)", "r104_100": "R104 (100)",
             "c104_100": "C104 (100)", "pr15_240": "PR15 (240)", "pr10_288": "PR10 (288)", "r1_2_1_200": "R1_2_1 (200)"}
    for stem in stems:
        name = names[stem]
        run = TimedILS(name, PATHS[name], 10, local_search="paper")
        nd = fs.NodeData(run.graph, run.depot, run.T_max, run.v_max, run.DM, run.F_leg, run.B_leg)
        routes = load_routes(stem, 30)
        t_old = {o: 0.0 for o in ("add", "replace", "swap", "two_opt")}; t_new = dict(t_old)
        n_moves = dict(t_old); bad = 0
        for route in routes:
            Nprime = set(run.graph.nodes) - set(route) - {run.depot}
            t0 = time.perf_counter(); old, amin0, amax0, d_room = baseline_sets(run, route, Nprime); t1 = time.perf_counter()
            # per-operator timing of the old construction is not separable above; time the total and the new per op
            amin, amax = fs.chain_bounds(nd, route)
            assert max(abs(x - y) for x, y in zip(amin, amin0)) < 1e-9 and max(abs(x - y) for x, y in zip(amax, amax0)) < 1e-9
            new = {}
            for o, fn in (("add", lambda: fs.build_add(nd, route, Nprime, amin, amax, d_room)),
                          ("replace", lambda: fs.build_replace(nd, route, Nprime, amin, amax, d_room)),
                          ("swap", lambda: fs.build_swap(nd, route, amin, amax, d_room)),
                          ("two_opt", lambda: fs.build_two_opt(nd, route, amin, amax, d_room))):
                s0 = time.perf_counter(); new[o] = fn(); t_new[o] += time.perf_counter() - s0
            t_old["add"] += t1 - t0     # total old time accumulated under "add"
            for o in old:
                n_moves[o] += len(old[o])
                missing, extra, worst = compare(old[o], new[o])
                if missing or extra or worst > 1e-6:
                    bad += 1
                    print(f"  MISMATCH {name} k={len(route)-1} {o}: missing {len(missing)} extra {len(extra)} worst {worst:.2e} "
                          f"e.g. {list(missing)[:3]} {list(extra)[:3]}")
        ks = sorted(len(r) - 1 for r in routes)
        print(f"{name:12s} routes {len(routes)} (k {ks[0]}..{ks[-1]}): sets identical on {len(routes)*4 - bad}/{len(routes)*4} operator-sets; "
              f"moves add {n_moves['add']} rep {n_moves['replace']} swap {n_moves['swap']} 2opt {n_moves['two_opt']} | "
              f"old total {t_old['add']:.2f}s  new total {sum(t_new.values()):.2f}s "
              f"(add {t_new['add']:.2f} rep {t_new['replace']:.2f} swap {t_new['swap']:.2f} 2opt {t_new['two_opt']:.2f})", flush=True)
