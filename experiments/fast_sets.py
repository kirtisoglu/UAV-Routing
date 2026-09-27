"""
Feasible move sets of the four operators, built in O(1) per candidate.

Same sets and same weights as the construction in run_ils_time_matched.py
(run_start_paper with ILS_INSERT_RATIO=1, ILS_REORDER_W=exch), but

* the realized-window test of a Swap or 2-opt is decided in O(1) per pair by
  concatenating the unchanged prefix, the reordered segment and the unchanged
  suffix, the segment being summarized by three numbers (travel duration D,
  waiting floor W, latest feasible entry L) that are updated in O(1) as the
  segment grows by one target (Savelsbergh 1992; Kindervater and Savelsbergh
  1997);
* the route-wide change of the midpoint information estimate caused by an
  Insert or a Replace is obtained by propagating the shift of a^min forward
  and of a^max backward only as far as it reaches, the recursion stopping at
  the first position whose bound is unchanged;
* the 2-opt exchange weight, summed over the reversed pairs, is computed for
  all pairs at once by S(p, q) = x(p, q) + S(p + 1, q - 1).

A route is `route = [depot, r_1, ..., r_k]`; `ext = route + [depot]`.
"""
INF = float("inf")


class NodeData:
    """Flat per-node data and the distance matrix, built once per instance."""

    def __init__(self, graph, depot, T_max, v_max, DM, F_leg, B_leg):
        self.depot, self.T_max, self.inv_v, self.DM = depot, T_max, 1.0 / v_max, DM
        self.F, self.B = F_leg, B_leg
        self.EW, self.LW, self.I0, self.GAM = {}, {}, {}, {}
        for n in graph.nodes:
            e, l = graph.nodes[n]["time_window"]
            self.EW[n], self.LW[n] = float(e), float(l)
            self.I0[n] = float(graph.nodes[n].get("info_at_lowest", 1.0))
            self.GAM[n] = float(graph.nodes[n].get("info_slope", 0.0))
        self.EW[depot], self.LW[depot], self.I0[depot], self.GAM[depot] = 0.0, float(T_max), 0.0, 0.0


def chain_bounds(nd, route):
    """a^min forward and a^max backward over ext = route + [depot], O(k)."""
    depot, EW, LW, DM, inv_v, T_max = nd.depot, nd.EW, nd.LW, nd.DM, nd.inv_v, nd.T_max
    n = len(route)
    amin = [0.0] * (n + 1); amax = [0.0] * (n + 1)
    prev = 0.0
    for q in range(1, n):
        nq = route[q]
        prev = max(EW[nq], prev + DM[route[q - 1]][nq] * inv_v)
        amin[q] = prev
    amin[n] = prev + DM[route[-1]][depot] * inv_v
    amax[n] = T_max
    nxt = T_max - DM[route[-1]][depot] * inv_v
    for q in range(n - 1, 0, -1):
        nq = route[q]
        hi = LW[nq] if LW[nq] < nxt else nxt
        amax[q] = hi
        nxt = hi - DM[route[q - 1]][nq] * inv_v
    amax[0] = nxt
    return amin, amax


def chained_sets(route, F, B, depot):
    """FR[p] = F_{r_0} & ... & F_{r_{p-1}}, BR[p] = B_{r_p} & ... & B_{r_{k+1}}."""
    k = len(route) - 1
    FR = [None] * (k + 2); BR = [None] * (k + 3)
    acc = set(F[depot]); FR[1] = acc
    for p in range(2, k + 2):
        acc = acc & F[route[p - 1]]; FR[p] = acc
    acc = set(B[depot]); BR[k + 1] = acc
    for p in range(k, 0, -1):
        acc = acc & B[route[p]]; BR[p] = acc
    return FR, BR


def delta_insert(nd, ext, k, amin, amax, p, u, lo, hi):
    """Route-wide change of the midpoint estimate for u inserted at p with
    realized slot window [lo, hi]; propagation stops where the bound is unchanged."""
    EW, LW, GAM, DM, inv_v = nd.EW, nd.LW, nd.GAM, nd.DM, nd.inv_v
    dI = nd.I0[u] + GAM[u] * (0.5 * (lo + hi) - EW[u])
    prev, pn = lo, u
    for q in range(p, k + 1):
        nq = ext[q]
        a = prev + DM[pn][nq] * inv_v
        if a < EW[nq]: a = EW[nq]
        if a <= amin[q] + 1e-9:
            break
        dI += GAM[nq] * 0.5 * (a - amin[q])
        prev, pn = a, nq
    nxt, nn = hi, u
    for q in range(p - 1, 0, -1):
        nq = ext[q]
        b = nxt - DM[nq][nn] * inv_v
        if b > LW[nq]: b = LW[nq]
        if b >= amax[q] - 1e-9:
            break
        dI += GAM[nq] * 0.5 * (b - amax[q])
        nxt, nn = b, nq
    return dI


def delta_replace(nd, ext, k, amin, amax, p, u, lo, hi):
    """Route-wide change of the midpoint estimate for u in place of r_p."""
    EW, LW, GAM, DM, inv_v = nd.EW, nd.LW, nd.GAM, nd.DM, nd.inv_v
    v = ext[p]
    dI = (nd.I0[u] + GAM[u] * (0.5 * (lo + hi) - EW[u])
          - (nd.I0[v] + GAM[v] * (0.5 * (amin[p] + amax[p]) - EW[v])))
    prev, pn = lo, u
    for q in range(p + 1, k + 1):
        nq = ext[q]
        a = prev + DM[pn][nq] * inv_v
        if a < EW[nq]: a = EW[nq]
        if abs(a - amin[q]) <= 1e-9:
            break
        dI += GAM[nq] * 0.5 * (a - amin[q])
        prev, pn = a, nq
    nxt, nn = hi, u
    for q in range(p - 1, 0, -1):
        nq = ext[q]
        b = nxt - DM[nq][nn] * inv_v
        if b > LW[nq]: b = LW[nq]
        if abs(b - amax[q]) <= 1e-9:
            break
        dI += GAM[nq] * 0.5 * (b - amax[q])
        nxt, nn = b, nq
    return dI


def build_add(nd, route, Nprime, amin, amax, d_room, counters=None):
    """[(score, ("add", u, p), dd)], score = dI / dd (dI if dd ~ 0), the weight of
    run_start_paper with ILS_INSERT_RATIO=1 and route-wide information change."""
    depot, EW, LW, DM, inv_v = nd.depot, nd.EW, nd.LW, nd.DM, nd.inv_v
    k = len(route) - 1
    ext = route + [depot]
    FR, BR = chained_sets(route, nd.F, nd.B, depot)
    moves = []
    for p in range(1, k + 2):
        i, j = ext[p - 1], ext[p]
        cand = FR[p] & BR[p] & Nprime
        if not cand:
            continue
        di, dj, dij = DM[i], DM[j], DM[i][j]
        a_i, a_j = amin[p - 1], amax[p]
        for u in cand:
            dd = di[u] + dj[u] - dij
            if dd > d_room:
                if counters is not None: counters["floor_excluded"] += 1
                continue
            lo = a_i + di[u] * inv_v
            if lo < EW[u]: lo = EW[u]
            hi = a_j - dj[u] * inv_v
            if hi > LW[u]: hi = LW[u]
            if lo > hi:
                if counters is not None: counters["label_excluded"] += 1
                continue
            dI = delta_insert(nd, ext, k, amin, amax, p, u, lo, hi)
            moves.append((dI / dd if dd > 1e-9 else dI, ("add", u, p), dd))
    return moves


def build_replace(nd, route, Nprime, amin, amax, d_room, counters=None):
    """[(score, ("replace", u, p), dd)], score = dI / dd_w (dI if dd_w ~ 0) with
    dd_w the detour of u into the gap left by r_p and dd the change of the route
    length, as in run_start_paper."""
    depot, EW, LW, DM, inv_v = nd.depot, nd.EW, nd.LW, nd.DM, nd.inv_v
    k = len(route) - 1
    ext = route + [depot]
    FR, BR = chained_sets(route, nd.F, nd.B, depot)
    moves = []
    for p in range(1, k + 1):
        i, v, j = ext[p - 1], ext[p], ext[p + 1]
        cand = FR[p] & BR[p + 1] & Nprime
        if not cand:
            continue
        di, dj = DM[i], DM[j]
        base = di[v] + dj[v]
        gap = di[j]
        a_i, a_j = amin[p - 1], amax[p + 1]
        for u in cand:
            dd = di[u] + dj[u] - base
            dd_w = di[u] + dj[u] - gap
            if dd > d_room:
                if counters is not None: counters["floor_excluded"] += 1
                continue
            lo = a_i + di[u] * inv_v
            if lo < EW[u]: lo = EW[u]
            hi = a_j - dj[u] * inv_v
            if hi > LW[u]: hi = LW[u]
            if lo > hi:
                if counters is not None: counters["label_excluded"] += 1
                continue
            dI = delta_replace(nd, ext, k, amin, amax, p, u, lo, hi)
            moves.append((dI / dd_w if dd_w > 1e-9 else dI, ("replace", u, p), dd))
    return moves


def build_two_opt(nd, route, amin, amax, d_room, arr=None, counters=None):
    """Every reversal (p, q), 1 <= p < q <= k, whose route passes the realized-
    window test, decided in O(1) per pair; weight = exchange value summed over the
    reversed pairs, read at the arrivals `arr` (a^min when arr is None)."""
    depot, EW, LW, GAM, DM, inv_v = nd.depot, nd.EW, nd.LW, nd.GAM, nd.DM, nd.inv_v
    k = len(route) - 1
    if k < 2:
        return []
    ext = route + [depot]
    A = amin if arr is None else arr
    S = [[0.0] * (k + 2) for _ in range(k + 2)]
    for gap in range(1, k):
        for p in range(1, k - gap + 1):
            q = p + gap
            S[p][q] = (GAM[ext[p]] - GAM[ext[q]]) * (A[q] - A[p]) + (S[p + 1][q - 1] if q - 1 > p + 1 else 0.0)
    moves = []
    for p in range(1, k):
        x = ext[p]; pre = ext[p - 1]
        d_pre_x = DM[pre][x]
        D, W, L = 0.0, -INF, LW[x]            # the reversed segment (r_p) alone
        first = x
        for q in range(p + 1, k + 1):
            y = first; xq = ext[q]
            if EW[y] > L:
                break                         # the segment cannot be entered on time
            tau = DM[xq][y] * inv_v
            L = LW[xq] if LW[xq] < L - tau else L - tau
            W = max(EW[y] + D, W)
            D += tau
            first = xq
            if L < EW[xq]:
                break                         # nor can any longer one
            nxt = ext[q + 1]
            dd = DM[pre][xq] + DM[x][nxt] - d_pre_x - DM[xq][nxt]
            if dd > d_room:
                if counters is not None: counters["floor_excluded"] += 1
                continue
            aq = amin[p - 1] + DM[pre][xq] * inv_v
            if aq < EW[xq]: aq = EW[xq]
            if aq > L:
                if counters is not None: counters["label_excluded"] += 1
                continue
            ap = aq + D
            if ap < W: ap = W
            if ap + DM[x][nxt] * inv_v > amax[q + 1]:
                if counters is not None: counters["label_excluded"] += 1
                continue
            moves.append((S[p][q], ("two_opt", p, q), dd))
    return moves


def build_swap(nd, route, amin, amax, d_room, arr=None, counters=None):
    """Every exchange (p, q) whose route passes the realized-window test, in O(1)
    per pair: the interior r_{p+1..q-1} grows by one target at its end as q
    advances.  Weight = (gamma_p - gamma_q)(a_q - a_p)."""
    depot, EW, LW, GAM, DM, inv_v = nd.depot, nd.EW, nd.LW, nd.GAM, nd.DM, nd.inv_v
    k = len(route) - 1
    if k < 2:
        return []
    ext = route + [depot]
    A = amin if arr is None else arr
    moves = []
    for p in range(1, k):
        x = ext[p]; pre = ext[p - 1]
        gx, ax = GAM[x], A[p]
        D = W = L = None; last = None
        for q in range(p + 1, k + 1):
            xq = ext[q]; nxt = ext[q + 1]
            if q > p + 1:
                z = ext[q - 1]                # joins the interior
                if q - 1 == p + 1:
                    D, W, L, last = 0.0, -INF, LW[z], z
                else:
                    tau = DM[last][z] * inv_v
                    if W + tau > LW[z]:
                        break                 # the interior cannot be flown on time
                    Lz = LW[z] - D - tau
                    if Lz < L: L = Lz
                    Wz = W + tau
                    W = Wz if Wz > EW[z] else EW[z]
                    D += tau; last = z
                if L < EW[ext[p + 1]]:
                    break
            if q == p + 1:
                dd = DM[pre][xq] + DM[x][nxt] - DM[pre][x] - DM[xq][nxt]
            else:
                c, e = ext[p + 1], ext[q - 1]
                dd = (DM[pre][xq] + DM[xq][c] + DM[e][x] + DM[x][nxt]
                      - DM[pre][x] - DM[x][c] - DM[e][xq] - DM[xq][nxt])
            if dd > d_room:
                if counters is not None: counters["floor_excluded"] += 1
                continue
            aq = amin[p - 1] + DM[pre][xq] * inv_v
            if aq < EW[xq]: aq = EW[xq]
            if aq > LW[xq]:
                if counters is not None: counters["label_excluded"] += 1
                continue
            if q == p + 1:
                ap = aq + DM[xq][x] * inv_v
            else:
                c = ext[p + 1]
                af = aq + DM[xq][c] * inv_v
                if af < EW[c]: af = EW[c]
                if af > L:
                    if counters is not None: counters["label_excluded"] += 1
                    continue
                al = af + D
                if al < W: al = W
                ap = al + DM[ext[q - 1]][x] * inv_v
            if ap < EW[x]: ap = EW[x]
            if ap > LW[x] or ap + DM[x][nxt] * inv_v > amax[q + 1]:
                if counters is not None: counters["label_excluded"] += 1
                continue
            moves.append(((gx - GAM[xq]) * (A[q] - ax), ("swap", p, q), dd))
    return moves
