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


def build_two_opt(nd, route, amin, amax, d_room, arr=None, counters=None, mode=None):
    """Every reversal (p, q), 1 <= p < q <= k, whose route passes the realized-
    window test, decided in O(1) per pair; weight = exchange value summed over the
    reversed pairs, read at the arrivals `arr` (a^min when arr is None).
    mode="mid" (TEST_RUNBOOK.md): the outer pair (r_p, r_q) is read instead at the
    midpoints of its realized windows before and after the move, the new windows
    coming from the segment summaries in O(1); the inner pairs keep their value."""
    depot, EW, LW, GAM, DM, inv_v = nd.depot, nd.EW, nd.LW, nd.GAM, nd.DM, nd.inv_v
    k = len(route) - 1
    if k < 2:
        return []
    if mode in ("midall", "boundall"):
        return build_two_opt_midall(nd, route, amin, amax, d_room, counters,
                                    read=("bound" if mode == "boundall" else "mid"))
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
            if mode == "mid":
                # new realized windows: r_q at position p (entered at aq), r_p at position q (reached at ap)
                amax_u = LW[x] if LW[x] < amax[q + 1] - DM[x][nxt] * inv_v else amax[q + 1] - DM[x][nxt] * inv_v
                amax_v = L if L < amax_u - D else amax_u - D
                x_mid = (GAM[x] * (0.5 * (ap + amax_u) - 0.5 * (amin[p] + amax[p]))
                         + GAM[xq] * (0.5 * (aq + amax_v) - 0.5 * (amin[q] + amax[q])))
                inner = S[p + 1][q - 1] if q - 1 > p + 1 else 0.0
                moves.append((x_mid + inner, ("two_opt", p, q), dd))
                continue
            moves.append((S[p][q], ("two_opt", p, q), dd))
    return moves


def build_swap(nd, route, amin, amax, d_room, arr=None, counters=None, mode=None):
    """Every exchange (p, q) whose route passes the realized-window test, in O(1)
    per pair: the interior r_{p+1..q-1} grows by one target at its end as q
    advances.  Weight = (gamma_p - gamma_q)(a_q - a_p).
    mode="mid" (TEST_RUNBOOK.md): both targets read at the midpoints of their
    realized windows before and after the move, the new windows from the
    interior summary in O(1)."""
    depot, EW, LW, GAM, DM, inv_v = nd.depot, nd.EW, nd.LW, nd.GAM, nd.DM, nd.inv_v
    k = len(route) - 1
    if k < 2:
        return []
    if mode in ("midall", "boundall"):
        return build_swap_midall(nd, route, amin, amax, d_room, counters,
                                 read=("bound" if mode == "boundall" else "mid"))
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
            if mode == "mid":
                # new realized windows: r_q at position p (entered at aq), r_p at position q (reached at ap)
                amax_u = LW[x] if LW[x] < amax[q + 1] - DM[x][nxt] * inv_v else amax[q + 1] - DM[x][nxt] * inv_v
                if q == p + 1:
                    amax_v = amax_u - DM[xq][x] * inv_v
                else:
                    c = ext[p + 1]
                    amax_v = min(L - DM[xq][c] * inv_v, amax_u - DM[xq][c] * inv_v - D - DM[ext[q - 1]][x] * inv_v)
                if amax_v > LW[xq]: amax_v = LW[xq]
                x_mid = (gx * (0.5 * (ap + amax_u) - 0.5 * (amin[p] + amax[p]))
                         + GAM[xq] * (0.5 * (aq + amax_v) - 0.5 * (amin[q] + amax[q])))
                moves.append((x_mid, ("swap", p, q), dd))
                continue
            moves.append(((gx - GAM[xq]) * (A[q] - ax), ("swap", p, q), dd))
    return moves


# ---------------------------------------------------------------------------
# TEST MODE "midall" (experiments/TEST_RUNBOOK.md): every target of the reordered
# part is read at the midpoint of its realized window after the move minus before.
# The windows after the move come from tables of nested segment summaries, O(1)
# per target, so a feasible pair costs O(q - p) instead of O(1).
# ---------------------------------------------------------------------------

def _readings(nd, ext, amin, amax, read):
    """The point of its realized window each target is read at.  read="mid" is the
    midpoint of (55); read="bound" is the end that maximises the reward, which for
    I_i(a) = gamma_i (a - e_i) + I_0i is the upper bound when gamma_i > 0 and the
    lower one when gamma_i < 0.  Returns the reading before the move, position by
    position, and, for "bound", the flag saying which end to read after it."""
    GAM = nd.GAM
    if read == "bound":
        up = [GAM[ext[j]] > 0.0 for j in range(len(ext))]
        return [(amax[j] if up[j] else amin[j]) for j in range(len(ext))], up
    return [0.5 * (a + b) for a, b in zip(amin, amax)], None


def _forward_tables(nd, route):
    """Summaries of every forward segment (r_j, ..., r_i), 1 <= j <= i <= k, by the
    append rule: Df = travel time at v_max, Wf = earliest arrival at r_i forced by the
    windows inside, Lf = latest arrival at r_j meeting every window inside (-INF when
    the segment misses a window however early it is entered)."""
    EW, LW, DM, inv_v = nd.EW, nd.LW, nd.DM, nd.inv_v
    k = len(route) - 1
    Df = [[0.0] * (k + 1) for _ in range(k + 1)]
    Wf = [[-INF] * (k + 1) for _ in range(k + 1)]
    Lf = [[-INF] * (k + 1) for _ in range(k + 1)]
    for j in range(1, k + 1):
        sj = route[j]; D, W, L, last = 0.0, EW[sj], LW[sj], sj
        Df[j][j], Wf[j][j], Lf[j][j] = D, W, L
        for i in range(j + 1, k + 1):
            z = route[i]; tau = DM[last][z] * inv_v
            if W + tau > LW[z]:
                break                          # every longer segment misses this window too
            Lz = LW[z] - D - tau
            if Lz < L: L = Lz
            W = W + tau if W + tau > EW[z] else EW[z]
            D += tau; last = z
            Df[j][i], Wf[j][i], Lf[j][i] = D, W, L
    return Df, Wf, Lf


def build_swap_midall(nd, route, amin, amax, d_room, counters=None, read="mid"):
    """Same feasible set as build_swap; score = sum over the reordered part
    (r_q, r_{p+1}, ..., r_{q-1}, r_p) of gamma times the change of the point each
    target is read at.  read="mid" takes the midpoint of the realized window,
    read="bound" ("boundall") the end that maximises the reward: the upper bound
    for a positive slope, the lower for a negative one."""
    depot, EW, LW, GAM, DM, inv_v = nd.depot, nd.EW, nd.LW, nd.GAM, nd.DM, nd.inv_v
    k = len(route) - 1
    if k < 2:
        return []
    ext = route + [depot]
    Df, Wf, Lf = _forward_tables(nd, route)
    mid_old, up = _readings(nd, ext, amin, amax, read)
    moves = []
    for p in range(1, k):
        x = ext[p]; pre = ext[p - 1]; gx = GAM[x]
        for q in range(p + 1, k + 1):
            xq = ext[q]; nxt = ext[q + 1]
            if q > p + 1 and (Lf[p + 1][q - 1] == -INF or Lf[p + 1][q - 1] < EW[ext[p + 1]]):
                break                          # the interior cannot be flown on time
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
            amax_u = amax[q + 1] - DM[x][nxt] * inv_v
            if amax_u > LW[x]: amax_u = LW[x]
            inner = 0.0
            if q == p + 1:
                ap = aq + DM[xq][x] * inv_v
                if ap < EW[x]: ap = EW[x]
                if ap > LW[x] or ap + DM[x][nxt] * inv_v > amax[q + 1]:
                    if counters is not None: counters["label_excluded"] += 1
                    continue
                amax_v = amax_u - DM[xq][x] * inv_v
            else:
                c, e = ext[p + 1], ext[q - 1]
                af = aq + DM[xq][c] * inv_v
                if af < EW[c]: af = EW[c]
                if af > Lf[p + 1][q - 1]:
                    if counters is not None: counters["label_excluded"] += 1
                    continue
                D, W = Df[p + 1][q - 1], Wf[p + 1][q - 1]
                al = af + D
                if al < W: al = W
                tau_e = DM[e][x] * inv_v
                ap = al + tau_e
                if ap < EW[x]: ap = EW[x]
                if ap > LW[x] or ap + DM[x][nxt] * inv_v > amax[q + 1]:
                    if counters is not None: counters["label_excluded"] += 1
                    continue
                tau_c = DM[xq][c] * inv_v
                amax_v = min(Lf[p + 1][q - 1] - tau_c, amax_u - tau_c - D - tau_e)
                for j in range(p + 1, q):          # the interior, re-timed by the new legs
                    lo = af + Df[p + 1][j]
                    if lo < Wf[p + 1][j]: lo = Wf[p + 1][j]
                    hi = amax_u - tau_e - Df[j][q - 1]
                    if hi > Lf[j][q - 1]: hi = Lf[j][q - 1]
                    new_j = (hi if up[j] else lo) if up is not None else 0.5 * (lo + hi)
                    inner += GAM[ext[j]] * (new_j - mid_old[j])
            if amax_v > LW[xq]: amax_v = LW[xq]
            if up is not None:
                new_p = amax_u if up[p] else ap
                new_q = amax_v if up[q] else aq
            else:
                new_p, new_q = 0.5 * (ap + amax_u), 0.5 * (aq + amax_v)
            score = (gx * (new_p - mid_old[p])
                     + GAM[xq] * (new_q - mid_old[q]) + inner)
            moves.append((score, ("swap", p, q), dd))
    return moves


def build_two_opt_midall(nd, route, amin, amax, d_room, counters=None, read="mid"):
    """Same feasible set as build_two_opt; score = sum over the reversed segment
    (r_q, ..., r_p) of gamma times the change of the point each target is read at,
    the midpoint of its realized window for read="mid" and the reward-maximising end
    for read="bound" ("boundall").  The tables of the reversed segments are filled
    with p descending, so the inner segments (r_q, ..., r_j), j > p, exist when the
    pair (p, q) is scored."""
    depot, EW, LW, GAM, DM, inv_v = nd.depot, nd.EW, nd.LW, nd.GAM, nd.DM, nd.inv_v
    k = len(route) - 1
    if k < 2:
        return []
    ext = route + [depot]
    Dt = [[0.0] * (k + 2) for _ in range(k + 2)]
    Wt = [[-INF] * (k + 2) for _ in range(k + 2)]
    Lt = [[-INF] * (k + 2) for _ in range(k + 2)]
    mid_old, up = _readings(nd, ext, amin, amax, read)
    moves = []
    for p in range(k - 1, 0, -1):
        x = ext[p]; pre = ext[p - 1]; gx = GAM[x]
        d_pre_x = DM[pre][x]
        D, W, L = 0.0, -INF, LW[x]
        first = x
        Dt[p][p], Wt[p][p], Lt[p][p] = D, W, L
        for q in range(p + 1, k + 1):
            y = first; xq = ext[q]
            if EW[y] > L:
                break
            tau = DM[xq][y] * inv_v
            L = LW[xq] if LW[xq] < L - tau else L - tau
            W = max(EW[y] + D, W)
            D += tau
            first = xq
            if L < EW[xq]:
                break
            Dt[p][q], Wt[p][q], Lt[p][q] = D, W, L
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
            amax_u = amax[q + 1] - DM[x][nxt] * inv_v
            if amax_u > LW[x]: amax_u = LW[x]
            amax_v = L if L < amax_u - D else amax_u - D
            inner = 0.0
            for j in range(p + 1, q):              # r_j moves to position p + q - j
                lo = aq + Dt[j][q]
                if lo < Wt[j][q]: lo = Wt[j][q]
                hi = amax_u - Dt[p][j]
                if hi > Lt[p][j]: hi = Lt[p][j]
                new_j = (hi if up[j] else lo) if up is not None else 0.5 * (lo + hi)
                inner += GAM[ext[j]] * (new_j - mid_old[j])
            if up is not None:
                new_p = amax_u if up[p] else ap
                new_q = amax_v if up[q] else aq
            else:
                new_p, new_q = 0.5 * (ap + amax_u), 0.5 * (aq + amax_v)
            score = (gx * (new_p - mid_old[p])
                     + GAM[xq] * (new_q - mid_old[q]) + inner)
            moves.append((score, ("two_opt", p, q), dd))
    return moves
