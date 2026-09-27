"""
TESTED AND REJECTED (2026-09-27): iterated local search whose descent is a variable
neighborhood descent over restricted candidate lists of every operator.  Kept as
the ablation that motivates the design of Section 4: on R101 (50) it reaches
10 850 (roulette within the lists: 11 717) against 11 902 for the random-order
full descent, and on R101 (100) 22 386 (22 427) against 22 884, because the
acceptance rate of Insert and Replace does not decay with the weight rank (see
experiments/DESIGN.md).  The pieces that survived (O(1) reordering tests,
propagated estimates, explicit shake enumeration, idle-shake termination) live in
experiments/fast_sets.py and run_ils_time_matched.py.

Differences from the design run by run_ils_time_matched.py --local-search paper:

1. Descent by variable neighborhood descent over four neighborhoods in a fixed
   order (Insert, Replace, 2-opt, Swap).  For the current route the feasible set
   of a neighborhood is built exactly as before (leg sets, exact slot test,
   energy floor), every move is weighted, and only the L moves of largest weight
   (the restricted candidate list, RCL) are evaluated by the SOCP, in weight
   order, first improvement.  An accepted move restarts the descent at Insert;
   an RCL evaluated without an acceptance passes to the next neighborhood; a
   route whose four RCLs are all evaluated without an acceptance is a local
   optimum and is shaken.  There is no stall parameter: the descent is short by
   construction, so the shake fires at every local optimum.

2. Every move is tested for time-window feasibility in O(1).  Insert and
   Replace by the slot test on the arrival bounds of the current route, Swap
   and 2-opt by concatenating the unchanged prefix, the reordered segment and
   the unchanged suffix, where the segment is summarized by three numbers
   (Savelsbergh 1992) that are updated in O(1) as the segment grows.  The
   weight of a 2-opt is the summed exchange value of the reversed pairs,
   computed for all pairs by a dynamic program in O(k^2).

3. The weight of an Insert is the estimated reward of the inserted target at
   the midpoint of its realized slot window over the detour; a Replace is
   charged the reward of the displaced target in the current schedule.

4. Termination by S consecutive shakes without an improvement of the best
   found route (or the exhaustion of the reference stack), wall clock as a
   safeguard.  The trace records the shake count at every improvement, so the
   objective and stopping time of any S can be read from one run.

Everything else is inherited from TimedILS: instance construction, the SOCP
evaluation with the evaluated-route store and the taut-string energy test, the
sweep shake on a reference route with the knapsack look-ahead, the reference
stack.

Usage (the paper's configuration):
  python3 experiments/run_ils_rcl.py --instance "R104 (100)" --rcl 10 \
      --cap-div 3 --lookahead 6 --max-idle-shakes 150 --budget 3600

ILS_LICENSE_GUARD=1 makes a Gurobi size-limited-license refusal count as an
infeasible route instead of aborting the run; only for smoke tests on a
machine without a full license, never for reported results.
"""
import os, sys, csv, time, random, argparse
from collections import deque

_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
sys.path.insert(0, os.path.join(_repo_root, "experiments"))
os.chdir(_repo_root)

import gurobipy as gp

from run_ils_time_matched import (TimedILS, chained_sets,
                                  route_distance, _MISS)
from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES
from uav_routing.local_search.initial_solution import build_R1, build_R2, build_R3, build_R4
from uav_routing.local_search.state import State
from uav_routing.local_search.proposal import route_to_nx

LICENSE_GUARD = bool(os.environ.get("ILS_LICENSE_GUARD"))
INF = float("inf")
OPS_DEFAULT = ("add", "replace", "two_opt", "swap")


class RCLILS(TimedILS):
    """Iterated local search of Section 4 with restricted candidate lists."""

    def __init__(self, name, path, budget, rcl=10, cap_div=3, lookahead=6,
                 max_idle_shakes=150, order=OPS_DEFAULT, draw="order",
                 info_bound=True, seed=42, **kw):
        kw.setdefault("local_search", "paper")       # builds the leg sets F_i, B_i
        super().__init__(name, path, budget, sweep_cap_div=cap_div, **kw)
        self.rcl_size = int(rcl)
        self.lookahead = int(lookahead)
        self.max_idle_shakes = int(max_idle_shakes)
        self.order = tuple(order)
        self.draw = draw                    # "order": weight order; "roulette": drawn from the RCL
        self.info_bound = info_bound
        self.seed = seed
        self.idle_shakes = 0
        self.trace2 = []                    # (wall, iter, shakes, best)
        self.stop_reason = None
        # flat node data, hoisted out of the hot loops
        G, depot = self.graph, self.depot
        self.EW, self.LW, self.I0, self.GAM = {}, {}, {}, {}
        for n in G.nodes:
            e, l = G.nodes[n]["time_window"]
            self.EW[n], self.LW[n] = float(e), float(l)
            self.I0[n] = float(G.nodes[n].get("info_at_lowest", 1.0))
            self.GAM[n] = float(G.nodes[n].get("info_slope", 0.0))
        self.EW[depot], self.LW[depot], self.I0[depot], self.GAM[depot] = 0.0, self.T_max, 0.0, 0.0
        self.inv_v = 1.0 / self.v_max

    # ------------------------------------------------------------------ store
    def _record_best(self, obj, route):
        super()._record_best(obj, route)
        self.idle_shakes = 0
        self.trace2.append((self.best_wall, self.iter_global, self.counters["kicks"], obj))

    def evaluate(self, state, new_route):
        if not LICENSE_GUARD:
            return super().evaluate(state, new_route)
        try:
            return super().evaluate(state, new_route)
        except gp.GurobiError as exc:
            if "size-limited" in str(exc):
                self.counters["license_reject"] += 1
                return None, None, "infeasible"
            raise

    def _resolve(self, state, new_route):
        if not LICENSE_GUARD:
            return super()._resolve(state, new_route)
        try:
            return super()._resolve(state, new_route)
        except gp.GurobiError as exc:
            if "size-limited" in str(exc):
                self.counters["license_reject"] += 1
                return None, None, "infeasible"
            raise

    # ------------------------------------------------------- route quantities
    def _context(self, state):
        """Arrival bounds (leg-chain), the schedule of the solved route and the
        room the energy floor leaves, all O(k)."""
        route = state.solver.tour_nodes
        depot = self.depot
        k = len(route) - 1
        ext = route + [depot]
        EW, LW, DM, inv_v, T_max = self.EW, self.LW, self.DM, self.inv_v, self.T_max
        amin = [0.0] * (k + 2); amax = [0.0] * (k + 2)
        prev = 0.0
        for q in range(1, k + 1):
            nq = ext[q]
            prev = max(EW[nq], prev + DM[ext[q - 1]][nq] * inv_v)
            amin[q] = prev
        amin[k + 1] = prev + DM[ext[k]][depot] * inv_v
        amax[k + 1] = T_max
        nxt = T_max - DM[ext[k]][depot] * inv_v
        for q in range(k, 0, -1):
            nq = ext[q]
            hi = LW[nq] if LW[nq] < nxt else nxt
            amax[q] = hi
            nxt = hi - DM[ext[q - 1]][nq] * inv_v
        amax[0] = nxt
        # the schedule the subproblem returned for R: arrival at every visit
        sv = state.solver
        arr = [0.0] * (k + 2)
        rew = [0.0] * (k + 2)
        for q in range(1, k + 1):
            nq = ext[q]
            try:
                a = float(sv.arrival(nq))
            except Exception:
                a = amin[q]
            arr[q] = a
            rew[q] = self.I0[nq] + self.GAM[nq] * (a - EW[nq])
        f, T_R, E_R, cap = self._route_stats(state)
        d_room = self.E_max / self.e_per_m - route_distance(route, self.graph, depot)
        return dict(route=route, ext=ext, k=k, amin=amin, amax=amax, arr=arr, rew=rew,
                    f=f, cap=cap, d_room=d_room)

    # ------------------------------------------------------------ feasible sets
    def _delta_insert(self, ctx, p, u, lo, hi):
        """Change of the route-wide midpoint estimate when u is inserted at
        position p with realized slot window [lo, hi].  The forward shift of
        a^min and the backward shift of a^max are propagated only as far as
        they reach: the recursion stops at the first position whose bound is
        unchanged, since every bound beyond it is unchanged too."""
        ext, amin, amax, k = ctx["ext"], ctx["amin"], ctx["amax"], ctx["k"]
        EW, LW, GAM, DM, inv_v = self.EW, self.LW, self.GAM, self.DM, self.inv_v
        dI = self.I0[u] + GAM[u] * (0.5 * (lo + hi) - EW[u])
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

    def _delta_replace(self, ctx, p, u, lo, hi):
        """Change of the route-wide midpoint estimate when r_p is replaced by u,
        whose realized slot window in R (-) r_p is [lo, hi].  The shifts may go
        either way and are propagated until a bound is unchanged."""
        ext, amin, amax, k = ctx["ext"], ctx["amin"], ctx["amax"], ctx["k"]
        EW, LW, GAM, DM, inv_v = self.EW, self.LW, self.GAM, self.DM, self.inv_v
        v = ext[p]
        dI = (self.I0[u] + GAM[u] * (0.5 * (lo + hi) - EW[u])
              - (self.I0[v] + GAM[v] * (0.5 * (amin[p] + amax[p]) - EW[v])))
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

    def _set_add(self, ctx, Nprime):
        route, ext, k = ctx["route"], ctx["ext"], ctx["k"]
        amin, amax, d_room = ctx["amin"], ctx["amax"], ctx["d_room"]
        EW, LW, DM, inv_v, d0 = self.EW, self.LW, self.DM, self.inv_v, self.d_floor
        FR, BR = chained_sets(route, self.F_leg, self.B_leg, self.depot)
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
                    self.counters["floor_excluded"] += 1
                    continue
                lo = a_i + di[u] * inv_v
                if lo < EW[u]: lo = EW[u]
                hi = a_j - dj[u] * inv_v
                if hi > LW[u]: hi = LW[u]
                if lo > hi:
                    self.counters["label_excluded"] += 1
                    continue
                dI = self._delta_insert(ctx, p, u, lo, hi)
                moves.append((dI / (d0 + dd), ("add", u, p), dd))
        return moves

    def _set_replace(self, ctx, Nprime):
        route, ext, k = ctx["route"], ctx["ext"], ctx["k"]
        amin, amax, d_room = ctx["amin"], ctx["amax"], ctx["d_room"]
        EW, LW, DM, inv_v, d0 = self.EW, self.LW, self.DM, self.inv_v, self.d_floor
        FR, BR = chained_sets(route, self.F_leg, self.B_leg, self.depot)
        moves = []
        for p in range(1, k + 1):
            i, v, j = ext[p - 1], ext[p], ext[p + 1]
            cand = FR[p] & BR[p + 1] & Nprime
            if not cand:
                continue
            di, dj = DM[i], DM[j]
            base = di[v] + dj[v]
            a_i, a_j = amin[p - 1], amax[p + 1]
            for u in cand:
                dd = di[u] + dj[u] - base                  # change of the route length
                if dd > d_room:
                    self.counters["floor_excluded"] += 1
                    continue
                lo = a_i + di[u] * inv_v
                if lo < EW[u]: lo = EW[u]
                hi = a_j - dj[u] * inv_v
                if hi > LW[u]: hi = LW[u]
                if lo > hi:
                    self.counters["label_excluded"] += 1
                    continue
                dI = self._delta_replace(ctx, p, u, lo, hi)
                moves.append((dI / (d0 + (dd if dd > 0.0 else 0.0)), ("replace", u, p), dd))
        return moves

    def _set_two_opt(self, ctx):
        """Every reversal (p, q) that passes the realized-window test, in O(1) per
        pair: the reversed segment is summarized by (D, W, L) and grows by one node
        at its front as q advances.  Weight: exchange value summed over the
        reversed pairs, by the recursion S(p, q) = x(p, q) + S(p + 1, q - 1)."""
        ext, k = ctx["ext"], ctx["k"]
        amin, amax, arr, d_room = ctx["amin"], ctx["amax"], ctx["arr"], ctx["d_room"]
        EW, LW, GAM, DM, inv_v = self.EW, self.LW, self.GAM, self.DM, self.inv_v
        if k < 2:
            return []
        # exchange values of all pairs, by increasing gap
        S = [[0.0] * (k + 2) for _ in range(k + 2)]
        for gap in range(1, k):
            for p in range(1, k - gap + 1):
                q = p + gap
                x = (GAM[ext[p]] - GAM[ext[q]]) * (arr[q] - arr[p])
                S[p][q] = x + (S[p + 1][q - 1] if q - 1 > p + 1 else 0.0)
        moves = []
        for p in range(1, k):
            x = ext[p]
            pre = ext[p - 1]
            d_pre_x = DM[pre][x]
            D, W, L = 0.0, -INF, LW[x]          # the segment (r_p) alone
            first = x
            for q in range(p + 1, k + 1):
                y = first; xq = ext[q]
                if EW[y] > L:
                    break                        # the segment cannot be entered on time
                tau = DM[xq][y] * inv_v
                L = LW[xq] if LW[xq] < L - tau else L - tau
                W = max(EW[y] + D, W)
                D += tau
                first = xq
                if L < EW[xq]:
                    break                        # nor can any longer one
                nxt = ext[q + 1]
                dd = DM[pre][xq] + DM[x][nxt] - d_pre_x - DM[xq][nxt]
                if dd > d_room:
                    self.counters["floor_excluded"] += 1
                    continue
                aq = amin[p - 1] + DM[pre][xq] * inv_v
                if aq < EW[xq]: aq = EW[xq]
                if aq > L:
                    self.counters["label_excluded"] += 1
                    continue
                ap = aq + D
                if ap < W: ap = W
                if ap + DM[x][nxt] * inv_v > amax[q + 1]:
                    self.counters["label_excluded"] += 1
                    continue
                w = S[p][q]
                moves.append((w if w > 0.0 else 0.0, ("two_opt", p, q), dd))
        return moves

    def _set_swap(self, ctx):
        """Every exchange (p, q) that passes the realized-window test, in O(1) per
        pair: the interior r_{p+1..q-1} is summarized by (D, W, L) and grows by
        one node at its end as q advances."""
        ext, k = ctx["ext"], ctx["k"]
        amin, amax, arr, d_room = ctx["amin"], ctx["amax"], ctx["arr"], ctx["d_room"]
        EW, LW, GAM, DM, inv_v = self.EW, self.LW, self.GAM, self.DM, self.inv_v
        if k < 2:
            return []
        moves = []
        for p in range(1, k):
            x = ext[p]; pre = ext[p - 1]
            gx, ax = GAM[x], arr[p]
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
                    self.counters["floor_excluded"] += 1
                    continue
                aq = amin[p - 1] + DM[pre][xq] * inv_v
                if aq < EW[xq]: aq = EW[xq]
                if aq > LW[xq]:
                    self.counters["label_excluded"] += 1
                    continue
                if q == p + 1:
                    ap = aq + DM[xq][x] * inv_v
                else:
                    c = ext[p + 1]
                    af = aq + DM[xq][c] * inv_v
                    if af < EW[c]: af = EW[c]
                    if af > L:
                        self.counters["label_excluded"] += 1
                        continue
                    al = af + D
                    if al < W: al = W
                    ap = al + DM[ext[q - 1]][x] * inv_v
                if ap < EW[x]: ap = EW[x]
                if ap > LW[x] or ap + DM[x][nxt] * inv_v > amax[q + 1]:
                    self.counters["label_excluded"] += 1
                    continue
                w = (gx - GAM[xq]) * (arr[q] - ax)
                moves.append((w if w > 0.0 else 0.0, ("swap", p, q), dd))
        return moves

    def _rcl(self, op, ctx, Nprime):
        if op == "add":
            moves = self._set_add(ctx, Nprime)
        elif op == "replace":
            moves = self._set_replace(ctx, Nprime)
        elif op == "two_opt":
            moves = self._set_two_opt(ctx)
        else:
            moves = self._set_swap(ctx)
        self.counters[f"set_{op}"] += len(moves)
        self.counters[f"sets_{op}"] += 1
        moves.sort(key=lambda m: (-m[0], m[1]))
        top = moves[:self.rcl_size]
        if self.draw == "roulette" and len(top) > 1:
            out = []
            rest = list(top)
            while rest:
                ws = [m[0] if m[0] > 0.0 else 0.0 for m in rest]
                if sum(ws) <= 0.0:
                    idx = random.randrange(len(rest))
                else:
                    idx = random.choices(range(len(rest)), weights=ws, k=1)[0]
                out.append(rest.pop(idx))
            top = out
        return top

    @staticmethod
    def _apply(route, key):
        if key[0] == "add":
            _, u, p = key; return route[:p] + [u] + route[p:]
        if key[0] == "replace":
            _, u, p = key; return route[:p] + [u] + route[p + 1:]
        if key[0] == "swap":
            _, p, q = key; r = list(route); r[p], r[q] = r[q], r[p]; return r
        _, p, q = key
        return route[:p] + route[p:q + 1][::-1] + route[q + 1:]

    # ---------------------------------------------------------------- shake
    def _enumeration(self, k):
        """The removals of a reference route with k targets: for cons = 1..c, the
        blocks of cons consecutive targets that sweep the route once, successive
        levels staggered by one position.  At least two targets are kept."""
        c = max(1, -(-k // self.sweep_cap_div))          # ceil(k / D)
        if self.sweep_cap_max:
            c = min(c, self.sweep_cap_max)
        c = min(c, max(1, k - 2))
        pairs = []
        for cons in range(1, c + 1):
            start = (cons - 1) % k
            removed = 0
            while removed < k:
                pairs.append((cons, 1 + start % k))
                start += cons; removed += cons
        return pairs

    @staticmethod
    def _remove_block(route, cons, post):
        """Route without the cons targets at positions post, post+1, ... (wrapping)."""
        k = len(route) - 1
        if k - cons < 2:
            return None
        idxs = {1 + ((post - 1 + h) % k) for h in range(cons)}
        return [route[i] for i in range(len(route)) if i not in idxs]

    # ------------------------------------------------------------------- run
    def run_rcl(self):
        random.seed(self.seed)
        G, depot = self.graph, self.depot
        forced = self.init_tour_kind or "R4"
        if forced == "R1":
            init_tour = build_R1(self.instance)
        elif forced == "R2":
            init_tour = build_R2(self.instance, rng=random.Random(self.init_seed))
        elif forced == "R3":
            init_tour = build_R3(self.instance, rng=random.Random(self.init_seed))
        else:
            init_tour = build_R4(self.instance)
        state = State.initial_state(self.instance, init_tour)
        state.parent = None
        self.cache[tuple(state.solver.tour_nodes)] = state.value
        self._record_best(state.value, state.solver.tour_nodes)
        print(f"[{self.name}] start ({forced}, rcl L={self.rcl_size}) init obj {state.value:.2f} "
              f"size {len(state.solver.tour_nodes) - 1} at {self.elapsed():.0f} s", flush=True)

        # A reference local optimum carries the enumeration of its removals: for
        # cons = 1..c the route is swept once in disjoint blocks of cons consecutive
        # targets (successive levels staggered by one position), then cons grows.
        # E(R) is the list of (cons, post) pairs; idx points at the next one.
        ref_state, ref_pairs, ref_idx, ref_stack = None, [], 0, []
        rhist = deque(maxlen=3)             # routes occupied since the last shake
        tol_rel = 1e-6
        it = 0
        while True:
            if self.remaining() <= 0:
                self.stop_reason = "budget"; break
            if self.idle_shakes >= self.max_idle_shakes:
                self.stop_reason = "idle_shakes"; break
            if self.idle >= self.max_iter:
                self.stop_reason = "max_iter"; break

            # ---------------- descent: VND over restricted candidate lists ----------------
            ctx = self._context(state)
            Nprime = set(G.nodes) - set(ctx["route"]) - {depot}
            rcls = {}
            oi = 0
            while oi < len(self.order):
                op = self.order[oi]
                if (op == "swap" and ctx["k"] < 2) or (op == "two_opt" and ctx["k"] < 2) \
                        or (op in ("add", "replace") and not Nprime):
                    oi += 1; continue
                if op not in rcls:
                    rcls[op] = self._rcl(op, ctx, Nprime)
                f, cap, route = ctx["f"], ctx["cap"], ctx["route"]
                moved = False
                tol = tol_rel * max(1.0, abs(f))
                for w, key, dd in rcls[op]:
                    if self.remaining() <= 0:
                        break
                    it += 1; self.iter_global += 1; self.idle += 1
                    self.counters[f"prop_{op}"] += 1
                    cand = self._apply(route, key)
                    tcand = tuple(cand)
                    if op in ("swap", "two_opt") and len(rhist) >= 2 and tcand == rhist[-2]:
                        self.counters["return_excluded"] += 1
                        continue
                    if self.cache.get(tcand, _MISS) is None:
                        self.counters["cache_excluded"] += 1
                        continue
                    if self.info_bound and op in ("add", "replace"):
                        ub = self._info_upper(cand)
                        if ub is not None and ub <= f:
                            self.counters["bound_pruned"] += 1
                            continue
                    self.counters[f"eval_{op}"] += 1
                    new_state, obj, verdict = self.evaluate(state, cand)
                    if new_state is None and obj is not None:
                        want = (obj > f) if op in ("add", "replace") else (obj > f - tol)
                        if want:
                            new_state, obj, verdict = self._resolve(state, cand)
                        else:
                            self.counters["cache_worse"] += 1; self.counters["worse"] += 1
                            continue
                    if new_state is None:
                        self.counters[f"reject_{op}"] += 1
                        continue
                    if op in ("add", "replace"):
                        accept = obj > f
                    elif obj > f + tol:
                        accept = True
                    elif abs(obj - f) <= tol:
                        _, _, _, cap_new = self._route_stats(new_state)
                        accept = cap_new < cap - 1e-9
                        if accept:
                            self.counters["lateral_accepted"] += 1
                    else:
                        accept = False
                    if accept:
                        new_state.parent = None
                        state = new_state
                        if op in ("add", "replace"):
                            rhist.clear()
                        rhist.append(tuple(state.solver.tour_nodes))
                        self.counters["accepted"] += 1; self.counters[f"acc_{op}"] += 1
                        if obj > self.best_obj:
                            self._record_best(obj, state.solver.tour_nodes)
                        moved = True
                        break
                    self.counters["worse"] += 1; self.counters[f"worse_{op}"] += 1
                if moved:
                    ctx = self._context(state)
                    Nprime = set(G.nodes) - set(ctx["route"]) - {depot}
                    rcls = {}
                    oi = 0
                else:
                    oi += 1
            if self.remaining() <= 0:
                self.stop_reason = "budget"; break

            # ---------------- local optimum of the four RCLs: reference and shake ----------------
            self.counters["lo_total"] += 1
            if ref_state is None or state.value > ref_state.value + 1e-9:
                if ref_state is not None:
                    ref_stack.append((ref_state, ref_pairs, ref_idx))
                ref_state = state
                ref_pairs = self._enumeration(len(state.solver.tour_nodes) - 1)
                ref_idx = 0
                self.counters["ref_improved"] += 1
            else:
                if state.solver.tour_nodes != ref_state.solver.tour_nodes:
                    state = ref_state
                    self.counters["shake_returns"] += 1
                if ref_idx >= len(ref_pairs):
                    self.counters["ref_exhausted"] += 1
                    if ref_stack:
                        ref_state, ref_pairs, ref_idx = ref_stack.pop()
                        state = ref_state
                        self.counters["ref_backtracks"] += 1
                    else:
                        self.stop_reason = "stack_exhausted"
                        break
            route = state.solver.tour_nodes
            # knapsack look-ahead over the next `lookahead` pairs of the enumeration;
            # the best is applied, the enumeration advances by one
            d_now = route_distance(route, G, depot)
            pick = None
            for j in range(ref_idx, min(ref_idx + max(1, self.lookahead), len(ref_pairs))):
                cons, post = ref_pairs[j]
                cand = self._remove_block(route, cons, post)
                if cand is None:
                    continue
                sc = self._knap(cand, max(0.0, d_now - route_distance(cand, G, depot)))
                if pick is None or sc > pick[0]:
                    pick = (sc, cand, j)
            ref_idx += 1
            self.counters["kicks"] += 1
            self.idle_shakes += 1
            rhist.clear()
            self._record_dyn(state.value, kick=1)
            if pick is None:
                continue
            self.counters["shake_removed"] += len(route) - len(pick[1])
            new_state, obj, verdict = self.evaluate(state, pick[1])
            if verdict == "cache_feasible":
                new_state, obj, verdict = self._resolve(state, pick[1])
            if new_state is not None:
                new_state.parent = None
                state = new_state
                if obj > self.best_obj:
                    self._record_best(obj, state.solver.tour_nodes)

        self.stop_wall = self.elapsed()
        print(f"\n[{self.name}] FINISHED  budget {self.budget:.0f} s  wall {self.stop_wall:.0f} s  "
              f"starts 1  stopped by {self.stop_reason} at {self.stop_wall:.0f} s", flush=True)
        print(f"[{self.name}] best obj {self.best_obj:.2f}  size {len(self.best_route) - 1}  "
              f"found at {self.best_wall:.1f} s (iter {self.best_iter})", flush=True)
        print(f"[{self.name}] shakes {self.counters['kicks']}  best at shake "
              f"{self.trace2[-1][2] if self.trace2 else 0}  socp {self.counters['socp_calls']}  "
              f"local optima {self.counters['lo_total']}", flush=True)
        print(f"[{self.name}] counters: {dict(self.counters)}", flush=True)
        print(f"[{self.name}] best route: {self.best_route}", flush=True)
        return self


def write_outputs(run, tag):
    safe = run.name.lower().replace(" ", "_").replace("(", "").replace(")", "")
    prefix = f"experiments/rcl_ils_{safe}_{tag}"
    with open(prefix + "_trace.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["wall_s", "iter", "shakes", "best_obj"])
        for row in run.trace2:
            w.writerow([f"{row[0]:.2f}", row[1], row[2], f"{row[3]:.4f}"])
    with open(prefix + "_summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["instance", "rcl", "cap_div", "lookahead", "max_idle_shakes", "best_obj",
                    "best_wall_s", "best_iter", "best_shake", "tour_size", "shakes", "socp_calls",
                    "accepted", "wall_s", "stop", "best_route"])
        w.writerow([run.name, run.rcl_size, run.sweep_cap_div, run.lookahead, run.max_idle_shakes,
                    f"{run.best_obj:.4f}", f"{run.best_wall:.1f}", run.best_iter,
                    run.trace2[-1][2] if run.trace2 else 0,
                    len(run.best_route) - 1, run.counters["kicks"], run.counters["socp_calls"],
                    run.counters["accepted"], f"{run.stop_wall:.1f}", run.stop_reason,
                    "-".join(str(n) for n in run.best_route)])
    print(f"Wrote {prefix}_trace.csv / _summary.csv", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", required=True)
    ap.add_argument("--budget", type=float, default=3600.0, help="wall-clock safeguard (s)")
    ap.add_argument("--rcl", type=int, default=10, help="L, moves per neighborhood evaluated by the SOCP")
    ap.add_argument("--cap-div", type=int, default=3, help="shake cap c = ceil(k / D)")
    ap.add_argument("--lookahead", type=int, default=6, help="pairs scored by the knapsack look-ahead")
    ap.add_argument("--max-idle-shakes", type=int, default=150,
                    help="S: stop after this many consecutive shakes without an improvement of R_best")
    ap.add_argument("--max-iter", type=int, default=None, help="secondary stop: iterations without an improvement")
    ap.add_argument("--order", default="add,replace,two_opt,swap")
    ap.add_argument("--draw", choices=["order", "roulette"], default="order")
    ap.add_argument("--no-info-bound", action="store_true")
    ap.add_argument("--init", choices=["R1", "R2", "R3", "R4"], default="R4")
    ap.add_argument("--init-seed", type=int, default=1)
    ap.add_argument("--seed", type=int, default=42, help="only used with --draw roulette")
    ap.add_argument("--fixed-speed", action="store_true")
    ap.add_argument("--no-loiter", action="store_true")
    ap.add_argument("--target", type=float, default=None)
    ap.add_argument("--stop-at-target", action="store_true")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--dynamics-out", default=None)
    args = ap.parse_args()

    match = [(n, p) for n, p in ALL_INSTANCES + EXPANSION_INSTANCES if n == args.instance]
    if not match:
        raise SystemExit(f"unknown instance: {args.instance}")
    name, path = match[0]
    tag = args.tag or f"L{args.rcl}_D{args.cap_div}_S{args.max_idle_shakes}"
    run = RCLILS(name, path, args.budget, rcl=args.rcl, cap_div=args.cap_div,
                 lookahead=args.lookahead, max_idle_shakes=args.max_idle_shakes,
                 order=tuple(x for x in args.order.split(",") if x), draw=args.draw,
                 info_bound=not args.no_info_bound, seed=args.seed,
                 init_tour_kind=args.init, init_seed=args.init_seed,
                 max_iter=args.max_iter, fixed_speed=args.fixed_speed, no_loiter=args.no_loiter,
                 target=args.target, stop_at_target=args.stop_at_target,
                 dynamics_out=args.dynamics_out).run_rcl()
    write_outputs(run, tag)


if __name__ == "__main__":
    main()
