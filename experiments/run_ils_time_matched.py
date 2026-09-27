"""
Time-matched multi-start ILS ("matheuristic, 3600 s protocol").

Motivation: the paper's Table `matheuristic-vs-exact` gave the ILS a fixed
iteration budget (100k iterations = 465-771 s on C-class) while the MISOCP
received a 3600 s cap. This runner matches wall-clock budgets so the two
methods are compared on equal footing, and strengthens the search where the
fixed-budget ILS demonstrably stalls (best found at iter ~35k on C101 (100),
zero improvement for the remaining 65k iterations).

Additions over run_ils_final_scored.py (scored sampling, segment kicks):

1. Wall-clock budget (default 3600 s) instead of an iteration budget. The
   timer covers everything: instance build, initial-tour construction, and
   all SOCP calls -- identical accounting to the MISOCP's 3600 s cap.
2. Multi-start: start 0 seeds from the R4 greedy construction (the paper's
   deterministic starter); every subsequent start draws a fresh R3 random
   tour. A start is abandoned when its best value has not improved for
   --restart-stall proposals; the global best is kept across starts.
3. Escalating perturbation: segment-removal length pool grows with the
   number of proposals since the last start-best improvement
   (tier 0: {2,3} -> tier 1: {4,6} -> tier 2: {8,12}), resetting on
   improvement. Deep C-class basins need cluster-scale kicks.
4. Relocation operator (Or-opt-1): remove one node and cascade-reinsert it
   at a different position. Unlike swap/2-opt, relocation preserves the
   relative order of all other nodes, so it composes the existing
   remove+add primitives into one atomic move. Probed SOCP feasibility on
   C-class (~3%) is on par with add/replace.
5. Energy lower-bound prefilter: any tour must consume at least
   (sum of arc distances) * P(v_mr)/v_mr, the energy of flying every arc
   at the maximum-range speed with no loitering. Proposals whose bound
   already exceeds E_max are rejected without an SOCP call.
6. Evaluation cache: route -> objective (or infeasible marker). Repeat
   proposals (frequent under tabu cycling) skip the SOCP.

Usage:
  python3 experiments/run_ils_time_matched.py --instance "C101 (100)" \
      --budget 3600 --target 11259.65
"""
import os, sys, random, csv, time, argparse, bisect

_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
os.chdir(_repo_root)

from collections import Counter, deque
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from uav_routing.local_search.initial_solution import (
    build_R1, build_R2, build_R3, build_R4,
)
from uav_routing.local_search.state import State
from uav_routing.local_search.proposal import route_to_nx

from run_ils_fb_cascade_demo import (
    make_instance, precompute_FB, compute_epsilon,
    cascade_feasible_add, cascade_feasible_replace,
)
from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES, SORTIE_TIME
from run_ils_final_scored import (
    add_fb_cascade_scored, replace_fb_cascade_scored,
    label_feasible_add, label_feasible_replace, compute_beta,
)

from run_ils_final_scored import taut_min_energy as _TAUT   # hot path: never re-imported per call
import heapq
import fast_sets as _fs      # O(1) move tests and propagated estimates (--fast-sets)
# Test-only: with ILS_LICENSE_GUARD=1 a Gurobi size-limited-license refusal counts as an
# infeasible route instead of aborting the run. Never set for reported results.
LICENSE_GUARD = bool(os.environ.get("ILS_LICENSE_GUARD"))

# A reordering accepted on a tie discards the four feasible sets, which
# resets the only condition that fires the shake. Two ways out: refuse the
# tie (NO_TIE), or allow it but force a shake after a run of them (TIE_CAP).
# How a reordering is accepted. "tie": an improvement, or an equal objective
# with a smaller capacity. "strict": an improvement only. "ratio": an
# improvement of f/cap, the quantity R4 uses, which prices information
# against the binding resource instead of treating freed capacity as free.
# Refuse a move back to a route the search has already occupied since the
# last shake. The acceptance rule is unchanged, so a reordering that frees
# capacity at a sliver of information is still taken; what is forbidden is
# returning, which is what turned such a pair into an endless alternation.
NO_REVISIT = bool(os.environ.get("ILS_NO_REVISIT"))
# The narrow form: refuse only a move back to the route the search has just
# left, and only for a reordering. That breaks the A-B-A-B alternation, which is
# the observed failure, without forbidding the legitimate returns a longer tabu
# also refuses. Insert and Replace are exempt: they change the visited set and
# are accepted only on a strict gain, so f rises along them and they cannot cycle.
# Number of A-B alternation hops tolerated before the repeat is refused: 1 refuses
# A-B-A, 2 allows A-B-A but refuses A-B-A-B. Only a genuine two-route repeat is
# refused; A-B-C-B and any longer cycle pass untouched. This is a count of hops,
# not a route length.
NO_RETURN = int(os.environ.get("ILS_NO_RETURN", 0))
# Break a sustained alternation. A single A-B-A is ordinary search and is left
# alone; only when the last 2k accepted routes alternate between exactly two
# routes is the search declared stuck, which fires the shake it was starving.
LOOP_BREAK = int(os.environ.get("ILS_LOOP_BREAK", 0))
# The shake fires only when the four move sets are exhausted, and any accepted
# move discards them. A run of accepted moves that never improves the best can
# therefore suppress the shake indefinitely. This fires one anyway after n
# iterations without a shake, so the trigger does not depend on exhaustion alone.
SHAKE_STALL = int(os.environ.get("ILS_SHAKE_STALL", 0))
# Count the moves whose score s(m) = dI / c is negative, per operator. Off by
# default: it is one pass over every set that is built.
SCORE_DIAG = bool(os.environ.get("ILS_SCORE_DIAG"))
# Choose the shake among the next m sweep candidates by the room it opens: the
# number of unvisited targets that gain at least one feasible insertion slot.
# The sweep still generates the candidates, so the enumeration is unchanged.
SHAKE_ROOM = int(os.environ.get("ILS_SHAKE_ROOM", 0))
# Choose the shake among the next m sweep candidates by what the removal buys:
# the freed distance is a budget, and the unvisited targets it makes insertable
# are bought greedily by information per detour. Counting the targets instead
# ignores that one far target consumes the budget two near ones could use.
SHAKE_KNAP = int(os.environ.get("ILS_SHAKE_KNAP", 0))
# Let the knapsack choose the removal size instead of the cap: at the sweep's
# current start, score every size by what it buys minus the information it
# destroys. The raw value grows with the freed distance, so only the net makes
# the size self-regulating; with it the cap ceil(k/D) is not needed.
KNAP_SIZE = bool(os.environ.get("ILS_KNAP_SIZE"))
# Skip the subproblem for an Insert or Replace that cannot be accepted. The
# information is linear in the arrival, so over the realized window of a visit it
# is largest at an endpoint; summing those maxima bounds f(R') from above, since
# it ignores both the coupling of the chain and the energy budget. Insert and
# Replace are accepted only on f(R') > f(R), so a bound at or below f(R) settles
# the move. Reorderings are exempt: the capacity clause can accept them anyway.
INFO_BOUND = os.environ.get("ILS_INFO_BOUND", "1") != "0"      # on by default
# With ILS_BOUND_END the pruned draw ends the iteration instead of reaching for
# another candidate, so a prune costs no subproblem at all.
INFO_BOUND_END = os.environ.get("ILS_BOUND_END", "1") != "0"   # on by default
# Diagnostic: solve every move the bound rejects and confirm it could not have
# been accepted. Slow, and only for checking the bound.
BOUND_AUDIT = bool(os.environ.get("ILS_BOUND_AUDIT"))
# Smallest removal the sweep may make. Removals of one or two targets improve the
# best in 0.9% and 1.3% of shakes against 5.5% for eight or more, so starting the
# escalation above them raises the yield per shake and costs nothing.
SWEEP_MIN = int(os.environ.get("ILS_SWEEP_MIN", 1))
# Insert scored by the information it adds, with no distance denominator and no
# shift: 87% of Insert scores are already positive, so the scaling only rescued
# a minority. Reorderings scored by the distance they save, since they almost
# never add information (100% of Swap and 87% of 2-opt scores are negative) and
# what they deliver is slack. Both are non-negative, so no shift is needed.
# Insert and Replace scored by the information change over the distance change,
# with no d0 floor and no shift. Inserting cannot shorten a route, so dd > 0 for
# Insert; Replace can shorten one, so the denominator is guarded. A non-positive
# score gets zero weight instead of being shifted onto a positive scale.
INSERT_RATIO = bool(os.environ.get("ILS_INSERT_RATIO"))
REORDER_W    = os.environ.get("ILS_REORDER_W", "")      # "" | "dsave" | "rank" | "best" | "exch"
# How a move is drawn from its set. "scaled" is the shifted roulette of (55).
# "rcl" draws uniformly from the K best moves by s, and "rank" weights by rank
# instead of value. Both use only the ordering of s, so neither needs the shift
# and neither depends on the origin of a score that is usually negative.
SELECT = os.environ.get("ILS_SELECT", "scaled")
RCL_K = int(os.environ.get("ILS_RCL_K", 5))
REORDER_ACCEPT = os.environ.get("ILS_REORDER_ACCEPT", "tie")
NO_TIE = bool(os.environ.get("ILS_NO_TIE")) or REORDER_ACCEPT == "strict"
TIE_CAP = int(os.environ.get("ILS_TIE_CAP", 0))

ILS_SEED = 42
_MISS = object()                    # 'absent from the evaluated-route store'
THETA = 10**12                      # the paper's shake fires on exhaustion, so this threshold never binds
SCALE_EPS = float(os.environ.get("ILS_SCALE_EPS", 0.01))
# diagnostic: draw Swap and 2-opt uniformly from their sets, as before change 2,
# instead of weighting them by the route-wide information change
REORDER_UNIFORM = bool(os.environ.get("ILS_REORDER_UNIFORM"))
# diagnostic: keep only the reorderings that shorten the route, as before
# change 4, instead of every leg-feasible position pair
REORDER_SHORTEN = bool(os.environ.get("ILS_REORDER_SHORTEN"))
                                    # paper (55): the least attractive move of a set keeps
                                    # a weight of eps times the range of the scores
MAX_ITER = 10**12                   # paper: iterations without an improvement of R* that
                                    # end the run; the default never binds, so a run is
                                    # ended by its wall-clock cap unless --max-iter is given
KICK_TIERS = ((2, 3), (4, 6), (8, 12))   # segment-length pools per tier
TIER_THRESHOLDS = (3000, 8000)      # proposals-since-improvement to escalate
TABU_BY_TIER = (10, 15, 25)
RESTART_STALL = 25000               # proposals without start-best improvement
MIN_RESTART_BUDGET = 120.0          # don't open a new start with less left
ELITE_RUIN_FRAC = 0.30              # nodes removed from global best on elite starts


def add_fb_cascade_uniform(route, complement_set, F, B, graph, depot,
                           v_max, T_max):
    """Ablation twin of add_fb_cascade_scored with the reward weighting removed.

    Identical edge shuffle and cascade gate; the only change is that the
    candidate node is drawn in uniform-random order instead of weighted by
    w_u = I_{e_u} + max(0, gamma_u * Delta_u). Isolates the value of the
    reward-weighted candidate sampling of Section 5.4.
    """
    eps = compute_epsilon(route, graph, depot, v_max)
    edges = list(enumerate(zip(route, route[1:] + [depot])))
    random.shuffle(edges)
    for p_idx, (i, j) in edges:
        p = p_idx + 1
        cands = list((F[i] & B[j]) & complement_set)
        if not cands:
            continue
        random.shuffle(cands)
        for u in cands:
            if cascade_feasible_add(route, p, u, graph, depot,
                                    v_max, T_max, eps):
                new_route = list(route)
                new_route.insert(p, u)
                return new_route
    return None


def replace_fb_cascade_uniform(route, complement_set, F, B, graph, depot,
                               v_max, T_max):
    """Uniform-sampling ablation twin of replace_fb_cascade_scored."""
    if len(route) < 2:
        return None
    eps = compute_epsilon(route, graph, depot, v_max)
    positions = list(range(1, len(route)))
    random.shuffle(positions)
    for p in positions:
        i = route[p - 1]
        j = route[p + 1] if p + 1 < len(route) else depot
        cands = list((F[i] & B[j]) & complement_set)
        if not cands:
            continue
        random.shuffle(cands)
        for u in cands:
            if cascade_feasible_replace(route, p, u, graph, depot,
                                        v_max, T_max, eps):
                new_route = list(route)
                new_route[p] = u
                return new_route
    return None


def relocate_fb_cascade(route, F, B, graph, depot, v_max, T_max,
                        max_sources=8):
    """Or-opt-1: remove one node, cascade-reinsert at a different position."""
    if len(route) < 3:
        return None
    positions = list(range(1, len(route)))
    random.shuffle(positions)
    for p in positions[:max_sources]:
        u = route[p]
        base_route = route[:p] + route[p + 1:]
        eps = compute_epsilon(base_route, graph, depot, v_max)
        edges = list(enumerate(zip(base_route, base_route[1:] + [depot])))
        random.shuffle(edges)
        for q_idx, (i, j) in edges:
            q = q_idx + 1
            if q == p:          # same slot: would recreate the input route
                continue
            if u not in F[i] or u not in B[j]:
                continue
            if cascade_feasible_add(base_route, q, u, graph, depot,
                                    v_max, T_max, eps):
                new_route = list(base_route)
                new_route.insert(q, u)
                return new_route
    return None


def cascade_feasible_route(route, graph, depot, v_max, T_max):
    """Full-route forward cascade: earliest arrivals at v_max with waiting.

    Gate for operators that rewire several edges at once (swap, 2-opt),
    where the incremental add/replace cascades do not apply. Returns True
    iff every window is reachable and the depot return fits the horizon.
    """
    prev = 0.0
    for q in range(1, len(route)):
        node = route[q]
        d = graph[route[q - 1]][node]['distance']
        e_q, l_q = graph.nodes[node]['time_window']
        prev = max(e_q, prev + d / v_max)
        if prev > l_q:
            return False
    return prev + graph[route[-1]][depot]['distance'] / v_max <= T_max


REORDER_STATS = {"cascade_rejected": 0}


def swap_fb_cascade(route, graph, depot, v_max, T_max, max_tries=40):
    """Exchange two visited nodes, gated by the full-route cascade."""
    n = len(route)
    if n < 3:
        return None
    for _ in range(max_tries):
        p = random.randint(1, n - 2)
        q = random.randint(p + 1, n - 1)
        new_route = list(route)
        new_route[p], new_route[q] = new_route[q], new_route[p]
        if cascade_feasible_route(new_route, graph, depot, v_max, T_max):
            return new_route
        REORDER_STATS["cascade_rejected"] += 1
    return None


def two_opt_fb_cascade(route, graph, depot, v_max, T_max, max_tries=40):
    """Reverse a segment route[p..q] (2-opt), gated by the full cascade."""
    n = len(route)
    if n < 4:
        return None
    for _ in range(max_tries):
        p = random.randint(1, n - 2)
        q = random.randint(p + 1, n - 1)
        new_route = route[:p] + route[p:q + 1][::-1] + route[q + 1:]
        if cascade_feasible_route(new_route, graph, depot, v_max, T_max):
            return new_route
        REORDER_STATS["cascade_rejected"] += 1
    return None


def shortening_reorders(route, graph, depot, op):
    """All Swap or 2-opt moves that shorten the route, sorted by the saving.
    Distance change in O(1) per pair; returns [(delta_d, p, q)] with delta_d < 0."""
    n = len(route)
    d = lambda a, b: graph[a][b]['distance']
    ext = route + [depot]
    moves = []
    for p in range(1, n - 1):
        for q in range(p + 1, n):
            a, rp, rq, b = ext[p - 1], ext[p], ext[q], ext[q + 1]
            if op == "two_opt" or q == p + 1:
                dd = d(a, rq) + d(rp, b) - d(a, rp) - d(rq, b)
            else:
                c, e = ext[p + 1], ext[q - 1]
                dd = (d(a, rq) + d(rq, c) + d(e, rp) + d(rp, b)
                      - d(a, rp) - d(rp, c) - d(e, rq) - d(rq, b))
            if dd < -1e-9:
                moves.append((dd, p, q))
    moves.sort()
    return moves


def apply_reorder(route, p, q, op):
    new_route = list(route)
    if op == "swap":
        new_route[p], new_route[q] = new_route[q], new_route[p]
    else:
        new_route = route[:p] + route[p:q + 1][::-1] + route[q + 1:]
    return new_route


def segment_remove_tiered(route, seg_lengths):
    candidate_ks = [k for k in seg_lengths if len(route) >= 1 + k + 1]
    if not candidate_ks:
        # fall back to the largest removable length >= 2
        max_k = len(route) - 2
        if max_k < 1:
            return None, None
        candidate_ks = [min(max_k, min(seg_lengths))]
    k = random.choice(candidate_ks)
    start = random.randint(1, len(route) - k)
    removed = route[start:start + k]
    return route[:start] + route[start + k:], removed


def _alternates(seq, hops):
    """True when seq is hops+2 states alternating between exactly two routes, so
    the candidate would extend an A-B alternation past `hops` hops: hops=1 catches
    A-B-A, hops=2 catches A-B-A-B. A window touching three or more routes never
    matches. `hops` is a count of alternation steps, not a route length."""
    return (len(seq) == hops + 2 and len(set(seq)) == 2
            and all(a != b for a, b in zip(seq, seq[1:])))


def sweep_remove(route, S, R, cap_div=3, cap_max=0):
    """Vansteenwegen-style shake ruin (C&OR 2009): remove R consecutive
    visits starting at sweep position S over the visited targets, with
    wraparound. S advances by the removal length after every shake and R
    escalates by one, capped at about a third of the current tour and
    reset to one by the caller on improvement. Coverage of the whole tour
    replaces the tabu list used by the random-segment kick.
    Returns (new_route, removed, S_next, R_next).
    """
    n = len(route) - 1          # visited targets, route[0] is the depot
    if n < 3:
        return None, None, S, R
    cap = max(2, -(-n // cap_div))   # ceil(n/cap_div)
    if cap_max:
        cap = min(cap, cap_max)
    lo = min(max(1, SWEEP_MIN), cap, n - 2)     # never below the floor, never above the cap
    r_eff = min(max(R, lo), cap, n - 2)         # always keep at least two targets
    start = (S - 1) % n
    idxs = {1 + ((start + k) % n) for k in range(r_eff)}
    removed = [route[i] for i in sorted(idxs)]
    new_route = [route[i] for i in range(len(route)) if i not in idxs]
    S_next = start + 1 + r_eff
    R_next = R + 1 if R + 1 <= cap else lo      # the escalation restarts at the floor
    return new_route, removed, S_next, R_next


def reward_guided_ruin(state, depot, seg_lengths):
    """Remove the k lowest reward-contribution nodes from the current tour.

    Uses the current SOCP solution's arrival times to score each visited node
    by its realized reward r_n = I_e + slope*(a_n - e_n), then drops the k
    weakest. Frees energy/time budget preferentially where it buys the least
    objective, so the scored-insertion repair can try higher-reward nodes --
    a targeted large-neighborhood ruin, in contrast to the random segment kick.
    Returns (new_route, removed) or (None, None) if the tour is too short.
    """
    sv = state.solver
    G = sv.graph
    tour = list(sv.tour_nodes)
    removable = [n for n in tour if n != depot]
    k = max(seg_lengths)
    if len(removable) < k + 1:
        return None, None
    rew = {}
    for n in removable:
        e_n = G.nodes[n]['time_window'][0]
        info = G.nodes[n]['info_at_lowest']
        slope = G.nodes[n]['info_slope']
        try:
            a_n = sv.arrival(n)
        except Exception:
            a_n = e_n
        rew[n] = info + slope * (a_n - e_n)
    drop = set(sorted(removable, key=lambda n: rew[n])[:k])
    new_route = [n for n in tour if n not in drop]
    return new_route, list(drop)


def route_distance(route, graph, depot):
    d = 0.0
    for a, b in zip(route, route[1:] + [depot]):
        d += graph[a][b]['distance']
    return d


def precompute_FB_leg(graph, depot, T_max, v_max):
    """Leg-feasibility sets (paper eq. fb-sets): F_i = {u : e_i + d_iu/v_max <= l_u},
    B_i = {u : e_u + d_ui/v_max <= l_i}; depot has e = 0, l = T_max."""
    nodes = list(graph.nodes)
    def e_of(n): return 0.0 if n == depot else graph.nodes[n]['time_window'][0]
    def l_of(n): return T_max if n == depot else graph.nodes[n]['time_window'][1]
    def d(a, b): return graph[a][b]['distance']
    F = {i: {u for u in nodes if u != i and e_of(i) + d(i, u) / v_max <= l_of(u)} for i in nodes}
    B = {j: {u for u in nodes if u != j and e_of(u) + d(u, j) / v_max <= l_of(j)} for j in nodes}
    return F, B


def chained_sets(route, F, B, depot):
    """Paper eq. leg-feas: FR[p] = F_{r_0} & ... & F_{r_{p-1}}, BR[p] = B_{r_p} & ... & B_{r_{k+1}},
    p = 1..k+1, with r_0 = r_{k+1} = depot and route = [depot, r_1, ..., r_k]."""
    k = len(route) - 1
    FR = [None] * (k + 2)
    BR = [None] * (k + 3)
    acc = set(F[depot]); FR[1] = acc
    for p in range(2, k + 2):
        acc = acc & F[route[p - 1]]; FR[p] = acc
    acc = set(B[depot]); BR[k + 1] = acc
    for p in range(k, 0, -1):
        acc = acc & B[route[p]]; BR[p] = acc
    return FR, BR


class TimedILS:
    """One time-budgeted multi-start ILS run on a single instance."""

    def __init__(self, name, path, budget, target=None, stop_at_target=False,
                 use_relocate=False,
                 use_lb_filter=True, restart_stall=RESTART_STALL,
                 seed_offset=0, escalate_kicks=False, tilt_p=0.0,
                 ruin_mode="segment", fixed_speed=False,
                 uniform_sampling=False, use_swap_2opt=True, no_loiter=False,
                 adaptive_ops=False, sweep_cap_div=6, sweep_cap_max=0, cand_mode="max",
                 local_search="phases", kappa_max=40, pair_weight="ratio",
                 paper_label=False, exhaust_shake=False, set_cache=False,
                 route_trace=None, move_trace=None,
                 rcl=5, shake_schedule="cap", shake_hold=2, restart_threshold=0,
                 init_tour_kind=None, init_seed=None, theta=None,
                 disabled_ops=(), no_fb=False, no_cascade=False,
                 no_cache=False, max_iter=None, no_shake=False, n_starts=1,
                 shake_return=False, shake_backtrack=False,
                 dynamics_out=None, weights_out=None,
                 fast_sets=False, reorder_rcl=0, max_idle_shakes=0, sweep_enum=False,
                 scaled_socp=False):
        self.name = name
        self.path = path
        self.budget = budget
        self.target = target
        self.stop_at_target = stop_at_target   # end the run once obj >= target (1 - 1e-4)
        self.use_relocate = use_relocate
        self.use_lb_filter = use_lb_filter
        self.restart_stall = restart_stall
        # Kick escalation ({2,3}->{4,6}->{8,12}) helps small instances escape
        # basins but wrecks large ones: an 8-12 node removal on a 30-node tour
        # is a near-restart a hill-climber cannot recover, so it plateaus low.
        # Default off => fixed {2,3} segment removal (the paper's proven kick).
        self.escalate_kicks = escalate_kicks
        self.tilt_p = tilt_p
        # Ruin operator for the kick: "segment" (random contiguous, paper
        # default) or "reward" (drop lowest-reward-contribution nodes).
        self.ruin_mode = ruin_mode
        # Sweep-shake escalation cap: ceil(n/cap_div), optionally clipped
        # at cap_max nodes (0 = no absolute clip).
        # Table 8 runs one start from a named construction heuristic.
        # Shake threshold theta: proposals without an accepted move. Scales
        # with the instance, since the share of infeasible proposals does.
        self._theta_arg = theta
        self.disabled_ops = set(disabled_ops)
        self.no_fb, self.no_cache = no_fb, no_cache
        if no_cascade:      # check 2 off: every candidate goes to the SOCP
            import run_ils_final_scored as _sc
            _sc.label_feasible_add = lambda *a, **k: True
            _sc.label_feasible_replace = lambda *a, **k: True
            globals()['cascade_feasible_route'] = lambda *a, **k: True
        self.init_tour_kind = init_tour_kind
        self.init_seed = init_seed
        self.sweep_cap_div = sweep_cap_div
        self.cand_mode = cand_mode
        self.local_search = local_search
        self.paper_label = paper_label
        self.exhaust_shake = exhaust_shake
        self.set_cache = set_cache
        self._set_best = {}
        self._mt_fh = None
        self._pending_ms = None
        self._mt_top = int(os.environ.get("MOVE_TRACE_TOP", 40))
        if move_trace:
            self._mt_fh = open(move_trace, "w", newline="")
            self._mt_w = csv.writer(self._mt_fh)
            # w_sum is the total weight of the whole set at the draw, so that a
            # reader can turn the listed weights into selection probabilities
            self._mt_w.writerow(["iter", "verdict", "op", "set_size", "chosen", "rank",
                                 "w_chosen", "w_sum", "cand", "weight"])
        self._rt_fh = None
        if route_trace:
            self._rt_fh = open(route_trace, "w", newline="")
            self._rt_w = csv.writer(self._rt_fh)
            self._rt_w.writerow(["event", "verdict", "iter", "wall_s", "obj", "best",
                             "T", "E", "cap", "amin", "amax",
                                 "route", "arrivals", "speeds", "loiter"])
        self.rcl = rcl
        self.shake_schedule = shake_schedule
        self.shake_hold = shake_hold
        self.restart_threshold = restart_threshold
        self.pair_weight = pair_weight
        self.kappa_max = kappa_max
        self.sweep_cap_max = sweep_cap_max
        # Ablation: uniform candidate order instead of reward-weighted (5.4).
        self.uniform_sampling = uniform_sampling
        # Intra-route reordering operators (swap, 2-opt), gated by the
        # full-route cascade. On co-monotone Solomon windows they rarely
        # fire, but they keep the operator set free of data-specific
        # exclusions; disable via --no-swap-2opt to reproduce the original
        # add/replace pool.
        self.use_swap_2opt = use_swap_2opt
        # Adaptive operator selection: draw operators by success-updated
        # weights instead of uniformly, so operators the instance rejects
        # (swap/2-opt on co-monotone windows) fade to a small floor
        # probability rather than being excluded by hand. Weights are
        # smoothed per proposal, w <- (1-rho)*w + rho*score, and persist
        # across restarts within a run.
        self.adaptive_ops = adaptive_ops
        self.op_weights = {}
        self.OP_RHO = 0.05          # smoothing rate
        self.OP_FLOOR = 0.05        # minimum weight: operators stay alive
        self.OP_SCORE = {"best": 3.0, "accepted": 1.5,
                         "rejected": 0.0, "none": 0.0}
        # Distinct seed stream per parallel worker so best-of-K samples
        # independent basins rather than replaying one trajectory.
        self.seed_offset = seed_offset
        self.t0 = time.time()

        self.instance, self.graph, self.drone = make_instance(path)
        self.instance.no_loiter = no_loiter      # read by State when it builds a Solver
        self.instance.socp_scaled = bool(scaled_socp)   # nondimensionalized subproblem (Solver reads it)
        self.theta = self._theta_arg if self._theta_arg is not None else THETA
        # Termination: iterations without an improvement of the best found
        # solution. The counter lives on the run, is reset by _record_best, and
        # ends the run (not just the current start) when it reaches the limit.
        self.max_iter = int(max_iter) if max_iter else int(os.environ.get("ILS_MAX_ITER", MAX_ITER))
        self.idle = 0
        # Multi-start local search without the shake (diagnostic, not in the
        # paper): each start runs to its first local optimum and stops.
        self.no_shake = no_shake
        # Shake the reference local optimum rather than whatever route the
        # search has drifted to, so that (post, cons) sweep one route.
        self.shake_return = shake_return
        # Section 4 design switches: O(1) move tests with propagated estimates (same sets,
        # same weights), a restricted candidate list for Swap and 2-opt, termination by
        # consecutive shakes without an improvement, and the explicit shake enumeration.
        self.fast_sets = fast_sets
        self.reorder_rcl = int(reorder_rcl)
        self.max_idle_shakes = int(max_idle_shakes)
        self.sweep_enum = sweep_enum
        self.scaled_socp = scaled_socp
        self.idle_shakes = 0
        # When the sweep of a reference comes back to a shake already applied
        # to it, its neighborhood is exhausted: fall back to the previous
        # local optimum and resume that one's sweep where it stopped.
        self.shake_backtrack = shake_backtrack
        self.n_starts = int(n_starts)
        self.lo_log = []                # (start, init kind, local optimum, wall at the stop)
        self.stop_reason = None
        self.stop_wall = None
        if fixed_speed:
            # Ablation: pin the speed envelope to the maximum-range speed
            # v_mr = (c2/c1)^(1/4), removing speed as a decision variable.
            # Loitering stays available (L = v_mr * t), so this isolates the
            # value of continuous speed optimization in the matheuristic.
            v_mr = (self.drone.c_2 / self.drone.c_1) ** 0.25
            self.drone.speed_min = v_mr
            self.drone.speed_max = v_mr
        self.depot = self.drone.base
        self.v_max = self.drone.speed_max
        self.T_max = self.instance.time_horizon
        self.E_max = self.instance.max_energy
        self.F, self.B = precompute_FB(self.graph, self.depot, self.T_max)
        if local_search == "paper":     # travel-aware leg sets of the paper (eq. fb-sets)
            self.F_leg, self.B_leg = precompute_FB_leg(self.graph, self.depot, self.T_max, self.v_max)
        if self.no_fb:      # check 1 off: every unscheduled target is a candidate
            allt = {n for n in self.graph.nodes if n != self.depot}
            self.F = {i: set(allt) for i in self.graph.nodes}
            self.B = {i: set(allt) for i in self.graph.nodes}
        # Distance floor d_0 of the move cost (53): one percent of the mean leg
        # length, so that the preference for a move that adds no distance does
        # not depend on the scale of the instance coordinates.
        _dsum = _dn = 0
        for _a, _b, _dd in self.graph.edges(data="distance"):
            if _dd:
                _dsum += float(_dd); _dn += 1
        self.d_floor = 0.01 * (_dsum / _dn) if _dn else 1.0
        # Flat distance matrix. The chain walk does tens of millions of these,
        # and a list of lists is 13x faster than the networkx adjacency lookup.
        _mx = max(self.graph.nodes)
        self.DM = [[0.0] * (_mx + 1) for _ in range(_mx + 1)]
        for _a, _b, _dd in self.graph.edges(data="distance"):
            if _dd is not None:
                self.DM[_a][_b] = float(_dd); self.DM[_b][_a] = float(_dd)
        self._nd = (_fs.NodeData(self.graph, self.depot, self.T_max, self.v_max, self.DM,
                                 self.F_leg, self.B_leg) if hasattr(self, 'F_leg') else None)
        if self.fast_sets and not (INSERT_RATIO and REORDER_W == "exch"):
            raise SystemExit("--fast-sets builds the weights of ILS_INSERT_RATIO=1 ILS_REORDER_W=exch")

        # Energy per meter is E_arc / L = P(v)/v with v = L/t. Since L >= d_ij,
        # every arc obeys E_arc >= d_ij * min_v P(v)/v, so the whole tour obeys
        # E_total >= (sum of arc distances) * min_v P(v)/v. We take that global
        # minimum as e_per_m, giving a valid energy lower bound that can never
        # false-reject a feasible route. Computed on a fine grid (robust to any
        # c_0) with a 0.1% safety margin below the true minimum.
        c0, c1, c2 = self.drone.c_0, self.drone.c_1, self.drone.c_2
        v_lo, v_hi = 1.0, 400.0
        grid_min = min(
            c0 / v + c1 * v ** 2 + c2 / v ** 2
            for v in (v_lo + (v_hi - v_lo) * k / 4000.0 for k in range(4001))
        )
        self.e_per_m = 0.999 * grid_min
        import run_ils_final_scored as _sc
        _sc.CAND_MODE = self.cand_mode
        _sc.E_MAX = self.E_max
        _sc.E_PER_M = self.e_per_m
        _sc.T_MAX = self.T_max
        _sc.PAIR_WEIGHT = self.pair_weight
        _sc.RCL = self.rcl

        self.cache = {}             # tuple(route) -> obj float or None
        self.counters = Counter()
        self.best_obj = -float('inf')
        self.best_route = None
        self.best_wall = None
        self.best_iter = None
        self.t_beat_target = None
        self.trace = []             # (wall_s, iter, best_obj)
        self.start_log = []         # (start_idx, kind, init_obj, best_obj)
        self.iter_global = 0
        # Per-step dynamics trace (current value, best value, kick markers)
        # for the perturbation-dynamics figure. Written incrementally so an
        # interrupted run keeps what it has.
        self.dynamics_out = dynamics_out
        self.weights_out = weights_out
        self._w_fh = None
        if weights_out:
            self._w_fh = open(weights_out, "w", newline="")
            self._w_w = csv.writer(self._w_fh)
            self._w_w.writerow(["iter", "wall_s", "start", "f_curr", "f_best",
                                "w_add", "w_replace", "w_swap", "w_two_opt",
                                "prop_add", "prop_replace", "prop_swap", "prop_two_opt",
                                "acc_add", "acc_replace", "acc_swap", "acc_two_opt",
                                "best_add", "best_replace", "best_swap", "best_two_opt"])
        self._dyn_fh = None
        self._dyn_w = None
        self._dyn_n = 0
        if dynamics_out:
            self._dyn_fh = open(dynamics_out, "w", newline="")
            self._dyn_w = csv.writer(self._dyn_fh)
            self._dyn_w.writerow(["iter", "wall_s", "f_curr", "f_best", "kick"])

    def _schedule(self, st, route):
        """Arrival at every visit and speed, length and loitering on every leg of a
        solved route, read from the subproblem's solution. Returns three strings
        aligned with the route: arrivals for r_1..r_k then the depot return, and
        per-leg speed and loitering for (r_0,r_1)..(r_k,r_0). Empty when the state
        carries no solution."""
        if st is None or st.solver is None or st.solver.solution is None:
            return "", "", ""
        try:
            td = st.solver.get_tour_data()
        except Exception:
            return "", "", ""
        cyc = list(route) + [self.depot]
        arr, spd, loi = [], [], []
        _cum = 0.0
        for a_, b_ in zip(cyc, cyc[1:]):
            _tt = td.times.get((a_, b_))
            _cum += 0.0 if _tt is None else _tt
            if b_ == self.depot:
                arr.append(f"{_cum:.1f}")      # the return; arrival_times[depot] is the departure
            else:
                a = td.arrival_times.get(b_)
                arr.append("" if a is None else f"{a:.1f}")
        for a, b in zip(cyc, cyc[1:]):
            tt = td.times.get((a, b)); L = td.lengths.get((a, b))
            if tt is None or L is None or tt <= 0:
                spd.append(""); loi.append("")
            else:
                spd.append(f"{L / tt:.2f}")
                loi.append(f"{max(0.0, L - self.graph[a][b]['distance']):.0f}")
        return "-".join(arr), "-".join(spd), "-".join(loi)

    def _room(self, route):
        """Unvisited targets with at least one feasible slot in `route`, by the
        O(1) slot test of (46). No solve: this is a time-window count only."""
        G, depot, T_max = self.graph, self.depot, self.T_max
        inv = 1.0 / self.v_max
        n = len(route)
        ew = lambda x: 0.0 if x == depot else float(G.nodes[x]["time_window"][0])
        lw = lambda x: T_max if x == depot else float(G.nodes[x]["time_window"][1])
        d = lambda x, y: 0.0 if x == y else G[x][y]["distance"]
        amin = [0.0] * (n + 1); p = 0.0
        for q in range(1, n):
            p = max(ew(route[q]), p + d(route[q - 1], route[q]) * inv); amin[q] = p
        amin[n] = p + d(route[-1], depot) * inv
        amax = [0.0] * (n + 1); amax[n] = T_max
        nxt = T_max - d(route[-1], depot) * inv
        for q in range(n - 1, 0, -1):
            amax[q] = min(lw(route[q]), nxt); nxt = amax[q] - d(route[q - 1], route[q]) * inv
        cnt = 0
        inside = set(route)
        for u in self.graph.nodes:
            if u in inside:
                continue
            eu, lu = ew(u), lw(u)
            for q in range(1, n + 1):
                prev = route[q - 1]; nx = route[q] if q < n else depot
                a = amin[q - 1] + d(prev, u) * inv
                if a < eu: a = eu
                if a <= lu and a + d(u, nx) * inv <= amax[q]:
                    cnt += 1; break
        return cnt

    def _info_upper(self, route):
        """Upper bound on f(route): every visit at the better end of its realized
        window, which ignores the coupling of the chain and the energy budget."""
        G, depot, T_max = self.graph, self.depot, self.T_max
        inv = 1.0 / self.v_max
        d = self.DM
        tw = G.nodes
        n = len(route)
        amin = [0.0] * n; p = 0.0
        for q in range(1, n):
            e_q = tw[route[q]]["time_window"][0]
            p = p + d[route[q - 1]][route[q]] * inv
            if p < e_q: p = e_q
            if p > tw[route[q]]["time_window"][1]:
                return None                      # infeasible: leave it to the caller
            amin[q] = p
        amax = [0.0] * n
        nxt = T_max - d[route[-1]][depot] * inv
        for q in range(n - 1, 0, -1):
            l_q = tw[route[q]]["time_window"][1]
            amax[q] = l_q if l_q < nxt else nxt
            if amax[q] < amin[q]:
                return None
            nxt = amax[q] - d[route[q - 1]][route[q]] * inv
        tot = 0.0
        for q in range(1, n):
            nq = route[q]; w = tw[nq]
            g = float(w.get("info_slope", 0.0)); b = float(w.get("info_at_lowest", 0.0))
            e = w["time_window"][0]
            a = g * (amin[q] - e) + b; c2 = g * (amax[q] - e) + b
            tot += a if a > c2 else c2
        return tot

    def _knap(self, route, budget):
        """Information purchasable with `budget` of freed distance.

        a^min and a^max both increase along the route, so the positions where a
        target u can be slotted form a contiguous slice: it starts at the first q
        with a^max_q >= e_u (binary search) and ends when a^min_{q-1} > l_u. Only
        that slice is scanned, and within it the leg sets F prune a position
        before any arithmetic. Cost is then O(k + sum_u |slice_u|) rather than
        O(|U| k).
        """
        G, depot, T_max = self.graph, self.depot, self.T_max
        F = self.F
        inv = 1.0 / self.v_max
        n = len(route)
        tw = G.nodes
        d = self.DM
        amin = [0.0] * (n + 1); p = 0.0
        for q in range(1, n):
            e_q = tw[route[q]]["time_window"][0]
            p = p + d[route[q - 1]][route[q]] * inv
            if p < e_q: p = e_q
            amin[q] = p
        amin[n] = p + d[route[-1]][depot] * inv
        amax = [0.0] * (n + 1); amax[n] = T_max
        nxt = T_max - d[route[-1]][depot] * inv
        for q in range(n - 1, 0, -1):
            l_q = tw[route[q]]["time_window"][1]
            amax[q] = l_q if l_q < nxt else nxt
            nxt = amax[q] - d[route[q - 1]][route[q]] * inv
        amax[0] = nxt

        items = []
        inside = set(route)
        amax_slice = amax[1:n + 1]            # increasing, for the binary search
        for u in G.nodes:
            if u in inside or u == depot:
                continue
            w = tw[u]["time_window"]; eu = w[0]; lu = w[1]
            if lu < amin[0] or eu > T_max:
                continue
            Fu = F[u]; du = d[u]
            q0 = bisect.bisect_left(amax_slice, eu) + 1      # first position that can be late enough
            best_dd = None; best_i = 0.0
            for q in range(q0, n + 1):
                if amin[q - 1] > lu:
                    break                                    # a^min only grows: no later slot fits
                prev = route[q - 1]
                if u not in F[prev]:
                    continue                                 # leg prev->u infeasible: no arithmetic
                nx = route[q] if q < n else depot
                if nx not in Fu:
                    continue
                a = amin[q - 1] + d[prev][u] * inv
                if a < eu: a = eu
                if a > lu: continue
                t_un = du[nx] * inv
                if a + t_un > amax[q]: continue
                dd = d[prev][u] + du[nx] - d[prev][nx]
                if best_dd is None or dd < best_dd:
                    hi = amax[q] - t_un
                    if hi > lu: hi = lu
                    g = float(tw[u].get("info_slope", 0.0)); b = float(tw[u].get("info_at_lowest", 0.0))
                    ia = g * (a - eu) + b; ib = g * (hi - eu) + b
                    best_dd = dd; best_i = ia if ia > ib else ib
            if best_dd is not None:
                items.append((best_dd, best_i))
        items.sort(key=lambda x: -(x[1] / (x[0] if x[0] > 1e-6 else 1e-6)))
        tot = 0.0; used = 0.0
        for dd, iu in items:
            if used + dd <= budget:
                used += dd; tot += iu
        return tot

    def _window_bounds(self, route):
        """Realized window of every visit: a^min forward and a^max backward at
        v_max, the bounds of Equation (45). Returned as two strings aligned with
        the arrivals, so the viewer can draw the slack the schedule had."""
        n = len(route)
        if n < 2:
            return "", ""
        inv_v = 1.0 / self.v_max
        G, depot, T_max = self.graph, self.depot, self.T_max
        ew = lambda x: 0.0 if x == depot else float(G.nodes[x]["time_window"][0])
        lw = lambda x: T_max if x == depot else float(G.nodes[x]["time_window"][1])
        d = lambda x, y: G[x][y]["distance"]
        amin = [0.0] * n; prev = 0.0
        for q in range(1, n):
            prev = max(ew(route[q]), prev + d(route[q - 1], route[q]) * inv_v)
            amin[q] = prev
        amax = [0.0] * n
        nxt = T_max - d(route[-1], depot) * inv_v
        for q in range(n - 1, 0, -1):
            hi = min(lw(route[q]), nxt)
            amax[q] = hi
            nxt = hi - d(route[q - 1], route[q]) * inv_v
        amin_depot = amin[n - 1] + d(route[-1], depot) * inv_v
        return ("-".join(f"{x:.1f}" for x in list(amin[1:]) + [amin_depot]),
                "-".join(f"{x:.1f}" for x in list(amax[1:]) + [float(T_max)]))

    def _record_route(self, event, route, obj, verdict="accepted", state=None):
        if self._rt_fh is None:
            return
        arr, spd, loi = self._schedule(state, route)
        # the solver's own totals, the quantities the acceptance rule compares;
        # recomputing them from the leg data would differ by the cone slack
        if state is not None and state.solver is not None and state.solver.solution is not None:
            _, _T, _E, _cap = self._route_stats(state)
            Ts, Es, caps = f"{_T:.2f}", f"{_E:.1f}", f"{_cap:.6f}"
        else:
            Ts = Es = caps = ""
        wlo, whi = self._window_bounds(route)
        self._rt_w.writerow([event, verdict, self.iter_global, f"{self.elapsed():.2f}",
                             "" if obj is None else f"{obj:.4f}", f"{self.best_obj:.4f}",
                             Ts, Es, caps, wlo, whi,
                             "-".join(str(n) for n in route), arr, spd, loi])
        if self._mt_fh is not None and self._pending_ms is not None:
            o_, n_, ch_, rk_, wc_, ws_, cands = self._pending_ms
            for c_, w_ in cands:
                self._mt_w.writerow([self.iter_global, verdict, o_, n_, ch_, rk_,
                                     f"{wc_:.6g}", f"{ws_:.6g}", c_, f"{w_:.6g}"])
            self._pending_ms = None

    def _record_dyn(self, curr, kick=0):
        if self._dyn_w is None:
            return
        self._dyn_w.writerow([self.iter_global, f"{self.elapsed():.2f}",
                              f"{curr:.4f}", f"{self.best_obj:.4f}", kick])
        self._dyn_n += 1
        if self._dyn_n % 5000 == 0:
            self._dyn_fh.flush()

    def elapsed(self):
        return time.time() - self.t0

    def remaining(self):
        return self.budget - self.elapsed()

    # ---------------- proposal machinery ----------------

    def propose(self, state, tabu_set):
        """Return (new_route or None, op_name)."""
        route = state.solver.tour_nodes
        complement = (set(self.graph.nodes) - set(route)) - tabu_set
        ops = []
        if complement:
            ops += ["add", "replace"]
        if self.use_relocate and len(route) >= 4:
            ops.append("relocate")
        if self.use_swap_2opt:
            if len(route) >= 3:
                ops.append("swap")
            if len(route) >= 4:
                ops.append("two_opt")
        # Operator ablation: drop named operators from the pool.
        ops = [o for o in ops if o not in self.disabled_ops]
        if not ops:
            return None, "no_op"
        if self.adaptive_ops:
            weights = [self.op_weights.setdefault(o, 1.0) for o in ops]
            op = random.choices(ops, weights=weights, k=1)[0]
        else:
            op = random.choice(ops)
        add_fn = (add_fb_cascade_uniform if self.uniform_sampling
                  else add_fb_cascade_scored)
        rep_fn = (replace_fb_cascade_uniform if self.uniform_sampling
                  else replace_fb_cascade_scored)
        if self.cand_mode == "pair":
            import run_ils_final_scored as _sc
            add_fn, rep_fn = _sc.add_pair_roulette, _sc.replace_pair_roulette
            _sc.ROUTE_E_FLOOR = route_distance(route, self.graph, self.depot) * self.e_per_m
            sv = state.solver; G = sv.graph; rew = {}
            for n in route:
                if n == self.depot:
                    continue
                try:
                    a_n = sv.arrival(n)
                except Exception:
                    continue
                nd = G.nodes[n]
                rew[n] = float(nd.get('info_at_lowest', 1.0)) + float(nd.get('info_slope', 0.0)) * (a_n - nd['time_window'][0])
            _sc.CURRENT_REWARDS = rew
        if op == "add":
            new_route = add_fn(
                route, complement, self.F, self.B, self.graph, self.depot,
                self.v_max, self.T_max)
        elif op == "replace":
            new_route = rep_fn(
                route, complement, self.F, self.B, self.graph, self.depot,
                self.v_max, self.T_max)
        elif op == "swap":
            new_route = swap_fb_cascade(
                route, self.graph, self.depot, self.v_max, self.T_max)
        elif op == "two_opt":
            new_route = two_opt_fb_cascade(
                route, self.graph, self.depot, self.v_max, self.T_max)
        else:
            new_route = relocate_fb_cascade(
                route, self.F, self.B, self.graph, self.depot,
                self.v_max, self.T_max)
        return new_route, op

    def feedback_op(self, op, outcome):
        """Update an operator's weight from its proposal outcome."""
        if not self.adaptive_ops or op == "no_op":
            return
        if outcome == "best":
            self.counters[f"best_{op}"] += 1
        w = self.op_weights.setdefault(op, 1.0)
        w = (1.0 - self.OP_RHO) * w + self.OP_RHO * self.OP_SCORE[outcome]
        self.op_weights[op] = max(self.OP_FLOOR, w)

    def evaluate(self, state, new_route):
        """SOCP-evaluate new_route (with cache + energy LB prefilter).

        Returns (new_state or None, obj or None, verdict) where verdict in
        {"feasible", "cache_feasible", "infeasible", "cache_infeasible",
         "lb_reject"}.
        """
        key = tuple(new_route)
        if key in self.cache and not self.no_cache:
            obj = self.cache[key]
            if obj is None:
                self.counters["cache_infeas"] += 1
                return None, None, "cache_infeasible"
            self.counters["cache_feas"] += 1
            return None, obj, "cache_feasible"
        d_total = route_distance(new_route, self.graph, self.depot)
        if self.use_lb_filter:
            if d_total * self.e_per_m > self.E_max:
                self.counters["lb_reject"] += 1
                self.cache[key] = None
                return None, None, "lb_reject"
        # Exact energy feasibility by the taut string: the minimum energy of
        # the route over all schedules within the windows. Above the budget
        # the route is infeasible and no solve is needed.
        E_ts, smin = _TAUT(new_route, self.graph, self.depot, self.drone, self.T_max, self.v_max)
        if smin >= 1.0 / self.v_max - 1e-12 and E_ts > self.E_max * (1 + 1e-9):
            self.counters["ts_reject"] += 1
            self.cache[key] = None
            return None, None, "infeasible"
        _t0 = time.perf_counter()
        try:
            new_state = state.flip(route_to_nx(new_route))
        except Exception as exc:
            if LICENSE_GUARD and "size-limited" in str(exc):
                self.counters["license_reject"] += 1
                return None, None, "infeasible"
            raise
        self.counters["socp_us"] += int((time.perf_counter() - _t0) * 1e6)
        self.counters["socp_calls"] += 1
        feas = not (new_state.solver is None or new_state.solver.solution is None)
        if os.environ.get("ILS_TS_VALIDATE"):
            import run_ils_final_scored as _sc, time as _t
            _t0 = _t.perf_counter()
            E_ts, smin = _sc.taut_min_energy(new_route, self.graph, self.depot, self.drone, self.T_max, self.v_max)
            _dt = _t.perf_counter() - _t0
            with open(os.environ["ILS_TS_VALIDATE"], "a") as fh:
                fh.write(",".join([getattr(self, "_diag_phase", "?"), str(int(feas)), f"{E_ts:.1f}",
                                   f"{self._energy(new_state):.1f}" if feas else "", f"{self.E_max:.1f}",
                                   f"{smin:.6f}", f"{1.0 / self.v_max:.6f}", f"{_dt * 1e6:.0f}",
                                   "-".join(str(x) for x in new_route)]) + "\n")
        if os.environ.get("ILS_DIAG"):
            import run_ils_final_scored as _sc
            old_route = state.solver.tour_nodes
            with open(os.environ["ILS_DIAG"], "a") as fh:
                fh.write(",".join(str(x) for x in [
                    getattr(self, "_diag_phase", "?"), int(feas),
                    len(old_route) - 1, len(new_route) - 1,
                    f"{route_distance(old_route, self.graph, self.depot):.1f}", f"{d_total:.1f}",
                    f"{d_total * self.e_per_m:.0f}",
                    f"{_sc.energy_lb_labels(new_route, self.graph, self.depot, self.v_max, self.T_max, self.drone):.0f}",
                    f"{self.E_max:.0f}", f"{state.value:.2f}",
                    f"{new_state.value:.2f}" if feas else "",
                    f"{self._energy(state):.0f}", f"{self._energy(new_state):.0f}" if feas else ""]) + "\n")
        if not feas:
            if smin >= 1.0 / self.v_max - 1e-12 and E_ts <= self.E_max:
                # The taut string certifies that a feasible schedule exists, so
                # the solver failed on this route rather than proving it
                # infeasible. Recording it as infeasible would bar it from the
                # search for the rest of the run, so the store is left alone.
                self.counters["socp_numeric_infeasible"] += 1
                return None, None, "infeasible"
            self.cache[key] = None
            self.counters["socp_infeas"] += 1
            return None, None, "infeasible"
        obj = new_state.value
        self.cache[key] = obj
        return new_state, obj, "feasible"

    def _record_best(self, obj, route):
        self.best_obj = obj
        self.best_route = list(route)
        self.idle = 0                       # an improvement restarts the stopping counter
        self.idle_shakes = 0
        self.best_wall = self.elapsed()
        self.best_iter = self.iter_global
        self.trace.append((self.best_wall, self.iter_global, obj))
        if (self.target is not None and self.t_beat_target is None
                and obj > self.target):
            self.t_beat_target = self.best_wall
            print(f"[{self.name}] *** surpassed MISOCP incumbent "
                  f"{self.target:.2f} at {self.best_wall:.0f} s "
                  f"(obj {obj:.2f}) ***", flush=True)
        if (self.stop_at_target and self.target is not None
                and obj >= self.target * (1 - 1e-4)):
            print(f"[{self.name}] *** reached the proven optimum "
                  f"{self.target:.2f} at {self.best_wall:.0f} s, stopping ***", flush=True)
            self.budget = self.elapsed()          # remaining() <= 0 ends the run

    # ---------------- one start ----------------

    def _elite_init_tour(self):
        """Ruin the global best route: drop a random ~30% of its targets.

        Returns (tour, dropped) so the caller can tabu the dropped nodes,
        forcing the reconstruction to explore a different completion.
        """
        targets = [n for n in self.best_route if n != self.depot]
        k = max(2, int(round(ELITE_RUIN_FRAC * len(targets))))
        drop = set(random.sample(targets, min(k, len(targets) - 2)))
        kept = [n for n in self.best_route if n == self.depot or n not in drop]
        return route_to_nx(kept), drop

    def run_start(self, start_idx):
        """Run one ILS start until stall or budget exhaustion."""
        base_seed = ILS_SEED + 100000 * self.seed_offset
        r3_seed = 1000 + 100000 * self.seed_offset + start_idx
        random.seed(base_seed + start_idx)
        # Worker 0 start 0: R4 greedy (the paper's deterministic canonical
        # seed). Every other (worker, start) that is not an elite restart
        # draws a fresh R3 with a worker-unique seed, so best-of-K samples
        # independent basins. Even starts >=2 ruin the worker's global best.
        forced = self.init_tour_kind      # single start from a named heuristic
        r4_start = (start_idx == 0 and self.seed_offset == 0)
        elite = (start_idx >= 2 and start_idx % 2 == 0
                 and self.best_route is not None
                 and len(self.best_route) >= 9)
        if forced:
            kind = forced if forced != "R3" else f"R3(seed={self.init_seed})"
        elif r4_start:
            kind = "R4"
        elif elite:
            kind = "elite(ruin 30%)"
        else:
            kind = f"R3(seed={r3_seed})"
        init_tabu = {}
        try:
            if forced == "R1":
                init_tour = build_R1(self.instance)
            elif forced == "R2":
                init_tour = build_R2(self.instance,
                                     rng=random.Random(self.init_seed))
            elif forced == "R3":
                init_tour = build_R3(self.instance,
                                     rng=random.Random(self.init_seed))
            elif forced == "R4" or r4_start:
                init_tour = build_R4(self.instance)
            elif elite:
                init_tour, dropped = self._elite_init_tour()
                init_tabu = {n: self.iter_global + 60 for n in dropped}
            else:
                init_tour = build_R3(self.instance,
                                     rng=random.Random(r3_seed))
            state = State.initial_state(self.instance, init_tour)
        except Exception as exc:
            print(f"[{self.name}] start {start_idx} ({kind}) failed to "
                  f"initialize: {exc}", flush=True)
            return
        state.parent = None
        init_obj = state.value
        start_best = init_obj
        self.cache[tuple(state.solver.tour_nodes)] = init_obj
        if init_obj > self.best_obj:
            self._record_best(init_obj, state.solver.tour_nodes)
        print(f"[{self.name}] start {start_idx} ({kind}) init obj "
              f"{init_obj:.2f} size {len(state.solver.tour_nodes) - 1} "
              f"at {self.elapsed():.0f} s", flush=True)

        tabu = dict(init_tabu)
        stagnation = 0
        since_improve = 0
        it = 0
        self.sweep_S, self.sweep_R = 1, 1   # per-climb shake state
        while self.remaining() > 0:
            it += 1
            self.iter_global += 1
            i = self.iter_global
            tabu = {n: e for n, e in tabu.items() if e > i}
            tier = (sum(since_improve >= th for th in TIER_THRESHOLDS)
                    if self.escalate_kicks else 0)

            if stagnation >= self.theta:
                # ---- kick (always accepted if SOCP-feasible) ----
                route = state.solver.tour_nodes
                if self.ruin_mode == "reward":
                    new_route, removed = reward_guided_ruin(
                        state, self.depot, KICK_TIERS[tier])
                elif self.ruin_mode == "sweep":
                    new_route, removed, self.sweep_S, self.sweep_R = \
                        sweep_remove(route, self.sweep_S, self.sweep_R,
                                     self.sweep_cap_div, self.sweep_cap_max)
                else:
                    new_route, removed = segment_remove_tiered(
                        route, KICK_TIERS[tier])
                stagnation = 0
                self.counters["kicks"] += 1
                self._record_dyn(state.value, kick=1)
                if new_route is not None:
                    new_state, obj, verdict = self.evaluate(state, new_route)
                    if verdict == "cache_feasible":
                        new_state, obj, verdict = self._resolve(state, new_route)
                    if new_state is not None:
                        new_state.parent = None
                        state = new_state
                        if self.ruin_mode != "sweep":
                            tenure = TABU_BY_TIER[tier]
                            tabu.update({n: i + tenure for n in removed})
                        # A kick can raise the objective (dropping a low-reward
                        # block lets downstream nodes retime). Record it as best
                        # and reset the stall counters, exactly like a normal
                        # accepted move, so real progress is not thrown away.
                        if obj > start_best:
                            start_best = obj
                            since_improve = 0
                            self.sweep_R = 1
                        if obj > self.best_obj:
                            self._record_best(obj, state.solver.tour_nodes)
                continue

            new_route, op = self.propose(state, set(tabu.keys()))
            self.counters[f"prop_{op}"] += 1
            if new_route is None:
                self.counters["saturated"] += 1
                self.feedback_op(op, "none")
                since_improve += 1
                stagnation += 1          # every proposal without an accepted move counts
                continue
            new_state, obj, verdict = self.evaluate(state, new_route)
            if verdict in ("infeasible", "cache_infeasible", "lb_reject"):
                self.feedback_op(op, "rejected")
                since_improve += 1
                stagnation += 1          # NoImpr as in Gunawan et al.: any non-accepted proposal
                continue
            if obj > state.value:          # strict improvement only
                self.feedback_op(op, "best" if obj > self.best_obj
                                 else "accepted")
                self.counters[f"acc_{op}"] += 1
                if obj > state.value:
                    self.counters[f"impr_{op}"] += 1
                if verdict == "cache_feasible":
                    new_state, obj, verdict = self._resolve(state, new_route)
                    if new_state is None:
                        since_improve += 1
                        continue
                new_state.parent = None
                state = new_state
                stagnation = 0
                self.counters["accepted"] += 1
                if obj > start_best:
                    start_best = obj
                    since_improve = 0
                    self.sweep_R = 1
                else:
                    since_improve += 1
                if obj > self.best_obj:
                    self._record_best(obj, state.solver.tour_nodes)
            else:
                self.feedback_op(op, "rejected")
                stagnation += 1
                since_improve += 1
                self.counters["worse"] += 1
                # Tilt: occasionally accept a worse move to walk laterally off
                # a plateau (SA-lite). Stagnation still advances so kicks fire.
                if self.tilt_p > 0.0 and random.random() < self.tilt_p:
                    if verdict == "cache_feasible":
                        new_state, obj, verdict = self._resolve(state, new_route)
                        if new_state is None:
                            continue
                    new_state.parent = None
                    state = new_state
                    self.counters["tilt_accept"] += 1

            self._record_dyn(state.value)
            if self._w_fh is not None and it % 200 == 0:
                ops4 = ("add", "replace", "swap", "two_opt")
                self._w_w.writerow(
                    [it, round(self.elapsed(), 2), start_idx,
                     round(state.value, 2), round(self.best_obj, 2)]
                    + [round(self.op_weights.get(o, 1.0), 4) for o in ops4]
                    + [self.counters[f"prop_{o}"] for o in ops4]
                    + [self.counters[f"acc_{o}"] for o in ops4]
                    + [self.counters[f"best_{o}"] for o in ops4])
                self._w_fh.flush()

            if since_improve >= self.restart_stall:
                if self.remaining() > MIN_RESTART_BUDGET:
                    break   # stall: abandon this start
                since_improve = 0   # budget tail: keep working in place

        self.start_log.append((start_idx, kind, init_obj, start_best))
        print(f"[{self.name}] start {start_idx} ({kind}) done: best "
              f"{start_best:.2f} after {it} proposals "
              f"({self.elapsed():.0f} s elapsed)", flush=True)

    def _resolve(self, state, new_route):
        """Re-solve a cache-hit route to materialize a State object."""
        _t0 = time.perf_counter()
        try:
            new_state = state.flip(route_to_nx(new_route))
        except Exception as exc:
            if LICENSE_GUARD and "size-limited" in str(exc):
                self.counters["license_reject"] += 1
                return None, None, "infeasible"
            raise
        self.counters["socp_us"] += int((time.perf_counter() - _t0) * 1e6)
        self.counters["socp_calls"] += 1
        if new_state.solver is None or new_state.solver.solution is None:
            return None, None, "infeasible"
        return new_state, new_state.value, "feasible"

    # ---------------- driver ----------------

    # ---------------- phased local search (third option) ----------------

    def _cur_rewards(self, state):
        sv = state.solver; G = sv.graph; rew = {}
        for n in sv.tour_nodes:
            if n == self.depot:
                continue
            try:
                a_n = sv.arrival(n)
            except Exception:
                continue
            nd = G.nodes[n]
            rew[n] = float(nd.get('info_at_lowest', 1.0)) + float(nd.get('info_slope', 0.0)) * (a_n - nd['time_window'][0])
        return rew

    def _energy(self, state):
        return state.solver.get_tour_data().total_energy

    def _accept_state(self, state, new_state, obj, route):
        new_state.parent = None
        if obj > self.best_obj:
            self._record_best(obj, route)
        return new_state

    def run_start_phased(self, start_idx):
        """Local search in phases, each run to its own local optimum:
        1 Insert (accept if f increases), 2 Replace (accept if f increases),
        3 Swap and 2-opt (accept if f does not decrease and the energy
        decreases). A full cycle without an accepted move is a local optimum
        and triggers the shake. No stagnation counter."""
        import run_ils_final_scored as _sc
        random.seed(ILS_SEED + 100000 * self.seed_offset + start_idx)
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
        if state.value > self.best_obj:
            self._record_best(state.value, state.solver.tour_nodes)
        print(f"[{self.name}] start {start_idx} ({forced}, phased) init obj "
              f"{state.value:.2f} size {len(state.solver.tour_nodes) - 1} at {self.elapsed():.0f} s", flush=True)
        self.sweep_S, self.sweep_R = 1, 1
        self._lo_best = -float("inf"); self._noimpr = 0; self._hold = 0
        f_tol = 1e-7

        def insert_phase(state):
            """Draw from the feasible set until it is exhausted. Returns (state, accepted_any)."""
            tried = set(); any_acc = False
            while self.remaining() > 0:
                route = state.solver.tour_nodes
                complement = set(self.graph.nodes) - set(route)
                if not complement:
                    break
                e_floor = route_distance(route, self.graph, self.depot) * self.e_per_m
                F = _sc.feasible_insert_pairs(route, complement, self.graph, self.depot, self.v_max, self.T_max, e_floor)
                d = _sc.draw_pair(F, tried)
                if d is None:
                    break
                u, p = d
                new_route = list(route); new_route.insert(p, u)
                self.iter_global += 1; self.counters["prop_add"] += 1
                new_state, obj, verdict = self.evaluate(state, new_route)
                if verdict == "cache_feasible" and obj > state.value:
                    new_state, obj, verdict = self._resolve(state, new_route)
                if new_state is not None and obj > state.value:
                    self.counters["acc_add"] += 1; self.counters["accepted"] += 1
                    state = self._accept_state(state, new_state, obj, new_route)
                    tried = set(); any_acc = True
                    continue
                tried.add((u, p))
                self.counters["ins_" + ("worse" if (new_state is not None or verdict == "cache_feasible") else verdict)] += 1
            return state, any_acc

        def replace_phase(state):
            tried = set()
            while self.remaining() > 0:
                route = state.solver.tour_nodes
                complement = set(self.graph.nodes) - set(route)
                if not complement or len(route) < 2:
                    break
                e_floor = route_distance(route, self.graph, self.depot) * self.e_per_m
                F = _sc.feasible_replace_pairs(route, complement, self.graph, self.depot, self.v_max, self.T_max, e_floor, self._cur_rewards(state))
                d = _sc.draw_pair(F, tried)
                if d is None:
                    break
                u, p = d
                new_route = list(route); new_route[p] = u
                self.iter_global += 1; self.counters["prop_replace"] += 1
                new_state, obj, verdict = self.evaluate(state, new_route)
                if verdict == "cache_feasible" and obj > state.value:
                    new_state, obj, verdict = self._resolve(state, new_route)
                if new_state is not None and obj > state.value:
                    self.counters["acc_replace"] += 1; self.counters["accepted"] += 1
                    state = self._accept_state(state, new_state, obj, new_route)
                    return state, True                  # back to Insert after any acceptance
                tried.add((u, p))
                self.counters["rep_" + ("worse" if (new_state is not None or verdict == "cache_feasible") else verdict)] += 1
            return state, False

        def reorder_phase(state):
            """Swap and 2-opt on shortening moves only, in order of the saving, first improvement.
            Returns (state, accepted_any)."""
            for op in ("swap", "two_opt"):
                route = state.solver.tour_nodes
                if len(route) < (3 if op == "swap" else 4):
                    continue
                for dd, p, q in shortening_reorders(route, self.graph, self.depot, op):
                    if self.remaining() <= 0:
                        break
                    new_route = apply_reorder(route, p, q, op)
                    self.iter_global += 1; self.counters[f"prop_{op}"] += 1
                    if not cascade_feasible_route(new_route, self.graph, self.depot, self.v_max, self.T_max):
                        self.counters["reo_label_infeasible"] += 1
                        continue
                    new_state, obj, verdict = self.evaluate(state, new_route)
                    if verdict == "cache_feasible" and obj >= state.value * (1 - f_tol):
                        new_state, obj, verdict = self._resolve(state, new_route)
                    if new_state is not None and obj >= state.value * (1 - f_tol):
                        self.counters[f"acc_{op}"] += 1; self.counters["accepted"] += 1
                        state = self._accept_state(state, new_state, obj, new_route)
                        return state, True              # back to Insert: the shorter route may have room
                    self.counters["reo_f_drop" if (new_state is not None or verdict == "cache_feasible") else "reo_" + verdict] += 1
                    self.counters["worse"] += 1
            return state, False

        while self.remaining() > 0:
            state, acc = insert_phase(state)
            if acc:
                continue                                # Insert is exhausted only when it accepts nothing
            state, acc = replace_phase(state)
            if acc:
                continue
            state, acc = reorder_phase(state)
            if acc:
                continue
            # ---- local optimum of all three: shake ----
            route = state.solver.tour_nodes
            # Gunawan et al. (2015) intensification: after restart_threshold local optima
            # without improving the best, continue from the best found route.
            if state.value > self._lo_best + 1e-9:
                self._lo_best = state.value; self._noimpr = 0
            else:
                self._noimpr += 1
            if self.restart_threshold and (self._noimpr + 1) % self.restart_threshold == 0 and self.best_route:
                new_state, obj, verdict = self.evaluate(state, list(self.best_route))
                if verdict == "cache_feasible":
                    new_state, obj, verdict = self._resolve(state, list(self.best_route))
                if new_state is not None:
                    new_state.parent = None; state = new_state; route = state.solver.tour_nodes
                    self.counters["restarts"] += 1
            if self.shake_schedule == "gunawan":
                # cons escalates by one every `hold` shakes, no cap but the route, reset on improvement
                new_route, removed, self.sweep_S, _ = sweep_remove(route, self.sweep_S, self.sweep_R, 1, 0)
                self._hold += 1
                if self._hold >= self.shake_hold:
                    self._hold = 0; self.sweep_R += 1
                if self.sweep_R > len(route) - 1 - 2:
                    self.sweep_R = 1
            else:
                new_route, removed, self.sweep_S, self.sweep_R = sweep_remove(
                    route, self.sweep_S, self.sweep_R, self.sweep_cap_div, self.sweep_cap_max)
            self.counters["kicks"] += 1
            self._record_dyn(state.value, kick=1)
            if new_route is None:
                continue
            new_state, obj, verdict = self.evaluate(state, new_route)
            if verdict == "cache_feasible":
                new_state, obj, verdict = self._resolve(state, new_route)
            if new_state is not None:
                new_state.parent = None
                state = new_state
                if obj > self.best_obj:
                    self._record_best(obj, state.solver.tour_nodes)
                    self.sweep_R = 1

    # ---------------- Section 4 as written in the paper (2026-09-17) ----------------

    def _route_stats(self, state):
        """(f, T_R, E_R, cap) of a solved state, cached on the state."""
        key = tuple(state.solver.tour_nodes)
        st = getattr(self, "_stats_cache", None)
        if st is None:
            st = self._stats_cache = {}
        if key not in st:
            td = state.solver.get_tour_data()
            T = sum(td.times.values()); E = td.total_energy
            st[key] = (state.value, T, E, max(T / self.T_max, E / self.E_max))
        return st[key]

    def _enumeration(self, k):
        """E(R) for a reference with k targets.  Level cons = 1..c holds the blocks of
        cons consecutive targets that sweep the route once (successive levels
        staggered by one position, the last block of a level wrapping).  The list
        interleaves the levels: the next block of size 1, the next of size 2, ...,
        the next of size c, then again the next of size 1, a level whose sweep is
        complete being skipped.  This keeps the escalation of Vansteenwegen et al.
        (cons grows by one at every shake) and covers every target at every
        level.  c = ceil(k / D); at least two targets are kept."""
        c = max(1, -(-k // self.sweep_cap_div))
        if self.sweep_cap_max:
            c = min(c, self.sweep_cap_max)
        c = min(c, max(1, k - 2))
        start = {cons: (cons - 1) % k for cons in range(1, c + 1)}
        removed = {cons: 0 for cons in range(1, c + 1)}
        pairs = []
        while any(removed[cons] < k for cons in removed):
            for cons in range(1, c + 1):
                if removed[cons] >= k:
                    continue
                pairs.append((cons, 1 + start[cons] % k))
                start[cons] += cons; removed[cons] += cons
        return pairs

    @staticmethod
    def _remove_block(route, cons, post):
        """Route without the cons targets at positions post, post + 1, ... (wrapping)."""
        k = len(route) - 1
        if k - cons < 2:
            return None
        idxs = {1 + ((post - 1 + h) % k) for h in range(cons)}
        return [route[i] for i in range(len(route)) if i not in idxs]

    def run_start_paper(self, start_idx):
        """One start of the ILS of Section 4: random operator, leg-feasible set,
        estimated ratio, linear-ranking roulette, strict acceptance for Insert and
        Replace, (f, cap) acceptance for Swap and 2-opt, shake after theta
        iterations without an accepted move."""
        random.seed(ILS_SEED + 100000 * self.seed_offset + start_idx)
        G = self.graph; depot = self.depot
        forced = self.init_tour_kind or "R4"
        if self.no_shake and start_idx > 0:
            forced = "R3"       # R4 is deterministic, so restarts need a random construction
        if forced == "R1":
            init_tour = build_R1(self.instance)
        elif forced == "R2":
            init_tour = build_R2(self.instance, rng=random.Random(self.init_seed))
        elif forced == "R3":
            init_tour = build_R3(self.instance,
                                 rng=random.Random((self.init_seed or 0) + 1000 * start_idx))
        else:
            init_tour = build_R4(self.instance)
        state = State.initial_state(self.instance, init_tour)
        state.parent = None
        self.cache[tuple(state.solver.tour_nodes)] = state.value
        if state.value > self.best_obj:
            self._record_best(state.value, state.solver.tour_nodes)
        print(f"[{self.name}] start {start_idx} ({forced}, paper) init obj "
              f"{state.value:.2f} size {len(state.solver.tour_nodes) - 1} at {self.elapsed():.0f} s", flush=True)

        F, B = self.F_leg, self.B_leg
        c0, c1, c2 = self.drone.c_0, self.drone.c_1, self.drone.c_2
        v_mr = (c2 / c1) ** 0.25
        e_mr = c0 / v_mr + c1 * v_mr ** 2 + c2 / v_mr ** 2      # P(v_mr)/v_mr
        Ibar = {}
        for n in G.nodes:
            if n == depot:
                continue
            nd = G.nodes[n]; e, l = nd['time_window']
            Ibar[n] = float(nd.get('info_at_lowest', 1.0)) + float(nd.get('info_slope', 0.0)) * (l - e) / 2.0
        DM = self.DM
        def d(a, b): return DM[a][b]

        # Node data as flat lookups. The realized-window weight reads them once
        # per candidate, so the networkx attribute lookups are hoisted out here.
        EW, LW, I0, GAM = {}, {}, {}, {}
        for n in G.nodes:
            e, l = G.nodes[n]['time_window']
            EW[n], LW[n] = float(e), float(l)
            I0[n] = float(G.nodes[n].get('info_at_lowest', 1.0))
            GAM[n] = float(G.nodes[n].get('info_slope', 0.0))
        EW[depot], LW[depot], I0[depot], GAM[depot] = 0.0, self.T_max, 0.0, 0.0
        inv_v = 1.0 / self.v_max
        T_max = self.T_max
        d0 = self.d_floor          # paper (53): 1% of the mean leg length

        def realized(cr, chk=True):
            """Arrival bounds (45) of the route cr and its information estimate.

            Walks the forward recursion for a^min and the backward recursion for
            a^max, and sums the midpoint reward (51) of every realized window
            (47) on the way back. Returns (False, 0.0) as soon as a realized
            window is empty, which is the exact time window test of the paper.
            """
            n = len(cr)
            amin = [0.0] * n
            prev = 0.0
            for q in range(1, n):
                nq = cr[q]
                row = DM[cr[q - 1]]
                prev = max(EW[nq], prev + row[nq] * inv_v)
                if chk and prev > LW[nq]:
                    return False, 0.0
                amin[q] = prev
            back = DM[cr[-1]][depot] * inv_v
            if chk and prev + back > T_max:
                return False, 0.0
            nxt = T_max - back                      # a^max at the last target
            tot = 0.0
            for q in range(n - 1, 0, -1):
                nq = cr[q]
                hi = LW[nq] if LW[nq] < nxt else nxt
                lo = amin[q]
                if chk and hi < lo:
                    return False, 0.0
                tot += I0[nq] + GAM[nq] * (0.5 * (lo + hi) - EW[nq])
                nxt = hi - DM[cr[q - 1]][nq] * inv_v
            return True, tot

        def chain_bounds(cr):
            """Arrival bounds (45) of the current route, over ext = cr + [depot].

            Only the current route needs the arrays themselves: they make the
            slot test (46) an O(1) check for a candidate, so the O(k) walk of
            realized() is paid only by the candidates that pass it.
            """
            n = len(cr)
            amin = [0.0] * (n + 1); amax = [0.0] * (n + 1)
            prev = 0.0
            for q in range(1, n):
                nq = cr[q]
                prev = max(EW[nq], prev + DM[cr[q - 1]][nq] * inv_v)
                amin[q] = prev
            amin[n] = prev + DM[cr[-1]][depot] * inv_v
            amax[n] = T_max
            nxt = T_max - DM[cr[-1]][depot] * inv_v
            for q in range(n - 1, 0, -1):
                nq = cr[q]
                hi = LW[nq] if LW[nq] < nxt else nxt
                amax[q] = hi
                nxt = hi - DM[cr[q - 1]][nq] * inv_v
            amax[0] = nxt
            return amin, amax

        self.sweep_S, self.sweep_R = 1, 1
        self._cons, self._hold, self._noimp_lo = 1, 0, 0
        self._lo_seen = set()
        self._tie_run = 0       # consecutive reorderings accepted without a gain
        ref_state = None        # local optimum the sweep is exploring around
        ref_pairs, ref_idx = [], 0   # --sweep enum: the removals of the reference and the next one
        ref_tried = set()       # (post, cons) already applied to it
        ref_stack = []          # (reference, post, cons, tried) of the earlier optima
        self._tabu = {tuple(state.solver.tour_nodes)}   # routes occupied since the last shake
        self._rhist = deque([tuple(state.solver.tour_nodes)],
                            maxlen=max(1, NO_RETURN + 1))   # recent accepted routes
        self._hist = deque(maxlen=2 * LOOP_BREAK if LOOP_BREAK else 1)
        self._last_shake_it = 0
        count = 0
        it = 0
        while self.remaining() > 0:
            if self.idle >= self.max_iter:
                self.stop_reason = "max_iter"
                self.stop_wall = self.elapsed()
                print(f"[{self.name}] stopped by max_iter={self.max_iter} after "
                      f"{it} iterations at {self.stop_wall:.0f} s "
                      f"(best {self.best_obj:.2f} found at {self.best_wall:.1f} s)", flush=True)
                break
            if self.max_idle_shakes and self.idle_shakes >= self.max_idle_shakes:
                self.stop_reason = "idle_shakes"
                self.stop_wall = self.elapsed()
                print(f"[{self.name}] stopped by max_idle_shakes={self.max_idle_shakes} after "
                      f"{self.counters['kicks']} shakes and {it} iterations at {self.stop_wall:.0f} s "
                      f"(best {self.best_obj:.2f} found at {self.best_wall:.1f} s)", flush=True)
                break
            it += 1; self.iter_global += 1
            self.idle += 1
            route = state.solver.tour_nodes
            k = len(route) - 1

            exhausted = (self.exhaust_shake and getattr(self, "_mkey", None) == tuple(route)
                         and len(self._msets) == len(ops)
                         and all(not self._msets.get(o) for o in ops))
            if SHAKE_STALL and it - self._last_shake_it >= SHAKE_STALL:
                exhausted = True              # stagnating without exhausting the sets
                self.counters["stall_shake"] += 1
            if LOOP_BREAK and len(self._hist) == 2 * LOOP_BREAK:
                _h = list(self._hist)
                if len(set(_h)) == 2 and all(a != b for a, b in zip(_h, _h[1:])) \
                        and _h[0::2].count(_h[0]) == LOOP_BREAK:
                    exhausted = True          # alternating between two routes: stuck
                    self._hist.clear()
                    self.counters["loop_break"] += 1
            if TIE_CAP and self._tie_run >= TIE_CAP:
                exhausted = True            # a lateral walk counts as a local optimum
                self.counters["tie_forced_shake"] += 1
            if count >= self.theta or exhausted:
                # ---- local optimum reached: record whether it repeats one already seen ----
                _lok = tuple(route)
                self.counters["lo_total"] += 1
                if self.no_shake:
                    # no perturbation: this start is finished at its first local optimum
                    self.lo_log.append((start_idx, forced, state.value, self.elapsed()))
                    print(f"[{self.name}] start {start_idx} ({forced}) local optimum "
                          f"{state.value:.2f} size {len(route) - 1} after {it} iterations "
                          f"at {self.elapsed():.0f} s", flush=True)
                    break
                if _lok in self._lo_seen:
                    self.counters["lo_repeat"] += 1
                else:
                    self._lo_seen.add(_lok)
                # ---- restart from the best route after too many fruitless local optima ----
                self._noimp_lo += 1
                if (self.restart_threshold and self._noimp_lo >= self.restart_threshold
                        and self.best_route is not None
                        and list(route) != list(self.best_route)):
                    cand = state.flip(route_to_nx(list(self.best_route)))
                    self.counters["socp_calls"] += 1
                    if cand.solver is not None and cand.solver.solution is not None:
                        cand.parent = None
                        state = cand
                        self._mkey = None
                        self.counters["restarts"] += 1
                        self._noimp_lo = 0
                        self._cons, self._hold = 1, 0
                        self.sweep_R, self.sweep_S = 1, 1
                        count = 0
                        continue
                if self.shake_return and self.sweep_enum:
                    # Explicit enumeration E(R^ref): for cons = 1..c one sweep of the
                    # route in disjoint blocks of cons targets, then cons + 1.  A descent
                    # that beats the reference replaces it; one that does not returns to
                    # it and the next pair is shaken.  An exhausted enumeration falls back
                    # to the previous reference, or restarts when there is none.
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
                            self._mkey = None
                            self.counters["shake_returns"] += 1
                        if ref_idx >= len(ref_pairs):
                            self.counters["ref_exhausted"] += 1
                            if self.shake_backtrack and ref_stack:
                                ref_state, ref_pairs, ref_idx = ref_stack.pop()
                                state = ref_state
                                self._mkey = None
                                self.counters["ref_backtracks"] += 1
                            else:
                                ref_idx = 0          # the descents are random: repeat the enumeration
                                self.counters["enum_restarts"] += 1
                    route = state.solver.tour_nodes
                elif self.shake_return:
                    # Variable-neighborhood step: the sweep belongs to one route.
                    # A descent that beats the reference replaces it and restarts
                    # the sweep; one that does not sends the search back, so the
                    # next (post, cons) shakes the same route rather than the one
                    # the failed descent happened to end on.
                    if ref_state is None or state.value > ref_state.value + 1e-9:
                        if ref_state is not None:
                            # the sweep state here is the next pair the old
                            # reference would have used, so it resumes cleanly
                            ref_stack.append((ref_state, self.sweep_S, self.sweep_R, ref_tried))
                        ref_state, ref_tried = state, set()
                        self.sweep_S, self.sweep_R, self._cons = 1, 1, 1
                        self.counters["ref_improved"] += 1
                    else:
                        if state.solver.tour_nodes != ref_state.solver.tour_nodes:
                            state = ref_state
                            self._mkey = None
                            self.counters["shake_returns"] += 1
                        if self.shake_backtrack and (self.sweep_S, self.sweep_R) in ref_tried:
                            # every shake of this reference has been tried
                            self.counters["ref_exhausted"] += 1
                            if ref_stack:
                                ref_state, self.sweep_S, self.sweep_R, ref_tried = ref_stack.pop()
                                state = ref_state
                                self._mkey = None
                                self.counters["ref_backtracks"] += 1
                            else:
                                ref_tried = set()      # nothing to fall back to
                    ref_tried.add((self.sweep_S, self.sweep_R))
                    route = state.solver.tour_nodes
                # ---- shake (Algorithm 3): remove cons consecutive targets ----
                if self.sweep_enum:
                    # knapsack look-ahead over the next pairs of the enumeration; the best
                    # is applied and the enumeration advances by one
                    d0_ = route_distance(route, G, depot)
                    pick = None
                    for j_ in range(ref_idx, min(ref_idx + max(1, SHAKE_KNAP), len(ref_pairs))):
                        cons_, post_ = ref_pairs[j_]
                        cand = self._remove_block(route, cons_, post_)
                        if cand is None:
                            continue
                        sc = self._knap(cand, max(0.0, d0_ - route_distance(cand, G, depot)))
                        if pick is None or sc > pick[0]:
                            pick = (sc, cand, cons_)
                    ref_idx += 1
                    if pick is None:
                        new_route, removed = None, []
                    else:
                        new_route, removed = pick[1], [None] * pick[2]
                        self.counters["knap_picked"] += 1
                        self.counters["shake_removed"] += pick[2]
                elif self.shake_schedule == "gunawan":
                    new_route, removed, self.sweep_S, _ = sweep_remove(
                        route, self.sweep_S, self._cons, 1, 0)      # cap_div = 1: no cap
                    self._hold += 1
                    if self._hold >= self.shake_hold:
                        self._cons += 1
                        self._hold = 0
                elif KNAP_SIZE:
                    # the sweep chooses where, the knapsack chooses how many
                    d0 = route_distance(route, G, depot)
                    f0 = self._mctx[0] if self._mctx else state.value
                    n_t = len(route) - 1
                    pick = None
                    for r_ in range(1, max(2, n_t - 1)):
                        cand, rem_, S2, R2 = sweep_remove(route, self.sweep_S, r_, 1, 0)
                        if cand is None:
                            break
                        freed = d0 - route_distance(cand, G, depot)
                        _, lost = realized(cand, False)
                        sc = self._knap(cand, max(0.0, freed)) - (info0 - lost)
                        if pick is None or sc > pick[0]:
                            pick = (sc, cand, rem_, S2, R2)
                    if pick is None:
                        new_route, removed = None, []
                    else:
                        _, new_route, removed, self.sweep_S, self.sweep_R = pick
                        self.counters["knap_size"] += len(removed)
                        self.counters["knap_picked"] += 1
                elif SHAKE_KNAP:
                    # Look ahead over the next SHAKE_KNAP sweep candidates and
                    # apply the one that buys the most, but advance the sweep by a
                    # single step: the enumeration must still cover the tour, so a
                    # candidate that is passed over here comes up again next time.
                    d0 = route_distance(route, G, depot)
                    S_, R_ = self.sweep_S, self.sweep_R
                    S_next = R_next = None
                    pick = None
                    for _ in range(SHAKE_KNAP):
                        cand, rem_, S2, R2 = sweep_remove(
                            route, S_, R_, self.sweep_cap_div, self.sweep_cap_max)
                        if S_next is None:
                            S_next, R_next = S2, R2      # the sweep advances by one
                        if cand is not None:
                            sc = self._knap(cand, max(0.0, d0 - route_distance(cand, G, depot)))
                            if pick is None or sc > pick[0]:
                                pick = (sc, cand, rem_)
                        S_, R_ = S2, R2
                    self.sweep_S, self.sweep_R = S_next, R_next
                    if pick is None:
                        new_route, removed = None, []
                    else:
                        _, new_route, removed = pick
                        self.counters["knap_picked"] += 1
                elif SHAKE_ROOM:
                    # take the best of the next SHAKE_ROOM sweep candidates
                    base = self._room(route)
                    S_, R_ = self.sweep_S, self.sweep_R
                    pick = None
                    for _ in range(SHAKE_ROOM):
                        cand, rem_, S2, R2 = sweep_remove(
                            route, S_, R_, self.sweep_cap_div, self.sweep_cap_max)
                        if cand is not None:
                            sc = self._room(cand) - base
                            if pick is None or sc > pick[0]:
                                pick = (sc, cand, rem_, S2, R2)
                        S_, R_ = S2, R2
                    if pick is None:
                        new_route, removed = None, []
                        self.sweep_S, self.sweep_R = S_, R_
                    else:
                        _, new_route, removed, self.sweep_S, self.sweep_R = pick
                        self.counters["room_opened"] += max(0, pick[0])
                else:
                    new_route, removed, self.sweep_S, self.sweep_R = sweep_remove(
                        route, self.sweep_S, self.sweep_R, self.sweep_cap_div, self.sweep_cap_max)
                count = 0
                self._tie_run = 0
                self._tabu = {tuple(route)}
                self._rhist.clear()
                self._hist.clear()
                self._last_shake_it = it
                self.counters["kicks"] += 1
                self.idle_shakes += 1
                self._record_dyn(state.value, kick=1)
                if new_route is not None:
                    new_state, obj, verdict = self.evaluate(state, new_route)
                    if verdict == "cache_feasible":
                        new_state, obj, verdict = self._resolve(state, new_route)
                    if new_state is not None:
                        new_state.parent = None
                        state = new_state
                        self._mkey = None    # the route changed: rebuild the sets
                        self._record_route("shake", state.solver.tour_nodes, obj, "shake", state)
                        if obj > self.best_obj:
                            self._record_best(obj, state.solver.tour_nodes)
                            self.sweep_R = 1; self._cons, self._hold, self._noimp_lo = 1, 0, 0
                continue

            # ---- Algorithm 2: operator, feasible set, ratio, draw ----
            Nprime = set(G.nodes) - set(route)
            ops = []
            if Nprime:
                ops += ["add", "replace"]
            if k >= 2:
                ops.append("swap")
            if k >= 3:
                ops.append("two_opt")
            ops = [o for o in ops if o not in self.disabled_ops]
            if not ops:
                count += 1; continue
            op = random.choice(ops)
            self.counters[f"prop_{op}"] += 1
            # The feasible set depends on the route alone, so it is built once per
            # route and reused until a move is accepted or a shake changes the route.
            rkey = tuple(route)
            if getattr(self, "_mkey", None) != rkey:
                # The sets are discarded with the route. Resuming a partly drawn
                # set on a return visit was measured to be worse: it sends the
                # route to the shake step sooner, because exhaustion is the shake
                # trigger, and the redraws it saves are answered from the
                # evaluated-route store without a solve anyway.
                self._mkey, self._msets, self._mctx = rkey, {}, None
            if self._mctx is None:
                _f, _T, _E, _cap = self._route_stats(state)
                _eps = _beta = None
                if self.paper_label or os.environ.get("ILS_PAPER_DIAG"):
                    _eps = compute_epsilon(route, G, depot, self.v_max)
                    _beta = compute_beta(route, G, depot, self.v_max, self.T_max)
                _ok0, _info0 = realized(route)
                _amin0, _amax0 = chain_bounds(route)
                self._mctx = (_f, _T, _E, _cap, _eps, _beta,
                              self.E_max / self.e_per_m - route_distance(route, G, depot),
                              _info0, _amin0, _amax0)
                self.counters["sets_built"] += 1
            f, T_R, E_R, cap_R, eps, beta, d_room, info0, amin0, amax0 = self._mctx
            ext = route + [depot]                                # ext[p] = r_p, ext[k+1] = depot
            cached_set = self._msets.get(op)
            moves = [] if cached_set is None else cached_set     # (score, key, dd)

            info_mode = os.environ.get("ILS_INFO_MODE", "route")

            def score_of(dI, dd_w):
                # paper (54): information change over the distance the move adds,
                # charging nothing for the distance a move frees
                return dI / (d0 + dd_w if dd_w > 0.0 else d0)

            if cached_set is not None:
                pass
            elif self.fast_sets:
                # the same sets and weights as the four blocks below, in O(1) per candidate
                if op == "add":
                    moves = _fs.build_add(self._nd, route, Nprime, amin0, amax0, d_room, self.counters)
                elif op == "replace":
                    moves = _fs.build_replace(self._nd, route, Nprime, amin0, amax0, d_room, self.counters)
                elif op == "swap":
                    moves = _fs.build_swap(self._nd, route, amin0, amax0, d_room, None, self.counters)
                else:
                    moves = _fs.build_two_opt(self._nd, route, amin0, amax0, d_room, None, self.counters)
            elif op == "add":
                FR, BR = chained_sets(route, F, B, depot)
                for p in range(1, k + 2):
                    i, j = ext[p - 1], ext[p]
                    cand = FR[p] & BR[p] & Nprime
                    dij = d(i, j)
                    for u in cand:
                        dd = d(i, u) + d(u, j) - dij
                        if dd > d_room:
                            self.counters["floor_excluded"] += 1
                            continue
                        # exact slot test (46) in O(1) from the bounds of R:
                        # inserting u between i and j leaves a_min at i and
                        # a_max at j unchanged, so this decides feasibility
                        au = EW[u]
                        cand_a = amin0[p - 1] + d(i, u) * inv_v
                        if cand_a > au: au = cand_a
                        if au > LW[u] or au + d(u, j) * inv_v > amax0[p]:
                            self.counters["label_excluded"] += 1
                            continue
                        ok, info_new = realized(route[:p] + [u] + route[p:], False)
                        if not ok:
                            self.counters["label_excluded"] += 1
                            continue
                        dI = (info_new - info0) if info_mode == "route" else Ibar[u]
                        _sa = (dI / dd if (INSERT_RATIO and dd > 1e-9)
                               else dI if INSERT_RATIO else score_of(dI, dd))
                        moves.append((_sa, ("add", u, p), dd))
            elif op == "replace":
                # A Replace is a removal followed by an insertion: the slot test
                # of R (-) r_p uses the same prefix and suffix as R, so the sets
                # and the bounds of R serve it unchanged.
                FR, BR = chained_sets(route, F, B, depot)
                for p in range(1, k + 1):
                    i, v, j = ext[p - 1], ext[p], ext[p + 1]
                    cand = FR[p] & BR[p + 1] & Nprime
                    base = d(i, v) + d(v, j)
                    gap = d(i, j)                 # route length after removing r_p
                    for u in cand:
                        dd = d(i, u) + d(u, j) - base        # true change of the route length
                        dd_w = d(i, u) + d(u, j) - gap       # detour of u into the gap: cost
                        if dd > d_room:
                            self.counters["floor_excluded"] += 1
                            continue
                        au = EW[u]
                        cand_a = amin0[p - 1] + d(i, u) * inv_v
                        if cand_a > au: au = cand_a
                        if au > LW[u] or au + d(u, j) * inv_v > amax0[p + 1]:
                            self.counters["label_excluded"] += 1
                            continue
                        ok, info_new = realized(route[:p] + [u] + route[p + 1:], False)
                        if not ok:
                            self.counters["label_excluded"] += 1
                            continue
                        dI = (info_new - info0) if info_mode == "route" else Ibar[u]
                        _sr = (dI / dd_w if (INSERT_RATIO and dd_w > 1e-9)
                               else dI if INSERT_RATIO else score_of(dI, dd_w))
                        moves.append((_sr, ("replace", u, p), dd))
            elif op == "swap":
                buf = list(route)          # one buffer, mutated and undone per candidate
                for p in range(1, k):
                    rp = ext[p]
                    for q in range(p + 1, k + 1):
                        rq = ext[q]
                        if rp not in F[rq]:
                            continue
                        ok = True
                        for m in range(p + 1, q):
                            rm = ext[m]
                            if rm not in F[rq] or rp not in F[rm]:
                                ok = False; break
                        if not ok:
                            continue
                        if q == p + 1:
                            dd = d(ext[p - 1], rq) + d(rp, ext[q + 1]) - d(ext[p - 1], rp) - d(rq, ext[q + 1])
                        else:
                            dd = (d(ext[p - 1], rq) + d(rq, ext[p + 1]) + d(ext[q - 1], rp) + d(rp, ext[q + 1])
                                  - d(ext[p - 1], rp) - d(rp, ext[p + 1]) - d(ext[q - 1], rq) - d(rq, ext[q + 1]))
                        if dd > d_room:
                            self.counters["floor_excluded"] += 1
                            continue
                        if REORDER_SHORTEN and dd >= 0.0:
                            self.counters["not_shorter"] += 1
                            continue
                        buf[p], buf[q] = buf[q], buf[p]
                        ok, info_new = realized(buf)
                        buf[p], buf[q] = buf[q], buf[p]
                        if not ok:
                            self.counters["label_excluded"] += 1
                            continue
                        if REORDER_W == "exch":
                            # information moved by the exchange itself: the higher
                            # slope belongs at the later visit. Isolates the intended
                            # effect from the downstream retiming that dominates dI.
                            _s = (GAM[rp] - GAM[rq]) * (amin0[q] - amin0[p])
                        else:
                            _s = (1.0 if REORDER_UNIFORM else
                                  (-dd) if REORDER_W == "dsave" else
                                  score_of(info_new - info0, dd))
                        moves.append((_s, ("swap", p, q), dd))
            else:   # two_opt: reversed pairs, incremental in q
                buf = list(route)
                for p in range(1, k):
                    for q in range(p + 1, k + 1):
                        rq = ext[q]
                        if any(ext[a] not in F[rq] for a in range(p, q)):
                            break                       # a longer segment contains this one
                        dd = d(ext[p - 1], rq) + d(ext[p], ext[q + 1]) - d(ext[p - 1], ext[p]) - d(rq, ext[q + 1])
                        if dd > d_room:
                            self.counters["floor_excluded"] += 1
                            continue
                        if REORDER_SHORTEN and dd >= 0.0:
                            self.counters["not_shorter"] += 1
                            continue
                        a2, b2 = p, q
                        while a2 < b2:
                            buf[a2], buf[b2] = buf[b2], buf[a2]; a2 += 1; b2 -= 1
                        ok, info_new = realized(buf)
                        a2, b2 = p, q
                        while a2 < b2:
                            buf[a2], buf[b2] = buf[b2], buf[a2]; a2 += 1; b2 -= 1
                        if not ok:
                            self.counters["label_excluded"] += 1
                            continue
                        if REORDER_W == "exch":
                            _s = 0.0
                            _a, _b = p, q
                            while _a < _b:
                                _s += (GAM[ext[_a]] - GAM[ext[_b]]) * (amin0[_b] - amin0[_a])
                                _a += 1; _b -= 1
                        else:
                            _s = (1.0 if REORDER_UNIFORM else
                                  (-dd) if REORDER_W == "dsave" else
                                  score_of(info_new - info0, dd))
                        moves.append((_s, ("two_opt", p, q), dd))

            if cached_set is None and self.reorder_rcl and op in ("swap", "two_opt") \
                    and len(moves) > self.reorder_rcl:
                # restricted candidate list: only the L_r reorderings of largest exchange
                # value are offered to the roulette (their acceptance rate decays with rank)
                self.counters["reorder_trimmed"] += len(moves) - self.reorder_rcl
                moves = heapq.nlargest(self.reorder_rcl, moves, key=lambda m: m[0])
            if cached_set is None:
                random.shuffle(moves)          # break ties between equal weights
                self._msets[op] = moves
                if SCORE_DIAG and moves:
                    self.counters[f"sneg_{op}"] += sum(1 for m in moves if m[0] < 0.0)
                    self.counters[f"szero_{op}"] += sum(1 for m in moves if m[0] == 0.0)
                    self.counters[f"stot_{op}"] += len(moves)
            if not moves:
                self.counters["saturated"] += 1
                count += 1; continue

            new_route = None
            while moves:
                # Paper (55): the scores carry a sign, so they are shifted onto a
                # positive scale over the set being drawn from before the roulette.
                # The shift leaves the least attractive move a small probability.
                if (INSERT_RATIO and op in ("add", "replace")) or \
                   (REORDER_W == "dsave" and op in ("swap", "two_opt")):
                    # weights are already non-negative: plain roulette, ratios kept
                    wts = [m[0] if m[0] > 0.0 else 0.0 for m in moves]
                    if sum(wts) <= 0.0:
                        wts = None; idx = random.randrange(len(moves))
                    else:
                        idx = random.choices(range(len(moves)), weights=wts, k=1)[0]
                elif REORDER_W == "best" and op in ("swap", "two_opt"):
                    # take the highest-scoring reordering: no shift, no epsilon.
                    # The score already ranks accepted moves above the median, so
                    # following it exactly removes the wasted draws below them.
                    wts = None
                    idx = max(range(len(moves)), key=lambda i: moves[i][0])
                elif REORDER_W == "rank" and op in ("swap", "two_opt"):
                    order = sorted(range(len(moves)), key=lambda i: moves[i][0])
                    wts = [0.0] * len(moves)
                    for r_, i_ in enumerate(order, 1): wts[i_] = float(r_)
                    idx = random.choices(range(len(moves)), weights=wts, k=1)[0]
                elif SELECT == "rcl":
                    # uniform over the K best: scale-free and origin-free
                    k = min(RCL_K, len(moves))
                    top = sorted(range(len(moves)), key=lambda i: -moves[i][0])[:k]
                    wts = None
                    idx = random.choice(top)
                elif SELECT == "rank":
                    # weight by rank, best gets n, worst gets 1
                    order = sorted(range(len(moves)), key=lambda i: moves[i][0])
                    wts = [0.0] * len(moves)
                    for r, i in enumerate(order, 1):
                        wts[i] = float(r)
                    idx = random.choices(range(len(moves)), weights=wts, k=1)[0]
                else:
                  s_lo = min(m[0] for m in moves)
                  s_hi = max(m[0] for m in moves)
                  rng = s_hi - s_lo
                  if rng <= 0.0:
                    wts = None
                    idx = random.randrange(len(moves))
                  else:
                    shift = SCALE_EPS * rng - s_lo
                    wts = [m[0] + shift for m in moves]
                    idx = random.choices(range(len(moves)), weights=wts, k=1)[0]
                _w_chosen = 1.0 if wts is None else wts[idx]
                _w_sum = float(len(moves)) if wts is None else sum(wts)
                _s_chosen, key, dd = moves.pop(idx)   # drawn once: removed from the set either way
                if wts is not None:
                    del wts[idx]
                if key[0] == "add":
                    _, u, p = key; cand_route = route[:p] + [u] + route[p:]
                elif key[0] == "replace":
                    _, u, p = key; cand_route = route[:p] + [u] + route[p + 1:]
                elif key[0] == "swap":
                    _, p, q = key; cand_route = list(route); cand_route[p], cand_route[q] = cand_route[q], cand_route[p]
                else:
                    _, p, q = key; cand_route = route[:p] + route[p:q + 1][::-1] + route[q + 1:]
                if NO_REVISIT and tuple(cand_route) in self._tabu:
                    self.counters["revisit_excluded"] += 1
                    continue
                if NO_RETURN and op in ("swap", "two_opt") and _alternates(
                        list(self._rhist) + [tuple(cand_route)], NO_RETURN):
                    self.counters["return_excluded"] += 1
                    continue
                if INFO_BOUND and op in ("add", "replace"):
                    _ub = self._info_upper(cand_route)
                    if _ub is not None and _ub <= f:
                        self.counters["bound_pruned"] += 1
                        if BOUND_AUDIT:
                            # solve the move the bound rejected and check the bound held
                            _st, _ob, _vd = self.evaluate(state, cand_route)
                            if _ob is not None:
                                self.counters["audit_solved"] += 1
                                if _ob > _ub + 1e-6:
                                    self.counters["audit_bound_violated"] += 1
                                if _ob > f + 1e-6:
                                    self.counters["audit_would_accept"] += 1
                        if INFO_BOUND_END:
                            break          # the iteration passes: no solve is paid
                        count += 1
                        continue
                if self.cache.get(tuple(cand_route), _MISS) is None:
                    # stored as infeasible: it can never be accepted, so drop it.
                    # A route stored WITH a value is not dropped: its value depends
                    # on the route alone, so it may be an improvement over a
                    # different current route even though it was refused before.
                    self.counters["cache_excluded"] += 1
                    continue
                if self._mt_fh is not None:
                    chosen = "-".join(str(x) for x in key[1:])
                    if wts is None:
                        rest = [(kk, 1.0) for _, kk, _ in moves[:self._mt_top]]
                        rank = 1
                    else:
                        pairs = sorted(zip(wts, moves), key=lambda x: -x[0])[:self._mt_top]
                        rest = [(kk, ww) for ww, (_, kk, _) in pairs]
                        rank = 1 + sum(1 for w in wts if w > _w_chosen)
                    self._pending_ms = (op, len(moves) + 1, chosen, rank, _w_chosen, _w_sum,
                                        [("-".join(str(x) for x in kk[1:]), ww) for kk, ww in rest])
                new_route = cand_route
                break
            if new_route is None:
                self.counters["saturated"] += 1
                count += 1; continue

            if os.environ.get("ILS_PAPER_DIAG"):
                _lab = cascade_feasible_route(new_route, G, depot, self.v_max, self.T_max)
                self.counters[f"diag_label_{'ok' if _lab else 'no'}"] += 1
            if self.set_cache:
                skey = frozenset(new_route)
                prev = self._set_best.get(skey)
                if prev is not None and op in ("swap", "two_opt") and prev >= f - 1e-9:
                    self.counters["set_excluded"] += 1     # a permutation of a set already explored
                    count += 1
                    continue
            self.counters[f"eval_{op}"] += 1
            new_state, obj, verdict = self.evaluate(state, new_route)
            if new_state is None and obj is not None:
                # the route's value is in the store, so the acceptance test is
                # answered without a solve; a solve is paid only to move there,
                # since the accepted state has to carry the schedule
                _tol = 1e-6 * max(1.0, abs(f))
                _want = (obj > f) if op in ("add", "replace") else (obj > f - _tol)
                if _want:
                    new_state, obj, verdict = self._resolve(state, new_route)
                else:
                    self.counters["cache_worse"] += 1
                    self.counters["worse"] += 1; self.counters[f"worse_{op}"] += 1
                    self._record_route(op, new_route, obj, "refused")
                    count += 1
                    self._record_dyn(state.value)
                    continue
            if os.environ.get("ILS_PAPER_DIAG"):
                _v = "feas" if new_state is not None or verdict in ("cache_feasible",) else "infeas"
                self.counters[f"diag_{'labok' if _lab else 'labno'}_{_v}"] += 1
            if new_state is None:
                self.counters[f"reject_{op}"] += 1      # infeasible or screened at evaluation
                self._record_route(op, new_route, None, "infeasible")   # no solution to read
                count += 1
                continue
            if self.set_cache:
                k2 = frozenset(new_route)
                if obj > self._set_best.get(k2, -float("inf")):
                    self._set_best[k2] = obj
            # ---- acceptance criterion ----
            if op in ("add", "replace"):
                accept = obj > f
            elif NO_TIE:
                accept = obj > f
            elif REORDER_ACCEPT == "tiefix":
                # The paper's rule with the tie band narrowed to numerical noise.
                # At 1e-6 the band was wider than real objective differences, so a
                # pair of routes was an improvement one way and a tie the other and
                # the search alternated between them forever. Here every accepted
                # move either raises f or leaves it unchanged and lowers cap, so
                # (f, -cap) rises lexicographically and no cycle can form, while
                # an improvement is never refused however small it is.
                if obj > f:
                    accept = True
                elif abs(obj - f) <= 1e-9 * max(1.0, abs(f)):
                    _, _, _, cap_new = self._route_stats(new_state)
                    accept = cap_new < cap_R
                else:
                    accept = False
            elif REORDER_ACCEPT == "lossq":
                # Accept a gain in information outright; accept a reordering that
                # does not gain (equal or worse) only when it frees capacity for
                # a later insertion.
                if obj > f + 1e-6 * max(1.0, abs(f)):
                    accept = True
                else:
                    _, _, _, cap_new = self._route_stats(new_state)
                    accept = cap_new < cap_R
            elif REORDER_ACCEPT == "combo":
                # Accept when the objective does not drop, or when it drops but
                # the reordering frees capacity for a later insertion.
                tol = 1e-6 * max(1.0, abs(f))
                if obj >= f - tol:
                    accept = True
                else:
                    _, _, _, cap_new = self._route_stats(new_state)
                    accept = cap_new < cap_R
            elif REORDER_ACCEPT == "loss":
                # Accept a strict improvement, or a strictly worse objective that
                # frees capacity: information given up now in exchange for room
                # that a later insertion can use.
                tol = 1e-6 * max(1.0, abs(f))
                if obj > f + tol:
                    accept = True
                elif obj < f - tol:
                    _, _, _, cap_new = self._route_stats(new_state)
                    accept = cap_new < cap_R
                else:
                    accept = False
            elif REORDER_ACCEPT == "geq":
                # Accept any reordering that does not lose objective, whatever
                # it does to the capacity.
                accept = obj >= f - 1e-6 * max(1.0, abs(f))
            elif REORDER_ACCEPT == "ratio":
                _, _, _, cap_new = self._route_stats(new_state)
                accept = (obj / cap_new) > (f / cap_R) * (1.0 + 1e-9) if cap_new > 0 else obj > f
            else:
                _, _, _, cap_new = self._route_stats(new_state)
                accept = obj > f or (abs(obj - f) <= 1e-6 * max(1.0, abs(f)) and cap_new < cap_R)
            if accept:
                if obj > f + 1e-6 * max(1.0, abs(f)):
                    self._tie_run = 0
                elif op in ("swap", "two_opt"):
                    self._tie_run += 1
                new_state.parent = None
                # Only a reordering can be repeated: a permutation of the current
                # node set cannot equal a route reached by Insert or Replace, which
                # change that set. Recording the predecessor only for Swap and
                # 2-opt makes the tabu explicitly a reordering-repetition rule.
                if op not in ("swap", "two_opt"):
                    self._rhist.clear()       # a new visit set: the old routes are unreachable
                state = new_state
                self._rhist.append(tuple(state.solver.tour_nodes))   # the route now occupied
                self._tabu.add(tuple(state.solver.tour_nodes))
                self._hist.append(tuple(state.solver.tour_nodes))
                self._mkey = None            # the route changed: rebuild the sets
                count = 0
                self.counters["accepted"] += 1; self.counters[f"acc_{op}"] += 1
                self._record_route(op, state.solver.tour_nodes, obj, "accepted", state)
                if obj > self.best_obj:
                    self._record_best(obj, state.solver.tour_nodes)
                    self.sweep_R = 1; self._cons, self._hold, self._noimp_lo = 1, 0, 0
            else:
                count += 1
                self.counters["worse"] += 1; self.counters[f"worse_{op}"] += 1
                self._record_route(op, new_route, obj, "refused", new_state)
            self._record_dyn(state.value)
        self.start_log.append((start_idx, forced, None, self.best_obj))
        print(f"[{self.name}] start {start_idx} ({forced}, paper) done after {it} iterations "
              f"({self.elapsed():.0f} s elapsed)", flush=True)

    def run(self):
        if self.no_shake:
            for start_idx in range(self.n_starts):
                self.run_start_paper(start_idx)
                if self.remaining() <= 0:
                    print(f"[{self.name}] safeguard budget reached after "
                          f"{start_idx + 1} starts", flush=True)
                    break
            self.stop_reason, self.stop_wall = "starts", self.elapsed()
            print(f"\n[{self.name}] FINISHED  {len(self.lo_log)} local optima  "
                  f"total {self.stop_wall:.0f} s  best {self.best_obj:.2f}", flush=True)
            for s_, k_, v_, w_ in self.lo_log:
                print(f"    start {s_} ({k_}): {v_:.2f} at {w_:.0f} s", flush=True)
            return self
        start_idx = 0
        while self.remaining() > MIN_RESTART_BUDGET or start_idx == 0:
            if self.local_search == "phases":
                self.run_start_phased(start_idx)
            elif self.local_search == "paper":
                self.run_start_paper(start_idx)
            else:
                self.run_start(start_idx)
            start_idx += 1
            if self.stop_reason is not None:
                break               # the iteration limit ends the run, not just this start
            if self.remaining() <= 0:
                break
        wall = self.elapsed()
        if self.stop_reason is None:
            self.stop_reason = "budget"
            self.stop_wall = wall
        print(f"\n[{self.name}] FINISHED  budget {self.budget:.0f} s  "
              f"wall {wall:.0f} s  starts {start_idx}  "
              f"stopped by {self.stop_reason} at {self.stop_wall:.0f} s", flush=True)
        print(f"[{self.name}] best obj {self.best_obj:.2f}  "
              f"size {len(self.best_route) - 1 if self.best_route else 0}  "
              f"found at {self.best_wall:.1f} s (iter {self.best_iter})",
              flush=True)
        if self.target is not None:
            beat = (f"{self.t_beat_target:.0f} s" if self.t_beat_target
                    else "never")
            print(f"[{self.name}] target {self.target:.2f}  "
                  f"first surpassed: {beat}", flush=True)
        import run_ils_final_scored as _sc
        self.counters["chk1_fb_excluded"] = _sc.CHECK_STATS["fb_excluded"]
        self.counters["chk2_cascade_rejected"] = _sc.CHECK_STATS["cascade_rejected"] + REORDER_STATS["cascade_rejected"]
        self.counters["chk2_cascade_passed"] = _sc.CHECK_STATS["cascade_passed"]
        print(f"[{self.name}] counters: {dict(self.counters)}", flush=True)
        if self.adaptive_ops and self.op_weights:
            tot = sum(self.op_weights.values())
            probs = {o: round(w / tot, 3)
                     for o, w in sorted(self.op_weights.items())}
            print(f"[{self.name}] final operator probabilities: {probs}",
                  flush=True)
        print(f"[{self.name}] best route: {self.best_route}", flush=True)
        if self._mt_fh is not None:
            self._mt_fh.flush(); self._mt_fh.close()
        if self._rt_fh is not None:
            self._rt_fh.flush(); self._rt_fh.close()
            print(f"[{self.name}] route trace written", flush=True)
        if self._dyn_fh is not None:
            self._dyn_fh.flush(); self._dyn_fh.close()
            print(f"[{self.name}] dynamics trace -> {self.dynamics_out} "
                  f"({self._dyn_n} rows)", flush=True)
        return self


def write_outputs(run, tag):
    safe = run.name.lower().replace(" ", "_").replace("(", "").replace(")", "")
    prefix = f"experiments/tm_ils_{safe}_{tag}"
    with open(prefix + "_trace.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["wall_s", "iter", "best_obj"])
        for row in run.trace:
            w.writerow([f"{row[0]:.2f}", row[1], f"{row[2]:.4f}"])
    with open(prefix + "_summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["instance", "budget_s", "best_obj", "best_wall_s",
                    "best_iter", "tour_size", "starts", "target",
                    "t_beat_target_s", "socp_calls", "cache_feas",
                    "cache_infeas", "lb_reject", "kicks", "best_route"])
        w.writerow([run.name, run.budget, f"{run.best_obj:.4f}",
                    f"{run.best_wall:.1f}", run.best_iter,
                    len(run.best_route) - 1 if run.best_route else 0,
                    len(run.start_log), run.target,
                    f"{run.t_beat_target:.1f}" if run.t_beat_target else "",
                    run.counters["socp_calls"], run.counters["cache_feas"],
                    run.counters["cache_infeas"], run.counters["lb_reject"],
                    run.counters["kicks"],
                    "-".join(str(n) for n in (run.best_route or []))])
    fig, ax = plt.subplots(figsize=(10, 5))
    xs = [r[0] for r in run.trace]
    ys = [r[2] for r in run.trace]
    ax.step(xs, ys, where="post", color="tab:blue", label="ILS best")
    if run.target is not None:
        ax.axhline(run.target, color="tab:red", linestyle="--",
                   label=f"MISOCP incumbent {run.target:.0f}")
    ax.set_xlabel("wall-clock (s)")
    ax.set_ylabel("objective")
    ax.set_title(f"Time-matched multi-start ILS on {run.name} "
                 f"({run.budget:.0f} s budget)")
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(prefix + "_trace.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Wrote {prefix}_trace.csv / _summary.csv / _trace.png", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", required=True,
                    help="instance name, e.g. 'C101 (100)'")
    ap.add_argument("--budget", type=float, default=3600.0)
    ap.add_argument("--target", type=float, default=None,
                    help="MISOCP incumbent objective for reference")
    ap.add_argument("--stop-at-target", action="store_true",
                    help="end the run once the objective reaches --target (proven optimum)")
    ap.add_argument("--tag", default=None, help="output filename tag")
    ap.add_argument("--relocate", action="store_true",
                    help="Add Or-opt-1 relocation to the operator pool "
                         "(off by default; matches the paper's add/replace set).")
    ap.add_argument("--no-lb-filter", action="store_true")
    ap.add_argument("--escalate-kicks", action="store_true",
                    help="Escalate kick size under deep stagnation "
                         "({2,3}->{4,6}->{8,12}); off by default (fixed {2,3}).")
    ap.add_argument("--restart-stall", type=int, default=RESTART_STALL)
    ap.add_argument("--tilt", type=float, default=0.0,
                    help="Probability of accepting a worse move (lateral "
                         "plateau escape); 0.0 = pure hill-climbing (default).")
    ap.add_argument("--ruin-mode", choices=["segment", "reward", "sweep"],
                    default="sweep",
                    help="Kick ruin: random segment (default), drop the "
                         "lowest-reward-contribution nodes (reward-guided "
                         "LNS), or the Vansteenwegen sweep shake "
                         "(escalating consecutive removal with positional "
                         "coverage, no tabu).")
    ap.add_argument("--no-loiter", action="store_true",
                    help="ablation: pin every flight length to the straight-line distance, so the "
                         "drone can retime only by changing speed and never by flying further")
    ap.add_argument("--fixed-speed", action="store_true",
                    help="Ablation: pin v_min = v_max = v_mr (no speed "
                         "optimization; loitering still allowed).")
    ap.add_argument("--cand-weight", choices=["max", "pair"],
                    default="max", help="candidate order for Insert/Replace: paper weight, or "
                                        "omega^2 over the time, energy, or combined insertion cost")
    ap.add_argument("--local-search", choices=["random", "phases", "paper"], default="paper",
                    help="random: operator drawn uniformly, strict acceptance, shake after theta; "
                         "phases: Insert, Replace, then Swap/2-opt each to a local optimum, shake at a local optimum; "
                         "paper: Section 4 as written (leg sets chained along the route, estimated ratio, "
                         "linear-ranking roulette, (f, cap) acceptance for reorders, shake after theta)")
    ap.add_argument("--set-cache", action="store_true",
                    help="paper mode: skip a candidate that reorders a set of targets already "
                         "evaluated, unless it shortens the route (Swap and 2-opt keep their "
                         "capacity role); cuts the permutation churn")
    ap.add_argument("--move-trace", default=None,
                    help="paper mode: with --route-trace, also record the candidate set that was on "
                         "offer at each recorded event (top MOVE_TRACE_TOP candidates by weight)")
    ap.add_argument("--route-trace", default=None,
                    help="paper mode: append (event, iter, wall, obj, route) to this CSV on every "
                         "accepted move and every shake, for the route animation")
    ap.add_argument("--no-exhaust-shake", dest="exhaust_shake", action="store_false",
                    help="turn off the exhaustion trigger and fall back to the theta counter")
    ap.add_argument("--exhaust-shake", action="store_true", default=True,
                    help="paper mode: shake as soon as all four feasible sets of the current route "
                         "are exhausted, instead of waiting for theta iterations without an accepted move")
    ap.add_argument("--no-paper-label", dest="paper_label", action="store_false",
                    help="turn off the arrival propagation and screen only by the leg sets")
    ap.add_argument("--paper-label", action="store_true", default=True,
                    help="paper mode: also require the arrival propagation along the route "
                         "(the leg condition chained with travel times) when building the feasible set")
    ap.add_argument("--pair-weight", choices=["rho", "ratio"], default="ratio",
                    help="weight on the feasible set: attainable slot reward, or its square over the normalized slot cost")
    ap.add_argument("--shake-schedule", choices=["cap", "gunawan"], default="cap",
                    help="cap: cons escalates to c = ceil(k/D) and resets (Vansteenwegen); gunawan: cons held for --shake-hold shakes, then +1, no cap, reset on improvement")
    ap.add_argument("--shake-return", action="store_true",
                    help="shake the reference local optimum instead of the current route: a "
                         "descent that does not beat the reference returns to it, so (post, cons) "
                         "sweep one route's neighborhood instead of drifting")
    ap.add_argument("--shake-backtrack", action="store_true",
                    help="with --shake-return: when a reference's (post, cons) sweep returns to "
                         "a shake already applied to it, fall back to the previous local optimum "
                         "and resume its sweep instead of re-treading this one")
    ap.add_argument("--no-shake", action="store_true",
                    help="diagnostic: no perturbation at all. Each start runs to its first "
                         "local optimum and stops; --starts gives the number of starts and "
                         "the best local optimum is reported with the total time")
    ap.add_argument("--starts", type=int, default=1,
                    help="number of independent local searches for --no-shake")
    ap.add_argument("--max-iter", type=int, default=None,
                    help="stop the run after this many consecutive iterations without an "
                         "improvement of the best found solution; the wall-clock budget is "
                         "then only a safeguard (default: never binds)")
    ap.add_argument("--shake-hold", type=int, default=2, help="gunawan schedule: shakes per cons value (Gunawan et al.: 2)")
    ap.add_argument("--restart-threshold", type=int, default=0, help="restart from the best route after this many local optima without improvement (Gunawan et al.: 10); 0 = off")
    ap.add_argument("--rcl", type=int, default=5, help="keep the best RCL pairs of the feasible set before the roulette (Gunawan et al.: 5); 0 = all")
    ap.add_argument("--kappa-max", type=int, default=40, help="draws for Swap and 2-opt")
    ap.add_argument("--uniform-sampling", action="store_true",
                    help="Ablation: draw candidates uniformly instead of "
                         "reward-weighted (Section 5.4).")
    ap.add_argument("--no-swap-2opt", action="store_true",
                    help="Drop swap and 2-opt from the operator pool "
                         "(reproduces the original add/replace set).")
    ap.add_argument("--init", choices=["R1", "R2", "R3", "R4"], default="R4",
                    help="single start from this construction heuristic")
    ap.add_argument("--init-seed", type=int, default=1,
                    help="random draw for R2/R3 when --init is used")
    ap.add_argument("--disable", default="",
                    help="comma-separated operators to drop: add,replace,swap,two_opt,relocate")
    ap.add_argument("--theta", type=int, default=None,
                    help="shake threshold in proposals without an accepted move "
                         "(default THETA = 1000; huge = never)")
    ap.add_argument("--multi-start", action="store_true",
                    help="legacy restarts and elite ruin; the paper's method is single-start")
    ap.add_argument("--no-fb", action="store_true", help="ablation: skip check 1 (F_i, B_j sets)")
    ap.add_argument("--no-cascade", action="store_true", help="ablation: skip check 2 (arrival propagation); the SOCP decides")
    ap.add_argument("--no-cache", action="store_true", help="ablation: skip check 3 (evaluated routes)")
    ap.add_argument("--sweep-cap-div", type=int, default=3,
                    help="Sweep shake: escalation cap = ceil(n/div). D=3 from the Table 9 sweep.")
    ap.add_argument("--fast-sets", action="store_true",
                    help="build the four move sets with O(1) tests and propagated estimates (same sets, same weights)")
    ap.add_argument("--reorder-rcl", type=int, default=0,
                    help="L_r: offer only the L_r Swap and 2-opt moves of largest exchange value (0 = all)")
    ap.add_argument("--max-idle-shakes", type=int, default=0,
                    help="S: stop after S consecutive shakes without an improvement of R_best (0 = off)")
    ap.add_argument("--sweep", choices=["paper", "enum"], default="paper",
                    help="enum: explicit enumeration of the removals of a reference (one sweep per cons)")
    ap.add_argument("--scaled-socp", action="store_true",
                    help="solve the fixed-tour subproblem in nondimensionalized units (Presolve 0, homogeneous barrier)")
    ap.add_argument("--sweep-cap-max", type=int, default=0,
                    help="Sweep shake: absolute cap on removal length "
                         "(0 = only the divisor cap).")
    ap.add_argument("--adaptive", action="store_true",
                    help="Draw operators uniformly instead of by "
                         "success-updated adaptive weights.")
    ap.add_argument("--weights-out", default=None,
                    help="CSV trace of operator weights every 200 proposals")
    ap.add_argument("--dynamics-out", default=None,
                    help="Write a per-step (f_curr, f_best, kick) trace to "
                         "this CSV for the perturbation-dynamics figure.")
    ap.add_argument("--seed-offset", type=int, default=0,
                    help="Worker index for best-of-K parallel runs; shifts "
                         "all RNG seeds so workers explore independent basins.")
    args = ap.parse_args()

    match = [(n, p) for n, p in ALL_INSTANCES + EXPANSION_INSTANCES
             if n == args.instance]
    if not match:
        raise SystemExit(f"unknown instance: {args.instance}")
    name, path = match[0]
    tag = args.tag or f"{int(args.budget)}s"
    if args.seed_offset:
        tag = f"{tag}_w{args.seed_offset}"
    run = TimedILS(name, path, args.budget, target=args.target,
                   stop_at_target=args.stop_at_target,
                   use_relocate=args.relocate,
                   use_lb_filter=not args.no_lb_filter,
                   restart_stall=(args.restart_stall if args.multi_start
                                  else 10**12),
                   no_fb=args.no_fb, no_cascade=args.no_cascade,
                   no_cache=args.no_cache, max_iter=args.max_iter,
                   no_shake=args.no_shake, n_starts=args.starts,
                   shake_return=args.shake_return, shake_backtrack=args.shake_backtrack,
                   seed_offset=args.seed_offset,
                   escalate_kicks=args.escalate_kicks,
                   tilt_p=args.tilt,
                   ruin_mode=args.ruin_mode,
                   fixed_speed=args.fixed_speed,
                   uniform_sampling=args.uniform_sampling, no_loiter=args.no_loiter,
                   use_swap_2opt=not args.no_swap_2opt,
                   adaptive_ops=args.adaptive,
                   theta=args.theta,
                   disabled_ops=[x for x in args.disable.split(',') if x],
                   init_tour_kind=args.init,
                   init_seed=args.init_seed,
                   sweep_cap_div=args.sweep_cap_div,
                   sweep_cap_max=args.sweep_cap_max,
                   fast_sets=args.fast_sets, reorder_rcl=args.reorder_rcl,
                   max_idle_shakes=args.max_idle_shakes, sweep_enum=(args.sweep == "enum"),
                   scaled_socp=args.scaled_socp,
                   cand_mode=args.cand_weight,
                   local_search=args.local_search, paper_label=args.paper_label,
                   exhaust_shake=args.exhaust_shake, set_cache=args.set_cache,
                   route_trace=args.route_trace,
                   move_trace=args.move_trace,
                   pair_weight=args.pair_weight, rcl=args.rcl,
                   shake_schedule=args.shake_schedule, shake_hold=args.shake_hold, restart_threshold=args.restart_threshold,
                   kappa_max=args.kappa_max,
                   weights_out=args.weights_out,
                   dynamics_out=args.dynamics_out).run()
    write_outputs(run, tag)


if __name__ == "__main__":
    main()
