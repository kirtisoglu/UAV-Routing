"""
Final design + score-biased candidate sampling within F[i] ∩ B[j] ∩ complement.

Replaces uniform random sampling of candidates with weighted sampling by

    weight(u) = I_{e_u} + max(0, γ_u * Δ_u)

which is the maximum possible reward the SOCP can extract from node u
within its time window. Higher-weight candidates are tried first.

Sampling order is constructed via the standard exponential-clock trick:
    key(u) = -log(U_u) / weight(u),   U_u ~ Uniform(0, 1]
    pick in ascending order of key
This gives a weighted-random permutation without replacement that
respects the weights (smaller key → larger weight → first to try).

Everything else (F/B precomputation, v_max cascade, segment perturbation,
hill-climbing, t_improve=10) is unchanged from run_ils_final.py.

CLI is identical: --perturbation-mode, --steps, --instance.
"""
import os, sys, random, csv, math, argparse

_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
os.chdir(_repo_root)

from collections import Counter
import matplotlib.pyplot as plt

from uav_routing.environment import Environment
from uav_routing.environment.calibration import calibrate
from uav_routing.environment.graph import Graph
from uav_routing.environment.drone import Drone
from uav_routing.local_search.initial_solution import build_R3
from uav_routing.local_search.state import State
from uav_routing.local_search.optimization import Tally
from uav_routing.local_search.proposal import perturb_state, route_to_nx

from run_ils_fb_cascade_demo import (
    assign_slopes, make_instance, precompute_FB,
    compute_epsilon, cascade_feasible_add, cascade_feasible_replace,
)
from run_ils_final import (
    R_CLASS_INSTANCES, C_CLASS_INSTANCES, ALL_INSTANCES,
    DEFAULT_ILS_STEPS, T_IMPROVE, K_REMOVE, TABU_TENURE,
    GRAPH_SEED, ILS_SEED, SORTIE_TIME, SEG_LENGTHS,
    segment_remove, apply_perturbation,
)


def candidate_weight(graph, u):
    """Maximum possible reward of visiting u.

    r_u(a_u) = I_{e_u} + γ_u (a_u - e_u) for a_u in [e_u, ℓ_u].
    Max at a_u = ℓ_u when γ_u > 0, at a_u = e_u when γ_u <= 0.
    So max reward = I_{e_u} + max(0, γ_u * Δ_u).
    """
    data = graph.nodes[u]
    e_u, l_u = data['time_window']
    info = data.get('info_at_lowest', 1.0)
    slope = data.get('info_slope', 0.0)
    return float(info) + max(0.0, float(slope) * (l_u - e_u))


def weighted_order(cands, weights):
    """Return cands in weighted-random order via the exponential-clock trick.

    For u with weight w_u, draw key_u = -log(U_u) / w_u with U_u ~ Uniform(0,1].
    Sort ascending. Smaller key ↔ larger weight ↔ appears earlier.
    """
    pairs = []
    for u, w in zip(cands, weights):
        w_eff = max(1e-12, w)
        U = random.random()
        if U <= 0:
            U = 1e-300
        key = -math.log(U) / w_eff
        pairs.append((key, u))
    pairs.sort(key=lambda p: p[0])
    return [u for _, u in pairs]


CHECK_STATS = {"fb_excluded": 0, "cascade_rejected": 0, "cascade_passed": 0}


def label_feasible_add(route, p, u, graph, depot, v_max, T_max, eps, beta):
    """Check 2 in constant time: insert u between route[p-1] and route[p].

    eps  : earliest arrival at every node of the current route (forward)
    beta : latest arrival at every node that keeps all later windows and
           the depot return reachable (backward)
    Feasible iff the earliest arrival at u fits its window and the earliest
    arrival at the successor does not exceed the successor's latest arrival.
    Equivalent to walking the propagation to the end of the route, because
    the downstream earliest arrivals are monotone in the arrival at the
    successor (Vansteenwegen et al. 2009, Wait and MaxShift).
    """
    i = route[p - 1]
    j = route[p] if p < len(route) else depot
    e_u, l_u = graph.nodes[u]['time_window']
    a_u = max(e_u, eps[i] + graph[i][u]['distance'] / v_max)
    if a_u > l_u:
        return False
    latest_j = beta[j] if j != depot else T_max
    return a_u + graph[u][j]['distance'] / v_max <= latest_j


def label_feasible_replace(route, p, u, graph, depot, v_max, T_max, eps, beta):
    """Check 2 in constant time: replace route[p] by u."""
    i = route[p - 1]
    j = route[p + 1] if p + 1 < len(route) else depot
    e_u, l_u = graph.nodes[u]['time_window']
    a_u = max(e_u, eps[i] + graph[i][u]['distance'] / v_max)
    if a_u > l_u:
        return False
    latest_j = beta[j] if j != depot else T_max
    return a_u + graph[u][j]['distance'] / v_max <= latest_j


# Candidate order for Insert and Replace. "max": the position-independent
# weight of the paper, omega_u = I_e + max(0, gamma * Delta). The ratio modes
# follow the insertion ratio of Vansteenwegen et al. (2009): omega_u^2 over
# the resource the insertion consumes at this position, in O(1) from the
# forward labels: "ratio_time" the time consumed before the successor,
# "ratio_energy" the detour energy (detour length times the minimum energy
# per meter), "ratio_both" the larger of the two normalized by T_max, E_max.
CAND_MODE = "max"
E_MAX = None
E_PER_M = None


def cand_weights(cands, graph, i, j, eps, depot, v_max, T_max):
    """Candidate weights of the previous design: the position-independent omega_u."""
    return [candidate_weight(graph, u) for u in cands]


# ---- "pair" mode: one probability assignment over all (target, slot) pairs.
# w(u, p) = best reward attainable in the slot if the pair is feasible by the
# labels and the energy floor, else 0; the move is one roulette draw over the
# pairs. Check 1 is implied by the label inequality, check 2 and the energy
# floor are the zero case; only the cache and the SOCP remain afterwards.
CURRENT_REWARDS = {}     # scheduled target -> reward in the current SOCP solution (set by the driver)
ROUTE_E_FLOOR = 0.0      # energy floor of the current route (set by the driver)
PAIR_STATS = {"pairs": 0, "feasible": 0}


PAIR_WEIGHT = "ratio"    # "rho": attainable slot reward; "ratio": rho^2 / c with c the normalized slot cost (paper)
T_MAX = None


def _slot_cost(graph, u, i, j, a_i, v_max):
    """Normalized resource consumed by placing u between i and j: the larger of
    the time consumed before the successor over T_max and the detour energy
    over E_max, floored so the ratio stays finite."""
    e_u = graph.nodes[u]['time_window'][0]
    d_iu = graph[i][u]['distance']; d_uj = graph[u][j]['distance']; d_ij = graph[i][j]['distance']
    t_cons = max(0.0, (max(e_u, a_i + d_iu / v_max) + d_uj / v_max) - (a_i + d_ij / v_max))
    e_cons = max(0.0, d_iu + d_uj - d_ij) * (E_PER_M or 0.0)
    return max(t_cons / T_MAX, e_cons / E_MAX, 1e-6)


def _weight(rho_or_gain, cost):
    if PAIR_WEIGHT == "ratio":
        return rho_or_gain * rho_or_gain / cost if rho_or_gain > 0 else 0.0
    return rho_or_gain


def _slot_reward(graph, u, i, j, a_i, latest_j, v_max):
    """(feasible, rho, detour) for u between i and j given the labels."""
    nd = graph.nodes[u]
    e_u, l_u = nd['time_window']
    d_iu = graph[i][u]['distance']; d_uj = graph[u][j]['distance']; d_ij = graph[i][j]['distance']
    a_u = max(e_u, a_i + d_iu / v_max)
    l_eff = min(l_u, latest_j - d_uj / v_max)
    if a_u > l_eff:
        return False, 0.0, 0.0
    g = float(nd.get('info_slope', 0.0)); info = float(nd.get('info_at_lowest', 1.0))
    rho = info + g * ((l_eff - e_u) if g >= 0 else (a_u - e_u))
    return True, rho, max(0.0, d_iu + d_uj - d_ij)


def taut_min_energy(route, graph, depot, drone, T_max, v_max):
    """Exact minimum energy of a fixed route over the schedules its corridor allows.

    In cumulative distance x the cumulative time a(x) must stay in a corridor:
    [e_q, l_q] at every target and [0, T_max] at the depot return. The energy of
    a leg is d * psi(t/d) with psi convex and minimal at 1/v_mr, so the cheapest
    schedule is the tightest path through the corridor -- constant slope between
    contact points, the preferred slope 1/v_mr wherever the corridor leaves room.

    Returns (E_min, min_slope); min_slope < 1/v_max means the schedule would need
    a speed above v_max, in which case the caller falls back to the SOCP.
    """
    xs = [0.0]; lo = [0.0]; hi = [0.0]; ds = []
    x = 0.0; prev = route[0]
    for node in route[1:] + [depot]:
        d = graph[prev][node]['distance']; x += d; ds.append(d); xs.append(x)
        if node == depot:
            lo.append(0.0); hi.append(T_max)
        else:
            e, l = graph.nodes[node]['time_window']; lo.append(e); hi.append(l)
        prev = node
    n = len(xs) - 1
    c0, c1, c2 = drone.c_0, drone.c_1, drone.c_2
    v_mp = (c2 / (3.0 * c1)) ** 0.25; v_mr = (c2 / c1) ** 0.25; s_star = 1.0 / v_mr
    P = lambda v: c0 + c1 * v ** 3 + c2 / v
    psi = lambda s: s * P(max(v_mp, 1.0 / s))
    slopes = []
    k = 0; y0 = 0.0
    INF = float("inf")
    while k < n:
        smin, smax, imin, imax = -INF, INF, None, None
        j = k + 1; knot = None
        while j <= n:
            dx = xs[j] - xs[k]
            slo = (lo[j] - y0) / dx; shi = (hi[j] - y0) / dx
            if slo > smax:              # cannot stay under the upper contact: bend there
                knot, s = imax, smax; break
            if shi < smin:              # cannot stay over the lower contact: bend there
                knot, s = imin, smin; break
            if slo > smin: smin, imin = slo, j
            if shi < smax: smax, imax = shi, j
            j += 1
        if knot is None:                # the end is reachable: preferred slope, clipped
            if s_star < smin:   knot, s = imin, smin
            elif s_star > smax: knot, s = imax, smax
            else:               knot, s = n, s_star
        if s <= 0.0:                    # the tube cannot be threaded: window-infeasible route
            return float("inf"), 1.0 / v_max
        for _ in range(k + 1, knot + 1):
            slopes.append(s)
        y0 += s * (xs[knot] - xs[k]); k = knot
    E = sum(d * psi(s) for d, s in zip(ds, slopes))
    return E, min(slopes)


def feasible_insert_pairs(route, complement_set, graph, depot, v_max, T_max, e_floor):
    """The feasible set F of insertions <u, p>: u between r_{p-1} and r_p passes
    the label test (earliest arrival at u <= latest arrival keeping the rest of
    the route feasible) and the energy floor. Returns [(u, p, rho)] with rho the
    best reward attainable in the slot; the weights are assigned on F afterwards."""
    eps = compute_epsilon(route, graph, depot, v_max)
    beta = compute_beta(route, graph, depot, v_max, T_max)
    F = []
    for p in range(1, len(route) + 1):
        i = route[p - 1]; j = route[p] if p < len(route) else depot
        a_i = eps[i]; latest_j = beta[j] if j != depot else T_max
        for u in complement_set:
            PAIR_STATS["pairs"] += 1
            ok, rho, detour = _slot_reward(graph, u, i, j, a_i, latest_j, v_max)
            if not ok or e_floor + E_PER_M * detour > E_MAX:
                continue
            PAIR_STATS["feasible"] += 1
            F.append((u, p, _weight(rho, _slot_cost(graph, u, i, j, a_i, v_max))))
    return F


def feasible_replace_pairs(route, complement_set, graph, depot, v_max, T_max, e_floor, cur_rewards):
    """The feasible set F of replacements <u, p>: u in place of r_p between
    r_{p-1} and r_{p+1}. Returns [(u, p, gain)] with gain = rho - reward of r_p
    in the current solution (may be <= 0; the weight is max(gain, 0))."""
    if len(route) < 2:
        return []
    eps = compute_epsilon(route, graph, depot, v_max)
    beta = compute_beta(route, graph, depot, v_max, T_max)
    F = []
    for p in range(1, len(route)):
        i = route[p - 1]; j = route[p + 1] if p + 1 < len(route) else depot
        r_p = route[p]
        a_i = eps[i]; latest_j = beta[j] if j != depot else T_max
        d_ip = graph[i][r_p]['distance'] + graph[r_p][j]['distance'] - graph[i][j]['distance']
        base = e_floor - E_PER_M * max(0.0, d_ip)     # floor of the route without r_p
        r_cur = cur_rewards.get(r_p, 0.0)
        for u in complement_set:
            PAIR_STATS["pairs"] += 1
            ok, rho, detour = _slot_reward(graph, u, i, j, a_i, latest_j, v_max)
            if not ok or base + E_PER_M * detour > E_MAX:
                continue
            PAIR_STATS["feasible"] += 1
            F.append((u, p, _weight(rho - r_cur, _slot_cost(graph, u, i, j, a_i, v_max))))
    return F


RCL = 5                  # Gunawan et al. (2015): keep the RCL best pairs of F before the roulette (0 = all of F)


def draw_pair(F, exclude=()):
    """Roulette draw over F with weights max(w, 0), skipping excluded pairs.
    With RCL > 0 only the RCL pairs of highest weight take part, as in the
    construction of Gunawan et al. (2015), who keep f = 5."""
    cand = [(w, u, p) for u, p, w in F if w > 0 and (u, p) not in exclude]
    if not cand:
        return None
    if RCL and len(cand) > RCL:
        cand.sort(reverse=True); cand = cand[:RCL]
    pairs = [(u, p) for w, u, p in cand]; ws = [w for w, u, p in cand]
    return random.choices(pairs, weights=ws, k=1)[0]


def add_pair_roulette(route, complement_set, F, B, graph, depot, v_max, T_max):
    Fs = feasible_insert_pairs(route, complement_set, graph, depot, v_max, T_max, ROUTE_E_FLOOR)
    d = draw_pair(Fs)
    if d is None:
        return None
    u, p = d
    new_route = list(route); new_route.insert(p, u)
    return new_route


def replace_pair_roulette(route, complement_set, F, B, graph, depot, v_max, T_max):
    Fs = feasible_replace_pairs(route, complement_set, graph, depot, v_max, T_max, ROUTE_E_FLOOR, CURRENT_REWARDS)
    d = draw_pair(Fs)
    if d is None:
        return None
    u, p = d
    new_route = list(route); new_route[p] = u
    return new_route


def add_fb_cascade_scored(route, complement_set, F, B, graph, depot,
                          v_max, T_max):
    eps = compute_epsilon(route, graph, depot, v_max)
    beta = compute_beta(route, graph, depot, v_max, T_max)
    edges = list(enumerate(zip(route, route[1:] + [depot])))
    random.shuffle(edges)
    for p_idx, (i, j) in edges:
        p = p_idx + 1
        cands = list((F[i] & B[j]) & complement_set)
        CHECK_STATS["fb_excluded"] += len(complement_set) - len(cands)
        if not cands: continue
        weights = cand_weights(cands, graph, i, j, eps, depot, v_max, T_max)
        ordered = weighted_order(cands, weights)
        for u in ordered:
            if label_feasible_add(route, p, u, graph, depot,
                                   v_max, T_max, eps, beta):
                CHECK_STATS["cascade_passed"] += 1
                new_route = list(route); new_route.insert(p, u)
                return new_route
            CHECK_STATS["cascade_rejected"] += 1
    return None


def compute_beta(route, graph, depot, v_max, T_max):
    """Backward cascade: latest feasible arrival at each non-depot node.

    Walks backward from the depot-return arc, propagating the latest time
    the drone can leave each node such that all downstream time windows
    are still reachable and the tour returns to the depot by T_max.
    """
    beta = {}
    if not route:
        return beta
    last = route[-1]
    if last != depot:
        d_last_depot = graph[last][depot]['distance']
        l_last = graph.nodes[last]['time_window'][1]
        beta[last] = min(l_last, T_max - d_last_depot / v_max)
    else:
        beta[depot] = T_max
    for q in range(len(route) - 2, -1, -1):
        node = route[q]
        nxt = route[q + 1]
        d = graph[node][nxt]['distance']
        if node == depot:
            beta[node] = beta[nxt] - d / v_max
            continue
        l_node = graph.nodes[node]['time_window'][1]
        beta[node] = min(l_node, beta[nxt] - d / v_max)
    return beta


def add_fb_cascade_best(route, complement_set, F, B, graph, depot,
                        v_max, T_max, alpha=1.0, push_alpha=0.0,
                        top_k=None, top_k_min=10, top_k_frac=0.20):
    """Solomon I1-style best-insertion (top-K stochastic, adaptive K).

    For each (i, j, u) triple that passes cascade feasibility, compute
    score = max_reward(u) − alpha · detour(i, u, j). Sample uniformly
    from the top-K triples.

    Default top_K adapts to feasible-pool size:
        K = max(top_k_min, top_k_frac * n_feasible)
    so the focus narrows on small instances (R101 50/100) and widens on
    larger ones (R1_2_1 200) automatically. Pass top_k explicitly to
    override.
    """
    eps = compute_epsilon(route, graph, depot, v_max)
    beta = compute_beta(route, graph, depot, v_max, T_max)
    candidates = []
    edges = list(enumerate(zip(route, route[1:] + [depot])))
    for p_idx, (i, j) in edges:
        p = p_idx + 1
        cands = (F[i] & B[j]) & complement_set
        if not cands:
            continue
        d_ij = graph[i][j]['distance']
        eps_i = eps[i]
        beta_j = beta[j] if j != depot else T_max
        # Cascade ε at j before insertion (already in eps dict if j in route,
        # else the cascade through depot).
        if j != depot:
            eps_j_before = eps[j]
            e_j = graph.nodes[j]['time_window'][0]
        else:
            eps_j_before = eps_i + d_ij / v_max
            e_j = 0.0
        for u in cands:
            if not cascade_feasible_add(route, p, u, graph, depot,
                                         v_max, T_max, eps):
                continue
            d_iu = graph[i][u]['distance']
            d_uj = graph[u][j]['distance']
            detour = d_iu + d_uj - d_ij

            # Cascade-aware reward at u
            e_u, l_u = graph.nodes[u]['time_window']
            slope = graph.nodes[u].get('info_slope', 0.0)
            info_base = graph.nodes[u].get('info_at_lowest', 1.0)
            eps_u = max(e_u, eps_i + d_iu / v_max)
            beta_u = min(l_u, beta_j - d_uj / v_max)
            if slope >= 0:
                t_arrive = max(eps_u, beta_u)  # latest feasible
            else:
                t_arrive = eps_u  # earliest
            # Clip into feasible window
            t_arrive = max(e_u, min(t_arrive, l_u))
            reward = info_base + slope * max(0.0, t_arrive - e_u)

            # Push-forward at j due to inserting u
            eps_j_after = max(e_j, eps_u + d_uj / v_max)
            push_forward = max(0.0, eps_j_after - eps_j_before)

            score = reward - alpha * detour - push_alpha * push_forward
            candidates.append((score, p, u))
    if not candidates:
        return None
    candidates.sort(key=lambda t: -t[0])
    if top_k is None:
        k = max(top_k_min, int(top_k_frac * len(candidates)))
    else:
        k = max(1, top_k)
    pool = candidates[:k]
    _, best_p, best_u = random.choice(pool)
    new_route = list(route)
    new_route.insert(best_p, best_u)
    return new_route


def replace_fb_cascade_best(route, complement_set, F, B, graph, depot,
                            v_max, T_max, alpha=1.0, push_alpha=0.0,
                            top_k=None, top_k_min=10, top_k_frac=0.20):
    """Best-insertion variant of replace (top-K stochastic, adaptive K)."""
    if len(route) < 2:
        return None
    eps = compute_epsilon(route, graph, depot, v_max)
    beta = compute_beta(route, graph, depot, v_max, T_max)
    candidates = []
    for p in range(1, len(route)):
        i = route[p-1]
        j = route[p+1] if p+1 < len(route) else depot
        old = route[p]
        cands = (F[i] & B[j]) & complement_set
        if not cands:
            continue
        d_i_old = graph[i][old]['distance']
        d_old_j = graph[old][j]['distance']
        old_detour = d_i_old + d_old_j
        eps_i = eps[i]
        beta_j = beta[j] if j != depot else T_max
        if j != depot:
            eps_j_before = eps[j]
            e_j = graph.nodes[j]['time_window'][0]
        else:
            eps_j_before = eps[old] + d_old_j / v_max
            e_j = 0.0
        # Cascade-aware reward for the OLD node at its current ε
        e_old, l_old = graph.nodes[old]['time_window']
        slope_old = graph.nodes[old].get('info_slope', 0.0)
        info_old = graph.nodes[old].get('info_at_lowest', 1.0)
        eps_old_curr = eps[old]
        beta_old_curr = beta.get(old, l_old)
        if slope_old >= 0:
            t_arr_old = max(eps_old_curr, beta_old_curr)
        else:
            t_arr_old = eps_old_curr
        t_arr_old = max(e_old, min(t_arr_old, l_old))
        old_reward = info_old + slope_old * max(0.0, t_arr_old - e_old)
        for u in cands:
            if not cascade_feasible_replace(route, p, u, graph, depot,
                                             v_max, T_max, eps):
                continue
            d_iu = graph[i][u]['distance']
            d_uj = graph[u][j]['distance']
            new_detour = d_iu + d_uj
            detour_delta = new_detour - old_detour
            e_u, l_u = graph.nodes[u]['time_window']
            slope = graph.nodes[u].get('info_slope', 0.0)
            info_base = graph.nodes[u].get('info_at_lowest', 1.0)
            eps_u = max(e_u, eps_i + d_iu / v_max)
            beta_u = min(l_u, beta_j - d_uj / v_max)
            if slope >= 0:
                t_arrive = max(eps_u, beta_u)
            else:
                t_arrive = eps_u
            t_arrive = max(e_u, min(t_arrive, l_u))
            reward_u = info_base + slope * max(0.0, t_arrive - e_u)
            reward_delta = reward_u - old_reward
            eps_j_after = max(e_j, eps_u + d_uj / v_max)
            push_forward = max(0.0, eps_j_after - eps_j_before)
            score = reward_delta - alpha * detour_delta - push_alpha * push_forward
            candidates.append((score, p, u))
    if not candidates:
        return None
    candidates.sort(key=lambda t: -t[0])
    if top_k is None:
        k = max(top_k_min, int(top_k_frac * len(candidates)))
    else:
        k = max(1, top_k)
    pool = candidates[:k]
    _, best_p, best_u = random.choice(pool)
    new_route = list(route)
    new_route[best_p] = best_u
    return new_route


def replace_fb_cascade_scored(route, complement_set, F, B, graph, depot,
                              v_max, T_max):
    if len(route) < 2: return None
    eps = compute_epsilon(route, graph, depot, v_max)
    beta = compute_beta(route, graph, depot, v_max, T_max)
    positions = list(range(1, len(route)))
    random.shuffle(positions)
    for p in positions:
        i = route[p-1]
        j = route[p+1] if p+1 < len(route) else depot
        cands = list((F[i] & B[j]) & complement_set)
        CHECK_STATS["fb_excluded"] += len(complement_set) - len(cands)
        if not cands: continue
        weights = cand_weights(cands, graph, i, j, eps, depot, v_max, T_max)
        ordered = weighted_order(cands, weights)
        for u in ordered:
            if label_feasible_replace(route, p, u, graph, depot,
                                       v_max, T_max, eps, beta):
                CHECK_STATS["cascade_passed"] += 1
                new_route = list(route); new_route[p] = u
                return new_route
            CHECK_STATS["cascade_rejected"] += 1
    return None


def remove_energy_weighted(state, depot, tabu_set):
    """Drop a node weighted by its incident-arc energy share.

    Uses the current SOCP solution to attribute per-arc energy to the two
    endpoint nodes (half each), then weights the removal probability by
    that share. Removes the node returning the most energy budget.

    Returns the new route (without the chosen node), or None if the tour
    has too few removable nodes.
    """
    sv = state.solver
    tour = list(sv.tour_nodes)
    drone = sv.drone
    if len(tour) <= 2:
        return None  # depot + 1 node: can't remove without an empty tour
    removable = [n for n in tour if n != depot and n not in tabu_set]
    if not removable:
        return None
    per_node_E = {n: 0.0 for n in removable}
    for e in sv.tour_edges:
        t = sv.var_time[e].X
        y = sv.var_y[e].X
        z = sv.var_z[e].X
        E_arc = drone.c_0 * t + drone.c_1 * y + drone.c_2 * z
        a, b = e
        if a in per_node_E: per_node_E[a] += E_arc * 0.5
        if b in per_node_E: per_node_E[b] += E_arc * 0.5
    weights = [per_node_E[u] for u in removable]
    total = sum(weights)
    if total <= 0:
        chosen = random.choice(removable)
    else:
        chosen = random.choices(removable, weights=weights, k=1)[0]
    new_route = [n for n in tour if n != chosen]
    return new_route


class TriggerSelector:
    """Phase-switching operator selector.

    Default sampling is random 50/50. When add_random_node fails (cone- or
    energy-infeasible) K times in a row, switch to replace-only for the next
    M iterations to give the search a chance to lower the tour's energy
    footprint via swaps. After M replace-only iterations, return to default
    sampling. The K counter is also reset by any successful (SOCP-feasible)
    add, since that signals the saturation has resolved.

    Maps directly to the DFS-backtrack picture: when the current subset
    cannot accept any more node (leaf of the precedence-DAG search), we
    backtrack via replace before resuming descent via add.
    """
    def __init__(self, K=10, M=20, target="replace_random_node"):
        self.K = K
        self.M = M
        self.target = target
        self.consec_add_fail = 0
        self.replace_cooldown = 0
        self.n_triggers = 0  # how many times the phase-switch fired
        self.last_forced = False  # True when last choose() returned the target

    def record(self, op, accepted: bool = False, socp_feasible: bool = False):
        if op == "add_random_node":
            if socp_feasible:
                self.consec_add_fail = 0
            else:
                self.consec_add_fail += 1

    def choose(self, operators):
        # Trigger fires its target even if not in the uniform pool — the
        # dispatcher in local_move_scored handles "remove_energy_weighted"
        # regardless of whether include_remove was set.
        if self.replace_cooldown > 0:
            self.replace_cooldown -= 1
            self.last_forced = True
            return self.target
        if self.consec_add_fail >= self.K:
            self.replace_cooldown = self.M - 1
            self.consec_add_fail = 0
            self.n_triggers += 1
            self.last_forced = True
            return self.target
        self.last_forced = False
        return random.choice(operators)


class AdaptiveSelector:
    """Per-operator sliding-window success tracker for adaptive selection.

    Weight(op) = (recent_accepts + smoothing) / (recent_attempts + smoothing).

    When add_random_node hits the energy ceiling and starts cone-rejecting,
    its weight falls; replace_random_node, which doesn't add energy, retains
    its weight and gets sampled more often -- letting the search swap nodes
    until the tour's energy footprint drops enough for add to succeed again.
    """
    def __init__(self, operators, window=100, smoothing=1.0):
        from collections import deque
        self.window = window
        self.smoothing = smoothing
        self.recent = {op: deque() for op in operators}  # 1 = accept, 0 = otherwise

    def record(self, op, accepted: bool, socp_feasible: bool = False):
        if op not in self.recent:
            return
        dq = self.recent[op]
        dq.append(1 if accepted else 0)
        if len(dq) > self.window:
            dq.popleft()

    def weight(self, op):
        dq = self.recent.get(op, None)
        if dq is None:
            return self.smoothing
        return (sum(dq) + self.smoothing) / (len(dq) + self.smoothing)

    def choose(self, operators):
        weights = [self.weight(op) for op in operators]
        total = sum(weights)
        r = random.random() * total
        cum = 0.0
        for op, w in zip(operators, weights):
            cum += w
            if r <= cum:
                return op
        return operators[-1]


def local_move_scored(state, graph, depot, v_max, T_max, F, B, tabu_set,
                      selector=None, include_remove=False,
                      best_insertion=False, best_alpha=1.0, best_top_k=10,
                      best_push_alpha=0.0):
    current = state.solver.tour_nodes
    complement_set = (set(graph.nodes) - set(current)) - tabu_set
    l = len(current)
    if l == 1:
        methods = ["add_random_node"]
    else:
        methods = ["add_random_node", "replace_random_node"]
        if include_remove and l >= 3:
            methods.append("remove_energy_weighted")
    if not complement_set and "add_random_node" in methods and not include_remove:
        return None, "no_op_full_tour"

    if selector is not None and len(methods) > 1:
        chosen_name = selector.choose(methods)
    else:
        chosen_name = random.choice(methods)
    if chosen_name == "add_random_node":
        if best_insertion:
            # best_top_k=0 (or None) triggers adaptive K = max(min, frac*n).
            top_k = best_top_k if best_top_k > 0 else None
            new_route = add_fb_cascade_best(current, complement_set, F, B,
                                            graph, depot, v_max, T_max,
                                            alpha=best_alpha,
                                            push_alpha=best_push_alpha,
                                            top_k=top_k)
        else:
            new_route = add_fb_cascade_scored(current, complement_set, F, B,
                                               graph, depot, v_max, T_max)
    elif chosen_name == "replace_random_node":
        if best_insertion:
            top_k = best_top_k if best_top_k > 0 else None
            new_route = replace_fb_cascade_best(current, complement_set, F, B,
                                                graph, depot, v_max, T_max,
                                                alpha=best_alpha,
                                                push_alpha=best_push_alpha,
                                                top_k=top_k)
        else:
            new_route = replace_fb_cascade_scored(current, complement_set, F, B,
                                                   graph, depot, v_max, T_max)
    elif chosen_name == "remove_energy_weighted":
        new_route = remove_energy_weighted(state, depot, tabu_set)
    else:
        raise RuntimeError(f"unknown: {chosen_name}")
    if new_route is None:
        return None, chosen_name
    new_state = state.flip(route_to_nx(new_route))
    new_state.last_operator = chosen_name
    return new_state, chosen_name


def _classify_infeas(proposed):
    """Map SOCP solver outcome → granular Tally outcome name."""
    sv = proposed.solver if proposed is not None else None
    if sv is None:
        return "infeas_cone", None
    fr = getattr(sv, "failure_reason", None)
    if fr == "physical_energy":
        return "infeas_energy", getattr(sv, "physical_energy", None)
    return "infeas_cone", None


def ils_run(state0, graph, depot, v_max, T_max, F, B, perturbation_mode,
            total_steps, t_improve, k_remove, tabu_tenure, tilt_p=0.0,
            E_max=None, adaptive=False, adaptive_window=100,
            trigger_K=0, trigger_M=20, trigger_target="replace_random_node",
            include_remove=False, best_insertion=False, best_alpha=1.0,
            best_top_k=10, best_push_alpha=0.0):
    best_state = state0; best_score = state0.value; current = state0
    tabu = {}; stagnation = 0
    tally = Tally(); saturated = Counter()
    f_best_trace = [(0, best_score)]; f_curr_trace = [(0, current.value)]
    perturb_iters = []
    if trigger_K > 0:
        selector = TriggerSelector(K=trigger_K, M=trigger_M,
                                    target=trigger_target)
    elif adaptive:
        ops = ["add_random_node", "replace_random_node"]
        if include_remove:
            ops.append("remove_energy_weighted")
        selector = AdaptiveSelector(ops, window=adaptive_window)
    else:
        selector = None

    for i in range(1, total_steps + 1):
        tabu = {n: e for n, e in tabu.items() if e > i}
        tour_size = sum(1 for n in current.tour.nodes if n != depot)
        if stagnation >= t_improve:
            perturbed, new_tabu = apply_perturbation(
                current, perturbation_mode, k_remove, i, tabu_tenure)
            if perturbed is None:
                tally.record_attempt("perturbation", tour_size)
                tally.record("perturbation", "saturated")
                saturated["perturbation"] += 1
            else:
                tabu.update(new_tabu)
                stagnation = 0
                tally.record_attempt("perturbation", tour_size)
                if perturbed.solver is not None and perturbed.solver.solution is not None:
                    e_used = perturbed.solver.physical_energy
                    delta = perturbed.value - current.value
                    tally.record_accept("perturbation", e_used, E_max, delta)
                    current = perturbed
                    perturb_iters.append(i)
                else:
                    kind, e_phys = _classify_infeas(perturbed)
                    if kind == "infeas_energy":
                        tally.record_infeas_energy("perturbation", e_phys, E_max)
                    else:
                        tally.record("perturbation", "infeas_cone")
        else:
            proposed, op_name = local_move_scored(
                current, graph, depot, v_max, T_max, F, B, set(tabu.keys()),
                selector=selector, include_remove=include_remove,
                best_insertion=best_insertion, best_alpha=best_alpha,
                best_top_k=best_top_k, best_push_alpha=best_push_alpha)
            if proposed is None:
                # Operator returned no candidate at all → cascade pre-filter saturated.
                tally.record_attempt(op_name, tour_size)
                tally.record(op_name, "infeas_cascade")
                saturated[op_name] += 1
                if selector is not None:
                    selector.record(op_name, accepted=False, socp_feasible=False)
            else:
                op = proposed.last_operator
                tally.record_attempt(op, tour_size)
                if proposed.solver is None or proposed.solver.solution is None:
                    kind, e_phys = _classify_infeas(proposed)
                    if kind == "infeas_energy":
                        tally.record_infeas_energy(op, e_phys, E_max)
                    else:
                        tally.record(op, "infeas_cone")
                    if selector is not None:
                        selector.record(op, accepted=False, socp_feasible=False)
                else:
                    prop_score = proposed.value
                    e_used = proposed.solver.physical_energy
                    delta = prop_score - current.value
                    forced = (selector is not None
                              and getattr(selector, "last_forced", False))
                    if prop_score >= current.value:
                        tally.record_accept(op, e_used, E_max, delta)
                        current = proposed; stagnation = 0
                        if prop_score > best_score:
                            best_state = proposed; best_score = prop_score
                        if selector is not None:
                            selector.record(op, accepted=True, socp_feasible=True)
                    elif forced:
                        # Trigger-forced backtrack (e.g. remove). Auto-accept
                        # regardless of objective — the point is to descend so
                        # subsequent add can find a different branch.
                        tally.record_accept(op, e_used, E_max, delta)
                        current = proposed
                        if selector is not None:
                            selector.record(op, accepted=True, socp_feasible=True)
                    elif tilt_p > 0.0 and random.random() < tilt_p:
                        tally.record_accept(op, e_used, E_max, delta)
                        current = proposed
                        if selector is not None:
                            selector.record(op, accepted=True, socp_feasible=True)
                    else:
                        tally.record_worse(op, e_used, E_max, delta)
                        stagnation += 1
                        if selector is not None:
                            selector.record(op, accepted=False, socp_feasible=True)
        f_best_trace.append((i, best_score))
        f_curr_trace.append((i, current.value if current.solver and current.solver.solution else None))
    return (best_state, best_score, tally, saturated,
            f_best_trace, f_curr_trace, perturb_iters)


def run_one(instance_name, instance_path, perturbation_mode, ils_steps,
            tilt_p=0.0, adaptive=False, adaptive_window=100,
            trigger_K=0, trigger_M=20,
            trigger_target="replace_random_node", include_remove=False,
            best_insertion=False, best_alpha=1.0, best_top_k=10,
            best_push_alpha=0.0):
    instance, graph, drone = make_instance(instance_path)
    init_rng = random.Random(ILS_SEED)
    init_tour = build_R3(instance, rng=init_rng)
    depot = drone.base; v_max = drone.speed_max; T_max = instance.time_horizon
    F, B = precompute_FB(graph, depot, T_max)

    random.seed(ILS_SEED)
    state0 = State.initial_state(instance, init_tour)
    init_obj = state0.value
    init_n = sum(1 for n in init_tour.nodes if n != depot)
    print(f"\n{'#'*78}\n{instance_name}  scored sampling  "
          f"perturbation_mode={perturbation_mode}  t_improve={T_IMPROVE}  "
          f"steps={ils_steps}\n{'#'*78}")
    print(f"[init] f(R_3) = {init_obj:.2f}  tour size: {init_n}  "
          f"tilt_p={tilt_p:.2f}  adaptive={adaptive}  "
          f"trigger_K={trigger_K} trigger_M={trigger_M}")

    best_state, best_score, tally, saturated, f_best, f_curr, _ = ils_run(
        state0, graph, depot, v_max, T_max, F, B, perturbation_mode,
        ils_steps, T_IMPROVE, K_REMOVE, TABU_TENURE, tilt_p=tilt_p,
        E_max=instance.max_energy,
        adaptive=adaptive, adaptive_window=adaptive_window,
        trigger_K=trigger_K, trigger_M=trigger_M,
        trigger_target=trigger_target,
        include_remove=include_remove,
        best_insertion=best_insertion, best_alpha=best_alpha,
        best_top_k=best_top_k, best_push_alpha=best_push_alpha)

    print(f"\n[final] best objective: {best_score:.2f}")
    best_size = (sum(1 for n in best_state.tour.nodes if n != depot)
                 if best_state else None)
    print(f"[final] best tour size: {best_size}")

    print(tally.report(
        label=f"\nOPERATOR TALLY ({instance_name}, scored, {ils_steps} iters)",
        E_max=instance.max_energy))

    rows = tally.summary()
    cols = ["operator", "attempts", "accepted", "worse",
            "infeas_cascade", "infeas_cone", "infeas_energy", "saturated",
            "SOCP_feas", "SOCP_feas_%"]
    total_a   = sum(r["attempts"]       for r in rows)
    total_ok  = sum(r["accepted"]       for r in rows)
    total_w   = sum(r["worse"]          for r in rows)
    total_fcas = sum(r["infeas_cascade"] for r in rows)
    total_fcon = sum(r["infeas_cone"]   for r in rows)
    total_fen  = sum(r["infeas_energy"] for r in rows)
    total_sat  = sum(r["saturated"]     for r in rows)
    feas = total_ok + total_w
    print(f"[totals] attempts={total_a} accepted={total_ok} "
          f"worse_rej={total_w} infeas_cascade={total_fcas} "
          f"infeas_cone={total_fcon} infeas_energy={total_fen} "
          f"saturated={total_sat}")
    print(f"  SOCP feasibility overall: {feas}/{total_a} = "
          f"{100*feas/max(1,total_a):.1f}%")
    print(f"  Of infeasibility: cascade={total_fcas} ({100*total_fcas/max(1, total_a-feas):.1f}%), "
          f"cone={total_fcon} ({100*total_fcon/max(1, total_a-feas):.1f}%), "
          f"energy={total_fen} ({100*total_fen/max(1, total_a-feas):.1f}%)")

    safe = instance_name.lower().replace(" ", "_").replace("(", "").replace(")", "")
    out_csv = f"experiments/ils_final_scored_{perturbation_mode}_{safe}_{ils_steps}.csv"
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            r2 = {k: r.get(k, 0) for k in cols}
            w.writerow(r2)
    print(f"Wrote {out_csv}")

    fig, ax = plt.subplots(figsize=(10, 5))
    valid = [(i, v) for i, v in f_curr if v is not None]
    if valid:
        ax.plot([x[0] for x in valid], [x[1] for x in valid],
                color='tab:orange', linewidth=1.0, alpha=0.5, label='Current obj')
    ax.plot([x[0] for x in f_best], [x[1] for x in f_best],
            color='tab:blue', linewidth=1.8, label='Best obj', zorder=5)
    ax.set_xlabel("ILS iteration")
    ax.set_ylabel("Objective")
    ax.set_title(f"Scored ILS on {instance_name}  ({ils_steps} iters)")
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)
    out_png = f"experiments/ils_final_scored_{perturbation_mode}_{safe}_{ils_steps}.png"
    fig.tight_layout()
    fig.savefig(out_png, dpi=150, bbox_inches='tight')
    print(f"Wrote {out_png}")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--perturbation-mode", choices=["segment", "random_k"],
                        default="segment")
    parser.add_argument("--steps", type=int, default=DEFAULT_ILS_STEPS)
    parser.add_argument("--tilt", type=float, default=0.0,
                        help="Probability of accepting a worse move (tilted run). "
                             "0.0 = pure hill-climbing (default). Try 0.1-0.3.")
    parser.add_argument("--adaptive", action="store_true",
                        help="Use AdaptiveSelector: sample operator weighted "
                             "by recent acceptance. Helps when add saturates "
                             "at the energy ceiling.")
    parser.add_argument("--adaptive-window", type=int, default=100,
                        help="Sliding window size for AdaptiveSelector.")
    parser.add_argument("--trigger-K", type=int, default=0,
                        help="TriggerSelector: K consecutive add SOCP-failures "
                             "fires a switch to replace-only for M iters. "
                             "0 (default) = trigger off. Try 10.")
    parser.add_argument("--trigger-M", type=int, default=20,
                        help="TriggerSelector: cooldown duration (iters) "
                             "after each fire. Default 20.")
    parser.add_argument("--trigger-target", type=str,
                        default="replace_random_node",
                        choices=["replace_random_node",
                                 "remove_energy_weighted"],
                        help="Operator to switch to when trigger fires.")
    parser.add_argument("--include-remove", action="store_true",
                        help="Add remove_energy_weighted to operator pool. "
                             "Drops a node weighted by its incident-arc "
                             "energy share -- the missing 'backtrack' "
                             "primitive for the DFS picture.")
    parser.add_argument("--best-insertion", action="store_true",
                        help="Use Solomon I1-style best-insertion: exhaustively "
                             "evaluate every (edge, candidate) pair and pick "
                             "the highest-scoring one. Replaces weighted-random "
                             "sampling within F[i] ∩ B[j] ∩ complement.")
    parser.add_argument("--best-alpha", type=float, default=1.0,
                        help="Best-insertion score = reward - alpha * detour. "
                             "Higher alpha penalises long detours.")
    parser.add_argument("--best-top-k", type=int, default=10,
                        help="Best-insertion samples uniformly from top_k "
                             "highest-scoring triples. top_k=1 is fully "
                             "deterministic (may get stuck on SOCP-infeasible "
                             "top picks). 0 = adaptive max(10, 20% of feas). "
                             "Default 10.")
    parser.add_argument("--best-push-alpha", type=float, default=0.0,
                        help="Coefficient on Solomon-style push-forward "
                             "penalty in the best-insertion score. Penalises "
                             "insertions that delay arrivals at downstream "
                             "nodes (in seconds, weighted against reward).")
    parser.add_argument("--instance", default="all",
                        help="all (default, R-class), all-c (C-class), all-six, "
                             "or a specific instance name")
    args = parser.parse_args()

    if args.instance == "all":
        targets = R_CLASS_INSTANCES
    elif args.instance == "all-c":
        targets = C_CLASS_INSTANCES
    elif args.instance == "all-six":
        targets = ALL_INSTANCES
    else:
        targets = [(name, path) for name, path in ALL_INSTANCES
                   if name == args.instance]
        if not targets:
            raise SystemExit(f"unknown instance: {args.instance}")

    for name, path in targets:
        run_one(name, path, args.perturbation_mode, args.steps,
                tilt_p=args.tilt, adaptive=args.adaptive,
                adaptive_window=args.adaptive_window,
                trigger_K=args.trigger_K, trigger_M=args.trigger_M,
                trigger_target=args.trigger_target,
                include_remove=args.include_remove,
                best_insertion=args.best_insertion,
                best_alpha=args.best_alpha, best_top_k=args.best_top_k,
                best_push_alpha=args.best_push_alpha)


if __name__ == "__main__":
    main()
