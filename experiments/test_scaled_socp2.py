"""Which Gurobi settings make the scaled fixed-tour SOCP agree with the physical
model?  Arbiter for disagreements: the taut-string energy test (exact)."""
import sys, csv, time, random, statistics as st
sys.path.insert(0, 'experiments')
import gurobipy as gp
from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES
from run_ils_fb_cascade_demo import make_instance
from run_ils_final_scored import taut_min_energy
from uav_routing.solver.socp import Solver
from uav_routing.local_search.proposal import route_to_nx
from test_scaled_socp import solve_scaled as _ss
PATHS = dict(ALL_INSTANCES + EXPANSION_INSTANCES)

def solve_scaled(route, inst, env, params, tol=1e-6):
    import test_scaled_socp as T
    # re-implement with arbitrary params by monkeypatching robust flag handling
    g, dr = inst.graph, inst.drone
    d_norm, t_norm = inst._d_norm, inst._t_norm
    E1 = inst.calib.scaled_max_energy; eta = inst.max_energy / E1
    c0s = dr.c_0 * t_norm / E1; c1s = dr.c_1 * d_norm ** 3 / (t_norm ** 2 * E1); c2s = dr.c_2 * t_norm ** 2 / (d_norm * E1)
    vmin, vmax = inst.speed_min_s, inst.speed_max_s; depot = dr.base
    ext = route + [depot]; legs = list(zip(ext, ext[1:]))
    m = gp.Model(env=env); m.Params.OutputFlag = 0; m.Params.Threads = 1
    for k, v in params.items(): setattr(m.Params, k, v)
    a = {n: m.addVar(lb=inst.tw_scaled[n][0], ub=inst.tw_scaled[n][1]) for n in route[1:]}
    t, L, s, y, z = {}, {}, {}, {}, {}
    for e in legs:
        ds = inst.d_scaled[e]
        t[e] = m.addVar(lb=0.0); L[e] = m.addVar(lb=ds); s[e] = m.addVar(lb=0.0); y[e] = m.addVar(lb=0.0); z[e] = m.addVar(lb=0.0)
    m.update(); energy = 0
    for e in legs:
        m.addConstr(L[e] >= vmin * t[e]); m.addConstr(L[e] <= vmax * t[e])
        m.addConstr(t[e] * t[e] <= z[e] * L[e]); m.addConstr(L[e] * L[e] <= t[e] * s[e]); m.addConstr(s[e] * s[e] <= L[e] * y[e])
        i, j = e
        if i == depot and j != depot: m.addConstr(a[j] == t[e])
        elif j != depot: m.addConstr(a[j] == a[i] + t[e])
        energy += c0s * t[e] + c1s * y[e] + c2s * z[e]
    m.addConstr(energy <= eta); last = route[-1]
    m.addConstr(a[last] + t[(last, depot)] <= inst.T_max_s)
    obj = 0
    for n in route[1:]:
        gam = g.nodes[n]['info_slope']; info = g.nodes[n]['info_at_lowest']
        obj += gam * t_norm * (a[n] - inst.tw_scaled[n][0]) + info
    m.setObjective(obj - 1e-5 * (energy / eta), gp.GRB.MAXIMIZE)
    t0 = time.perf_counter(); m.optimize(); dt = time.perf_counter() - t0
    if m.SolCount == 0: return None, dt, m.BarIterCount, m.Status
    E = 0.0
    for e in legs:
        tv = t[e].X * t_norm; Lv = L[e].X * d_norm
        if tv > 0 and Lv > 0:
            v = Lv / tv; E += dr.c_0 * tv + dr.c_1 * Lv * v * v + dr.c_2 * tv / v
    return (m.ObjVal if E <= inst.max_energy * (1 + tol) else None), dt, m.BarIterCount, m.Status

COMBOS = {
    "default":        {},
    "P0":             {"Presolve": 0},
    "P0+BH":          {"Presolve": 0, "BarHomogeneous": 1},
    "P0+BH+NF3":      {"Presolve": 0, "BarHomogeneous": 1, "NumericFocus": 3},
    "BH":             {"BarHomogeneous": 1},
    "P0+BH+NF1":      {"Presolve": 0, "BarHomogeneous": 1, "NumericFocus": 1},
}
for stem, name in {"r102_100": "R102 (100)", "rc104_100": "RC104 (100)", "r101_100": "R101 (100)",
                   "pr15_240": "PR15 (240)", "c104_100": "C104 (100)"}.items():
    inst, graph, drone = make_instance(PATHS[name]); env = gp.Env(params={"OutputFlag": 0})
    rows = list(csv.DictReader(open(f'animation/traces/{stem}_route.csv')))
    routes = []; seen = set()
    for r in rows:
        rt = tuple(int(x) for x in r['route'].split('-'))
        # the 32-target cap was the cloud's pip-license limit; this machine has the
        # full license, and the long routes are the ones worth testing
        if rt not in seen: seen.add(rt); routes.append(list(rt))
    random.Random(2).shuffle(routes); routes = routes[:40]
    phys = []; taut = []
    for route in routes:
        sv = Solver(route_to_nx(route), inst, _gurobi_env=env); phys.append(sv.obj_value)
        E_ts, smin = taut_min_energy(route, graph, drone.base, drone, inst.time_horizon, drone.speed_max)
        taut.append("feas" if (smin >= 1/drone.speed_max - 1e-12 and E_ts <= inst.max_energy) else ("infeas" if (smin >= 1/drone.speed_max - 1e-12 and E_ts > inst.max_energy) else "undecided"))
    print(f"\n{name}: {len(routes)} routes; physical model feasible {sum(1 for f in phys if f is not None)}; taut string says feasible {taut.count('feas')} infeasible {taut.count('infeas')} undecided {taut.count('undecided')}")
    print(f"  physical vs taut: phys-infeasible but taut-feasible {sum(1 for f,t in zip(phys,taut) if f is None and t=='feas')}, phys-feasible but taut-infeasible {sum(1 for f,t in zip(phys,taut) if f is not None and t=='infeas')}")
    for cname, params in COMBOS.items():
        res = [solve_scaled(route, inst, env, params) for route in routes]
        agree = sum(1 for (f, _, _, _), p in zip(res, phys) if (f is None) == (p is None))
        wrong_infeas = sum(1 for (f, _, _, _), t in zip(res, taut) if f is None and t == 'feas')
        wrong_feas = sum(1 for (f, _, _, _), t in zip(res, taut) if f is not None and t == 'infeas')
        rel = [abs(f - p) / max(1.0, abs(p)) for (f, _, _, _), p in zip(res, phys) if f is not None and p is not None]
        print(f"  {cname:12s} agree with physical {agree:2d}/{len(routes)}  vs taut: false-infeasible {wrong_infeas:2d} false-feasible {wrong_feas:2d}  "
              f"|rel obj diff| max {max(rel) if rel else float('nan'):.1e}  ms/solve {1000*st.median(r[1] for r in res):5.1f}  bar-iter {st.median(r[2] for r in res):.0f}")
