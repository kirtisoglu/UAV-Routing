"""Nondimensionalized fixed-tour SOCP against the physical-unit model of socp.py:
objective, feasibility, barrier iterations and time on routes from the traces."""
import sys, csv, time, random, statistics as st
sys.path.insert(0, 'experiments')
import gurobipy as gp
from gurobipy import GRB
from run_ils_final import ALL_INSTANCES, EXPANSION_INSTANCES
from run_ils_fb_cascade_demo import make_instance
from uav_routing.solver.socp import Solver
from uav_routing.local_search.proposal import route_to_nx
PATHS = dict(ALL_INSTANCES + EXPANSION_INSTANCES)

def solve_scaled(route, inst, env, robust=False, tiebreak=1e-5):
    """Same SOCP in the units of Environment._build_normalization: lengths / d_norm,
    times / t_norm, speeds / v_opt, energy / E1 with E1 the eta = 1 budget."""
    g, dr = inst.graph, inst.drone
    d_norm, t_norm = inst._d_norm, inst._t_norm
    E1 = inst.calib.scaled_max_energy
    eta = inst.max_energy / E1
    c0s = dr.c_0 * t_norm / E1
    c1s = dr.c_1 * d_norm ** 3 / (t_norm ** 2 * E1)
    c2s = dr.c_2 * t_norm ** 2 / (d_norm * E1)
    vmin, vmax = inst.speed_min_s, inst.speed_max_s
    depot = dr.base
    ext = route + [depot]; legs = list(zip(ext, ext[1:]))
    m = gp.Model(env=env); m.Params.OutputFlag = 0; m.Params.Threads = 1
    if robust:
        m.Params.Presolve = 0; m.Params.BarHomogeneous = 1; m.Params.NumericFocus = 3
    a = {n: m.addVar(lb=inst.tw_scaled[n][0], ub=inst.tw_scaled[n][1]) for n in route[1:]}
    t, L, s, y, z = {}, {}, {}, {}, {}
    for e in legs:
        ds = inst.d_scaled[e]
        t[e] = m.addVar(lb=0.0); L[e] = m.addVar(lb=ds); s[e] = m.addVar(lb=0.0); y[e] = m.addVar(lb=0.0); z[e] = m.addVar(lb=0.0)
    m.update()
    energy = 0
    for e in legs:
        m.addConstr(L[e] >= vmin * t[e]); m.addConstr(L[e] <= vmax * t[e])
        m.addConstr(t[e] * t[e] <= z[e] * L[e]); m.addConstr(L[e] * L[e] <= t[e] * s[e]); m.addConstr(s[e] * s[e] <= L[e] * y[e])
        i, j = e
        if i == depot and j != depot: m.addConstr(a[j] == t[e])
        elif j != depot: m.addConstr(a[j] == a[i] + t[e])
        energy += c0s * t[e] + c1s * y[e] + c2s * z[e]
    m.addConstr(energy <= eta)
    last = route[-1]
    m.addConstr(a[last] + t[(last, depot)] <= inst.T_max_s)
    obj = 0
    for n in route[1:]:
        e_i = g.nodes[n]['time_window'][0]; gam = g.nodes[n]['info_slope']; info = g.nodes[n]['info_at_lowest']
        obj += gam * t_norm * (a[n] - inst.tw_scaled[n][0]) + info
    m.setObjective(obj - tiebreak * (energy / eta), GRB.MAXIMIZE)
    t0 = time.perf_counter(); m.optimize(); dt = time.perf_counter() - t0
    if m.SolCount == 0:
        return None, dt, m.BarIterCount, m.Status
    # physical energy check, as socp.py does
    E = 0.0
    for e in legs:
        tv = t[e].X * t_norm; Lv = L[e].X * d_norm
        if tv > 0 and Lv > 0:
            v = Lv / tv; E += dr.c_0 * tv + dr.c_1 * Lv * v * v + dr.c_2 * tv / v
    feas = E <= inst.max_energy * (1 + 1e-7)
    return (m.ObjVal if feas else None), dt, m.BarIterCount, m.Status

def main():
    names = {"r101_100": "R101 (100)", "r102_100": "R102 (100)", "rc104_100": "RC104 (100)", "r101_50": "R101 (50)"}
    for stem, name in names.items():
        inst, graph, drone = make_instance(PATHS[name])
        env = gp.Env(params={"OutputFlag": 0})
        rows = list(csv.DictReader(open(f'animation/traces/{stem}_route.csv')))
        routes = []; seen = set()
        for r in rows:
            rt = tuple(int(x) for x in r['route'].split('-'))
            if rt not in seen and len(rt) - 1 <= 32:
                seen.add(rt); routes.append(list(rt))
        random.Random(1).shuffle(routes); routes = routes[:60]
        res = []
        for route in routes:
            t0 = time.perf_counter(); sv = Solver(route_to_nx(route), inst, _gurobi_env=env); t_old = time.perf_counter() - t0
            f_old = sv.obj_value; it_old = sv.model.BarIterCount
            f_new, t_new, it_new, status = solve_scaled(route, inst, env, robust=False)
            f_rob, t_rob, it_rob, _ = solve_scaled(route, inst, env, robust=True)
            res.append((len(route) - 1, f_old, f_new, f_rob, t_old, t_new, t_rob, it_old, it_new, it_rob, status))
        both = [r for r in res if r[1] is not None and r[2] is not None]
        dis = [(r[0], r[1], r[2], r[10]) for r in res if (r[1] is None) != (r[2] is None)]
        rel = [abs(r[1] - r[2]) / max(1.0, abs(r[1])) for r in both]
        relr = [abs(r[1] - r[3]) / max(1.0, abs(r[1])) for r in res if r[1] is not None and r[3] is not None]
        print(f"\n{name}: {len(res)} routes, k {min(r[0] for r in res)}..{max(r[0] for r in res)}")
        print(f"  feasibility disagreements physical vs scaled(default params): {len(dis)}  {dis[:4]}")
        print(f"  objective |rel diff| scaled-default vs physical: max {max(rel):.2e} median {st.median(rel):.2e}   scaled-robust vs physical: max {max(relr):.2e}")
        print(f"  time/solve ms: physical(robust) {1000*st.median(r[4] for r in res):.1f}  scaled(default) {1000*st.median(r[5] for r in res):.1f}  scaled(robust) {1000*st.median(r[6] for r in res):.1f}")
        print(f"  barrier iterations: physical {st.median(r[7] for r in res):.0f}  scaled(default) {st.median(r[8] for r in res):.0f}  scaled(robust) {st.median(r[9] for r in res):.0f}")
        print(f"  infeasible routes among sample: physical {sum(1 for r in res if r[1] is None)}  scaled {sum(1 for r in res if r[2] is None)}")

if __name__ == "__main__":
    main()
