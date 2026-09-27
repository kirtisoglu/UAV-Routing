"""Shared instance construction for every paper run in this folder.

One definition, used by all drivers, so a result can never come from a
different protocol than the one the README documents.

Protocol (fixed for the whole paper):
  * slope realization s = 1 (GRAPH_SEED), sampled per Eq. (gamma-sampling):
    sign ~ U{-1,+1} and |gamma_i| ~ U(0.5 I_e/Delta, 1.0 I_e/Delta)
  * sortie time 3 h, calibration method "campaign"
  * eta scales the energy budget only; the mission horizon is unchanged
"""
import os, sys, random

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)
os.chdir(REPO)

from uav_routing.environment import Environment
from uav_routing.environment.calibration import calibrate
from uav_routing.environment.graph import Graph
from uav_routing.environment.drone import Drone

GRAPH_SEED  = 1        # slope realization s = 1
GUROBI_SEED = 42
SORTIE_TIME = 3.0      # hours
CALIBRATION = "campaign"

R_CLASS = {
    "R101 (50)":    "datasets/data/50_r101.txt",
    "R101 (100)":   "datasets/data/r101.txt",
    "R1_2_1 (200)": "datasets/homberger_200/r1_2_1.txt",
}


def assign_slopes(graph, seed=GRAPH_SEED, lo=0.5, hi=1.0):
    rng = random.Random(seed)
    for node_id, data in graph.nodes(data=True):
        delta_t = data['time_window'][1] - data['time_window'][0]
        if delta_t == 0:
            graph.nodes[node_id]['info_slope'] = 0.0
            continue
        bound = data['info_at_lowest'] / delta_t
        sign = rng.choice([-1.0, 1.0])
        graph.nodes[node_id]['info_slope'] = sign * rng.uniform(lo * bound, hi * bound)


# The four slope regimes of Section 5.2. "mixed" is the default sampling of
# Eq. (gamma-sampling); the other three are the built-in graph modes, whose
# distributions match the paper: growth ~ U(0, I_e/Delta), decay ~
# U(-I_e/Delta, 0), static = 0.
REGIMES = {"mixed": "zero", "growth": "positive",
           "decay": "negative", "static": "zero"}


def make_instance(path, eta=1.0, seed=GRAPH_SEED, regime="mixed"):
    """Build one calibrated instance at the given energy scaling and regime.

    `seed` is the slope realization s of the paper's tables; s = 1 is the
    default used by every experiment that does not sweep it.
    """
    assert regime in REGIMES, f"unknown slope regime {regime!r}"
    g = Graph(path=path, slope=REGIMES[regime], seed=seed)
    if regime == "mixed":
        assign_slopes(g, seed=seed)
    drone = Drone(base=g.graph['base'])
    calib = calibrate(graph=g, tour_length=len(g.nodes) // 2, drone=drone,
                      drone_sortie_time=SORTIE_TIME,
                      calibration_method=CALIBRATION)
    inst = Environment(calib, drone, eta=eta)
    # eta must scale the energy budget and nothing else.
    assert abs(inst.max_energy - calib.scaled_max_energy * eta) < 1e-6 * max(1.0, inst.max_energy), \
        "eta is not scaling the energy budget as documented"
    # Return the calibrated graph, whose distances are metres and whose time
    # windows are seconds, the same graph the SOCP and every feasibility test
    # use. The raw benchmark graph `g` carries Solomon coordinate units and
    # must never reach a distance, time or energy computation; it is identical
    # to the calibrated one in node set, slopes and depot. This matches
    # experiments/run_ils_fb_cascade_demo.make_instance.
    return inst, inst.graph, drone


def leg_records(res, drone, instance):
    """Per-leg schedule of a solved tour, in physical units."""
    v_max = drone.speed_max
    out = []
    for (i, j), ad in res["arc_data"].items():
        e_leg = drone.socp_energy_function(ad["t"], ad["y"], ad["z"])
        out.append({
            "i": i, "j": j, "t_s": ad["t"], "t_lo_s": ad["d"] / v_max,
            "L_m": ad["L"], "d_m": ad["d"], "loiter_m": ad["L"] - ad["d"],
            "v_ms": ad["L"] / ad["t"] if ad["t"] > 0 else None,
            "energy_J": e_leg,
            "energy_pct": 100.0 * e_leg / instance.max_energy,
            "arrival_s": res["arrival_times"].get(j),
            "tw": list(ad["tw"]),
        })
    return sorted(out, key=lambda r: (r["arrival_s"] is None, r["arrival_s"] or 0.0))
