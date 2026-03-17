"""
socp.py
=======
Fixed-tour SOCP subproblem solver (Paper Section 5.1, eq 31-35).

Given a fixed tour (node sequence), optimizes speeds, travel times, and
path lengths to maximize information collection subject to energy, time
window, and campaign time constraints. Works in physical units (seconds,
meters, Joules) with no normalization.

Used both standalone (for evaluating candidate tours in local search)
and as a feasibility checker in the initial tour heuristics.
"""

import gurobipy as gp
from gurobipy import GRB

from dataclasses import dataclass, field
from typing import List, Dict, Tuple


@dataclass
class TourResult:
    """Container for SOCP solution data.

    Attributes
    ----------
    sequence : list of int
        Ordered node IDs in the tour.
    feasible : bool
        Whether the SOCP found a feasible solution.
    objective : float
        Total information collected.
    total_energy : float
        Total energy consumed in Joules.
    arrival_times : dict
        Node ID -> arrival time in seconds.
    lengths : dict
        (i,j) -> path length in meters.
    energies : dict
        (i,j) -> energy consumed on arc in Joules.
    times : dict
        (i,j) -> travel time on arc in seconds.
    """
    sequence: List[int]        # e.g., [0, 1, 5, 3, 2]
    feasible: bool
    objective: float = 0.0
    total_energy: float = 0.0

    # Dictionaries mapping edge (i, j) or node i to values
    arrival_times: Dict[int, float] = field(default_factory=dict)
    lengths: Dict[Tuple[int, int], float] = field(default_factory=dict)
    energies: Dict[Tuple[int, int], float] = field(default_factory=dict)
    times: Dict[Tuple[int, int], float] = field(default_factory=dict)


class Solver:

    """Fixed-tour SOCP subproblem (Paper Section 5.1, eq 31-35).

    Given a fixed tour, optimizes speeds (equivalently, travel times and path
    lengths) to maximize information collection subject to energy, time window,
    and campaign time constraints.

    Works in physical units (seconds, meters, Joules) — no normalization.
    The SOCP is a continuous convex program solved by Gurobi's barrier method.
    Unlike the MISOCP, tight variable upper bounds are not needed and would
    harm barrier convergence.
    """

    def __init__(self,
                 tour,
                 instance,
                 initial=True,
                 warm_start=None,
                 threads=1,
                 time_limit=None,
                 log_output=False,
                 _gurobi_env=None):
        """Initialize and solve the SOCP for a given tour.

        Parameters
        ----------
        tour : nx.DiGraph
            Directed cycle representing the tour.
        instance : Environment
            Calibrated problem instance.
        initial : bool
            If True, this is the first solve (no warm start applied).
        warm_start : bool, optional
            If True, seed variable start values from the previous solution.
        threads : int
            Number of Gurobi threads (default 1 for local search).
        time_limit : float, optional
            Gurobi time limit in seconds.
        log_output : bool
            If True, enable Gurobi console output.
        _gurobi_env : gurobipy.Env, optional
            Shared Gurobi environment to avoid per-solve overhead.
        """
        self.log_output  = log_output
        self.warm_start  = warm_start
        self.threads     = threads
        self.time_limit  = time_limit
        self.instance    = instance
        self.graph       = instance.graph
        self.drone       = instance.drone

        # Share a single Gurobi environment across copies to avoid overhead
        if _gurobi_env is not None:
            self._gurobi_env = _gurobi_env
        else:
            self._gurobi_env = gp.Env(params={"OutputFlag": 0})

        self.solve_socp(tour, initial=initial)


    def copy(self, tour):
        """Create a new Solver for a different tour, sharing the Gurobi environment."""
        return Solver(
            tour=tour,
            instance=self.instance,
            initial=False,
            warm_start=self.warm_start,
            threads=self.threads,
            time_limit=self.time_limit,
            log_output=self.log_output,
            _gurobi_env=self._gurobi_env,
        )


    def solve_socp(self, tour, initial):
        """Build and solve the SOCP model for the given tour."""
        self.tour_nodes, self.tour_edges = self._get_ordered_tour_data(tour)

        self.var_arrival = {}
        self.var_time    = {}
        self.var_length  = {}
        self.var_s       = {}
        self.var_y       = {}
        self.var_z       = {}

        self.model = gp.Model("UAV-Tour-Speeds", env=self._gurobi_env)

        if not self.log_output:
            self.model.Params.OutputFlag = 0
        self.model.Params.Threads = self.threads
        if self.time_limit:
            self.model.Params.TimeLimit = self.time_limit

        self._build_variables()
        self._build_constraints()
        self._build_objective()

        if self.warm_start is True and not initial:
            self.apply_warm_start()

        self.model.optimize()

        if self.model.SolCount > 0:
            self.solution  = True
            self.obj_value = self.model.ObjVal
        else:
            self.solution  = None
            self.obj_value = None


    # --------------------- Build Methods ---------------------

    def _build_variables(self):
        """Create decision variables in physical units.

        Arrival times a_i bounded by time windows (eq 27).
        Edge variables: t, L >= 0 with L >= d (eq 25, 30).
        Auxiliary variables s, y, z >= 0 only (eq 30) — no tight upper bounds,
        which is critical for barrier convergence in the continuous SOCP.
        """
        mdl  = self.model
        base = self.drone.base

        # Arrival time variables — time window bounds (eq 27)
        for n in self.tour_nodes:
            if n == base:
                continue
            e_i, l_i = self.graph.nodes[n]['time_window']
            self.var_arrival[n] = mdl.addVar(
                lb=e_i, ub=l_i, name=f"a_{n}")

        # Edge variables in physical units
        for e in self.tour_edges:
            d = self.graph[e[0]][e[1]]['distance']
            self.var_time[e]   = mdl.addVar(lb=0.0, name=f"t_{e}")
            self.var_length[e] = mdl.addVar(lb=d,   name=f"L_{e}")   # eq 25
            self.var_s[e]      = mdl.addVar(lb=0.0, name=f"s_{e}")   # eq 30
            self.var_y[e]      = mdl.addVar(lb=0.0, name=f"y_{e}")   # eq 30
            self.var_z[e]      = mdl.addVar(lb=0.0, name=f"z_{e}")   # eq 30

        mdl.update()


    def _build_constraints(self):
        """Build constraints matching paper eq 25-30, 32-35."""
        mdl   = self.model
        drone = self.drone
        base  = drone.base
        E_max = self.instance.max_energy
        T_max = self.instance.time_horizon

        energy_lhs = 0

        for e in self.tour_edges:
            t = self.var_time[e]
            L = self.var_length[e]
            s = self.var_s[e]
            y = self.var_y[e]
            z = self.var_z[e]

            # Speed bounds (eq 26): v_min * t <= L <= v_max * t
            mdl.addConstr(L >= drone.speed_min * t, name=f"speed_lb_{e}")
            mdl.addConstr(L <= drone.speed_max * t, name=f"speed_ub_{e}")

            # Rotated SOC cones (eq 33-35): t² <= z·L, L² <= t·s, s² <= L·y
            mdl.addConstr(t * t <= z * L, name=f"cone1_{e}")
            mdl.addConstr(L * L <= t * s, name=f"cone2_{e}")
            mdl.addConstr(s * s <= L * y, name=f"cone3_{e}")

            # Arrival time linking (eq 28-29)
            prev, n = e
            if prev == base and n != base:
                mdl.addConstr(self.var_arrival[n] == t, name=f"arr_base_{e}")
            elif n != base:
                mdl.addConstr(
                    self.var_arrival[n] == self.var_arrival[prev] + t,
                    name=f"arr_{e}")

            # Energy (eq 32): sum(c_0*t + c_1*y + c_2*z) <= E_max
            energy_lhs += drone.c_0 * t + drone.c_1 * y + drone.c_2 * z

        mdl.addConstr(energy_lhs <= E_max, name="energy")

        # Campaign time: last arrival + return travel <= T_max
        last_node = self.tour_nodes[-1]
        return_edge = (last_node, base)
        if last_node != base and return_edge in self.var_time:
            mdl.addConstr(
                self.var_arrival[last_node] + self.var_time[return_edge] <= T_max,
                name="campaign_time"
            )


    def _build_objective(self):
        """Objective (eq 31): max sum_i (gamma_i * a_i - gamma_i * e_i + I_{e_i})."""
        base = self.drone.base

        obj_expr = 0
        for n in self.tour_nodes:
            if n == base:
                continue
            e_i  = self.graph.nodes[n]['time_window'][0]
            slope      = self.graph.nodes[n]['info_slope']
            info_lowest = self.graph.nodes[n]['info_at_lowest']
            obj_expr += slope * (self.var_arrival[n] - e_i) + info_lowest

        self.model.setObjective(obj_expr, GRB.MAXIMIZE)


    # --------------------- Warm Start ---------------------

    def apply_warm_start(self):
        """Seeds variable start values from the previous solution."""
        for var_dict in [self.var_arrival, self.var_time, self.var_length,
                         self.var_s, self.var_y, self.var_z]:
            for var in var_dict.values():
                try:
                    var.Start = var.X
                except Exception:
                    pass


    # --------------------- Utilities ---------------------

    def _get_ordered_tour_data(self, tour_graph):
        """Extract ordered node and edge sequences from a directed cycle.

        Parameters
        ----------
        tour_graph : nx.DiGraph
            Directed cycle starting at the depot.

        Returns
        -------
        tuple of (list, list)
            (ordered_nodes, ordered_edges) where edges include the return arc.
        """
        base_id   = self.drone.base
        num_nodes = len(tour_graph.nodes)

        if num_nodes == 1:
            return [base_id], []

        ordered_nodes = [base_id]
        current = base_id

        for _ in range(num_nodes - 1):
            successors = list(tour_graph.successors(current))
            if not successors:
                raise ValueError(f"Node {current} has no successor. Graph is broken.")
            current = successors[0]
            ordered_nodes.append(current)

        ordered_edges = [(ordered_nodes[i], ordered_nodes[i + 1])
                         for i in range(len(ordered_nodes) - 1)]
        ordered_edges.append((ordered_nodes[-1], base_id))

        return ordered_nodes, ordered_edges


    def get_failure_reason(self):
        """Diagnoses why a tour failed."""
        total_dist = sum(self.graph.edges[e]['distance'] for e in self.tour_edges)
        if total_dist > self.instance.max_distance:
            return "Physical_Distance_Too_Long"

        for e in self.tour_edges:
            u, v = e
            dist     = self.graph.edges[e]['distance']
            min_time = dist / self.drone.speed_max
            if self.graph.nodes[u]["time_window"][0] + min_time > self.graph.nodes[v]["time_window"][1]:
                return "Time_Window_Violation"

        status = self.model.Status
        status_map = {
            GRB.INFEASIBLE: "Infeasible",
            GRB.INF_OR_UNBD: "Infeasible_or_Unbounded",
            GRB.UNBOUNDED: "Unbounded",
            GRB.TIME_LIMIT: "Time_Limit",
        }
        return status_map.get(status, f"Gurobi_Status_{status}")


    def get_tour_data(self) -> TourResult:
        """Extracts solver results. All values are already in physical units."""
        if not self.solution:
            return TourResult(sequence=self.tour_nodes, feasible=False)

        drone = self.drone
        base  = drone.base

        arrivals = {}
        for n in self.tour_nodes:
            if n == base:
                arrivals[n] = 0.0
            else:
                arrivals[n] = self.var_arrival[n].X

        lengths  = {}
        times    = {}
        energies = {}
        total_e  = 0

        for e in self.tour_edges:
            t_val = self.var_time[e].X
            L_val = self.var_length[e].X
            y_val = self.var_y[e].X
            z_val = self.var_z[e].X

            lengths[e]  = L_val
            times[e]    = t_val
            e_val       = drone.socp_energy_function(t_val, y_val, z_val)
            energies[e] = e_val
            total_e    += e_val

        return TourResult(
            sequence=self.tour_nodes,
            feasible=True,
            objective=self.obj_value,
            total_energy=total_e,
            arrival_times=arrivals,
            lengths=lengths,
            energies=energies,
            times=times,
        )
