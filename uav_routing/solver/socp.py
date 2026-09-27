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

import os
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
                 no_loiter=False,
                 scaled=None,
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
        scaled : bool, optional
            Solve the subproblem in the nondimensionalized units of
            ``Environment._build_normalization`` (lengths over ``_d_norm``, times
            over ``_t_norm``, speeds over the maximum-range speed, energy over the
            eta = 1 budget).  The coefficients then all lie within a few orders of
            magnitude, so the barrier converges in a fraction of the iterations
            and the numerical-focus setting is not needed.  Every quantity the
            solver reports is converted back to physical units.  When None, the
            attribute ``instance.socp_scaled`` decides (default False).
        _gurobi_env : gurobipy.Env, optional
            Shared Gurobi environment to avoid per-solve overhead.
        """
        self.log_output  = log_output
        self.warm_start  = warm_start
        self.threads     = threads
        self.no_loiter   = no_loiter
        self.scaled      = bool(getattr(instance, 'socp_scaled', False)) if scaled is None else bool(scaled)
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
            no_loiter=self.no_loiter,
            scaled=self.scaled,
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
        # Presolve declares feasible routes infeasible on this cone model: its
        # coefficient ranges span 1e4 to 2e8 in physical units and the
        # tolerance-based reductions prove false infeasibilities ("Barrier
        # performed 0 iterations, Model is infeasible"). Validated 2026-09-14
        # on 120 such routes with an explicit feasible schedule: presolve off
        # with the homogeneous barrier recovers all of them.
        self.model.Params.Presolve = 0
        self.model.Params.BarHomogeneous = 1
        if not self.scaled:
            # In physical units the coefficient ranges need the numerical focus;
            # in nondimensionalized units the default focus is exact and 3-5x faster.
            self.model.Params.NumericFocus = int(
                os.environ.get("UAV_NUMERIC_FOCUS",
                               globals().get("NUMERIC_FOCUS_OVERRIDE", 3)))
        # unit conversion factors (1.0 in physical units)
        if self.scaled:
            self._dn = float(self.instance._d_norm)
            self._tn = float(self.instance._t_norm)
            self._E1 = float(self.instance.calib.scaled_max_energy)
        else:
            self._dn = self._tn = self._E1 = 1.0
        if self.time_limit:
            self.model.Params.TimeLimit = self.time_limit

        self._build_variables()
        self._build_constraints()
        self._build_objective()

        if self.warm_start is True and not initial:
            self.apply_warm_start()

        self.model.optimize()

        self.failure_reason = None
        self.physical_energy = None
        if self.model.SolCount == 0:
            self.solution  = None
            self.obj_value = None
            self.failure_reason = "cone_no_solution"
        else:
            E_phys = self._compute_physical_energy()
            self.physical_energy = E_phys
            # In scaled units the cone slack on very short legs lets the exact energy
            # exceed the budget by up to ~2e-6 of E_max at the solver's tolerances
            # (measured on boundary routes); 1e-5 accepts those, 0.001% of the budget.
            if E_phys <= self.instance.max_energy * (1.0 + (1e-5 if self.scaled else 0.0)):
                self.solution  = True
                self.obj_value = self.model.ObjVal
            else:
                self.solution  = None
                self.obj_value = None
                self.failure_reason = "physical_energy"


    def _compute_physical_energy(self) -> float:
        """Compute physical energy from cone-tight formula E = c_0*t +
        c_1*L*v^2 + c_2*t/v with v = L/t. Used both to enforce the
        post-check and to expose the value for diagnostics.
        """
        drone = self.drone
        E_phys = 0.0
        for e in self.tour_edges:
            t_val = self.var_time[e].X * self._tn
            L_val = self.var_length[e].X * self._dn
            if t_val <= 0.0 or L_val <= 0.0:
                continue
            v = L_val / t_val
            E_phys += drone.c_0 * t_val + drone.c_1 * L_val * v * v + drone.c_2 * t_val / v
        return E_phys


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
            e_i, l_i = (self.instance.tw_scaled[n] if self.scaled
                        else self.graph.nodes[n]['time_window'])
            self.var_arrival[n] = mdl.addVar(
                lb=e_i, ub=l_i, name=f"a_{n}")

        # Edge variables in physical units
        for e in self.tour_edges:
            d = self.instance.d_scaled[e] if self.scaled else self.graph[e[0]][e[1]]['distance']
            self.var_time[e]   = mdl.addVar(lb=0.0, name=f"t_{e}")
            # lb = d is constraint (25); pinning ub = d forbids loitering, leaving
            # the speed within [v_min, v_max] as the only way to retime a leg
            self.var_length[e] = mdl.addVar(
                lb=d, ub=(d if self.no_loiter else float("inf")), name=f"L_{e}")   # eq 25
            self.var_s[e]      = mdl.addVar(lb=0.0, name=f"s_{e}")   # eq 30
            self.var_y[e]      = mdl.addVar(lb=0.0, name=f"y_{e}")   # eq 30
            self.var_z[e]      = mdl.addVar(lb=0.0, name=f"z_{e}")   # eq 30

        mdl.update()


    def _build_constraints(self):
        """Build constraints matching paper eq 25-30, 32-35."""
        mdl   = self.model
        drone = self.drone
        base  = drone.base
        if self.scaled:
            dn, tn, E1 = self._dn, self._tn, self._E1
            v_lo, v_hi = self.instance.speed_min_s, self.instance.speed_max_s
            c0, c1, c2 = drone.c_0 * tn / E1, drone.c_1 * dn ** 3 / (tn ** 2 * E1), drone.c_2 * tn ** 2 / (dn * E1)
            E_max = self.instance.max_energy / E1
            T_max = self.instance.T_max_s
        else:
            v_lo, v_hi = drone.speed_min, drone.speed_max
            c0, c1, c2 = drone.c_0, drone.c_1, drone.c_2
            E_max = self.instance.max_energy
            T_max = self.instance.time_horizon
        self._coef = (c0, c1, c2)

        energy_lhs = 0

        for e in self.tour_edges:
            t = self.var_time[e]
            L = self.var_length[e]
            s = self.var_s[e]
            y = self.var_y[e]
            z = self.var_z[e]

            # Speed bounds (eq 26): v_min * t <= L <= v_max * t
            mdl.addConstr(L >= v_lo * t, name=f"speed_lb_{e}")
            mdl.addConstr(L <= v_hi * t, name=f"speed_ub_{e}")

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
            energy_lhs += c0 * t + c1 * y + c2 * z

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
        """Objective (eq 31): max sum_i (gamma_i * a_i - gamma_i * e_i + I_{e_i})
        minus a tiny tie-breaker on total energy.

        The penalty matches the one in uav_routing/solver/exact.py: scaled
        by 1e-5 / E_max so its absolute magnitude is at most 1e-5 (orders
        of magnitude below any per-node info reward), but still drives the
        solver to pick the minimum-energy solution within an info-tie. Keep
        all three model files (exact.py, analysis.solve_with_gap_log, this
        one) in sync.
        """
        base  = self.drone.base
        drone = self.drone
        c0, c1, c2 = self._coef
        E_max = self.instance.max_energy / self._E1      # 1 / eta-scaled budget in the model's units

        obj_expr = 0
        for n in self.tour_nodes:
            if n == base:
                continue
            e_i  = self.instance.tw_scaled[n][0] if self.scaled else self.graph.nodes[n]['time_window'][0]
            slope       = self.graph.nodes[n]['info_slope'] * self._tn   # information per model time unit
            info_lowest = self.graph.nodes[n]['info_at_lowest']
            obj_expr += slope * (self.var_arrival[n] - e_i) + info_lowest

        energy_expr = gp.quicksum(
            c0 * self.var_time[e] +
            c1 * self.var_y[e] +
            c2 * self.var_z[e]
            for e in self.tour_edges
        )
        obj_expr = obj_expr - 0.00001 * (energy_expr / E_max)

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


    def arrival(self, n):
        """Arrival time at scheduled target n in seconds (physical units)."""
        return 0.0 if n == self.drone.base else self.var_arrival[n].X * self._tn

    def get_tour_data(self) -> TourResult:
        """Extracts solver results, converted to physical units."""
        if not self.solution:
            return TourResult(sequence=self.tour_nodes, feasible=False)

        drone = self.drone
        base  = drone.base

        arrivals = {}
        for n in self.tour_nodes:
            if n == base:
                arrivals[n] = 0.0
            else:
                arrivals[n] = self.var_arrival[n].X * self._tn

        lengths  = {}
        times    = {}
        energies = {}
        total_e  = 0

        c0, c1, c2 = self._coef
        for e in self.tour_edges:
            t_val = self.var_time[e].X
            L_val = self.var_length[e].X
            y_val = self.var_y[e].X
            z_val = self.var_z[e].X

            lengths[e]  = L_val * self._dn
            times[e]    = t_val * self._tn
            e_val       = (c0 * t_val + c1 * y_val + c2 * z_val) * self._E1   # Joules in both modes
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
