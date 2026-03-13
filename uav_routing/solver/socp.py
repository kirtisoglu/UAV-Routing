

import gurobipy as gp
from gurobipy import GRB

from dataclasses import dataclass, field
from typing import List, Dict, Tuple


@dataclass
class TourResult:
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

    """Incremental, warm-start capable SOCP model for fixed-tour speed optimization.

    Solves in scaled space using pre-computed lookups from the Environment instance:

      - instance.d_scaled[(i,j)]  : distance / d_norm              (dimensionless)
      - instance.tw_scaled[i]     : (e_i, l_i) / t_norm            (dimensionless)
      - instance.speed_max_s      : drone.speed_max / optimum_speed (dimensionless)
      - instance.speed_min_s      : drone.speed_min / optimum_speed (dimensionless)

    Normalization is computed once in Environment._build_normalization().
    Unscaling happens once at result extraction in get_tour_data().
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
        self.model.Params.Method = 2       # Barrier (stable for SOCP)
        self.model.Params.ScaleFlag = 2    # Aggressive scaling
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
        inst = self.instance
        mdl  = self.model

        T_max_s   = inst.T_max_s
        speed_max = inst.speed_max_s
        speed_min = inst.speed_min_s

        # Tight upper bounds (same as exact.py)
        max_t = T_max_s
        max_L = speed_max * T_max_s
        max_z = T_max_s / speed_min
        max_s = speed_max * max_L
        max_y = (speed_max ** 3) * T_max_s

        # Arrival time variables — only for non-depot tour nodes
        base = self.drone.base
        for n in self.tour_nodes:
            if n == base:
                continue
            e_s, l_s = inst.tw_scaled[n]
            self.var_arrival[n] = mdl.addVar(
                lb=e_s, ub=l_s, vtype=GRB.CONTINUOUS, name=f"a_{n}")

        # Edge variables
        for e in self.tour_edges:
            d_s = inst.d_scaled[e]
            self.var_time[e]   = mdl.addVar(lb=0.0, ub=max_t, vtype=GRB.CONTINUOUS, name=f"t_{e}")
            self.var_length[e] = mdl.addVar(lb=d_s, ub=max_L, vtype=GRB.CONTINUOUS, name=f"L_{e}")
            self.var_s[e]      = mdl.addVar(lb=0.0, ub=max_s, vtype=GRB.CONTINUOUS, name=f"s_{e}")
            self.var_y[e]      = mdl.addVar(lb=0.0, ub=max_y, vtype=GRB.CONTINUOUS, name=f"y_{e}")
            self.var_z[e]      = mdl.addVar(lb=0.0, ub=max_z, vtype=GRB.CONTINUOUS, name=f"z_{e}")

        mdl.update()


    def _build_constraints(self):
        mdl  = self.model
        inst = self.instance
        base = self.drone.base

        # Energy coefficients: fixed at alpha=1, RHS = alpha
        v_opt       = self.drone.optimum_speed
        d_norm      = inst._d_norm
        base_energy = inst.calib.scaled_max_energy
        c1_n = self.drone.c_1 * v_opt**2 * d_norm / base_energy
        c2_n = self.drone.c_2 / v_opt**2 * d_norm / base_energy

        energy_lhs = 0

        for e in self.tour_edges:
            t = self.var_time[e]
            L = self.var_length[e]
            s = self.var_s[e]
            y = self.var_y[e]
            z = self.var_z[e]

            # SOCP cones (rotated second-order cones)
            mdl.addConstr(t * t <= z * L, name=f"cone1_{e}")
            mdl.addConstr(L * L <= t * s, name=f"cone2_{e}")
            mdl.addConstr(s * s <= L * y, name=f"cone3_{e}")

            # Speed bounds: L/t in [speed_min_s, speed_max_s]
            mdl.addConstr(L <= inst.speed_max_s * t, name=f"speed_ub_{e}")
            mdl.addConstr(L >= inst.speed_min_s * t, name=f"speed_lb_{e}")

            # Linking bounds (tighten the conic relaxation)
            mdl.addConstr(y <= inst.speed_max_s**3 * t, name=f"y_link_{e}")
            mdl.addConstr(z <= t / inst.speed_min_s,    name=f"z_link_{e}")

            # Arrival time linking
            prev, n = e
            if prev == base and n != base:
                # First arc from depot: a[n] = t
                mdl.addConstr(self.var_arrival[n] == t, name=f"time_link_base_{e}")
            elif n != base:
                # Interior arc: a[n] = a[prev] + t
                mdl.addConstr(self.var_arrival[n] == self.var_arrival[prev] + t,
                              name=f"time_link_{e}")
            # Return arc (n == base): no arrival linking, handled by campaign constraint

            energy_lhs += c1_n * y + c2_n * z

        # Energy budget: RHS = eta (scales with eta)
        mdl.addConstr(energy_lhs <= inst.eta, name="energy")

        # Campaign time: total tour time <= T_max_s
        # Last node's arrival + return travel time must not exceed time horizon
        last_node = self.tour_nodes[-1]
        return_edge = (last_node, base)
        if last_node != base and return_edge in self.var_time:
            mdl.addConstr(
                self.var_arrival[last_node] + self.var_time[return_edge] <= inst.T_max_s,
                name="campaign_time"
            )


    def _build_objective(self):
        inst   = self.instance
        t_norm = inst._t_norm
        base   = self.drone.base

        obj_expr = 0
        for n in self.tour_nodes:
            if n == base:
                continue
            e_s, _ = inst.tw_scaled[n]
            slope       = self.graph.nodes[n]['info_slope']
            info_lowest = self.graph.nodes[n]['info_at_lowest']
            # info = slope * t_norm * (a_scaled - e_scaled) + info_lowest
            obj_expr += slope * t_norm * (self.var_arrival[n] - e_s) + info_lowest

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
        """Extracts solver results, unscaling all values to physical units."""
        if not self.solution:
            return TourResult(sequence=self.tour_nodes, feasible=False)

        inst   = self.instance
        t_norm = inst._t_norm
        d_norm = inst._d_norm
        v_opt  = self.drone.optimum_speed
        base   = self.drone.base

        # Unscale arrivals: a_physical = a_scaled * t_norm
        arrivals = {}
        for n in self.tour_nodes:
            if n == base:
                arrivals[n] = 0.0
            else:
                arrivals[n] = self.var_arrival[n].X * t_norm

        lengths  = {}
        times    = {}
        energies = {}
        total_e  = 0

        for e in self.tour_edges:
            t_s = self.var_time[e].X
            y_s = self.var_y[e].X
            z_s = self.var_z[e].X
            L_s = self.var_length[e].X

            # Unscale to physical units
            t_val = t_s * t_norm
            L_val = L_s * d_norm
            y_val = y_s * (d_norm * v_opt**2)
            z_val = z_s * (d_norm / v_opt**2)

            lengths[e]  = L_val
            times[e]    = t_val
            e_val       = self.drone.socp_energy_function(t_val, y_val, z_val)
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
