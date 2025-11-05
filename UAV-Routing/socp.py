

from docplex.mp.model import Model

class Solver:
    """
    Incremental and warm-start capable SOCP model for flow on a cycle-like graph.
    """

    def __init__(self, 
                 parent=None,
                 tour=None, 
                 distances=None, 
                 tour_edges=None, 
                 nodes=None, 
                 metadata=None, 
                 warm_start = None
                 ):
        """
        Initialize model once.
        :param N: list of node indices
        :param E: list of (i,j) edge tuples
        :param params: dict with keys:
                       - d[(i,j)]: demand/distance for edge
                       - s[i]: scalar offset for node
                       - v_min, v_max: scalar bounds for v
                       - I[i], gamma[i]: parameters for objective
                       - in_edges[i], out_edges[i]: adjacency lists
        """
        if parent==None:
            self.initialize_solver(nodes, tour_edges, tour, distances, metadata, warm_start)
        else:
            self.update_solver()
        

    # ------------------------------------------------------------------
    def initialize_solver(self, nodes, tour_edges, tour, distances, metadata, warm_start):
        
        self.energy_coeff = [79.85628000000003, 0.016636725000000005, 321.82080840000015, 0.009242624999999999] # we can calculate them here 
        self.nodes = nodes
        self.tour_edges = tour_edges
        self.tour = tour
        self.distances =distances
        self.warm_start = warm_start
        
        self.max_energy = metadata["max_energy"]
        self.campaign_time = metadata["campaign_time"]
        self.speed_max = metadata["speed_max"]
        self.speed_min = metadata["speed_min"]
        self.base = metadata["base"]
        
        self.model = Model(name="UAV-Tour-Speeds")
        
        self._build_initial_variables()
        self._build_initial_constraints()
        self._build_objective()
        
        
    # ------------------------------------------------------------------
    def _build_initial_variables(self):
        self.var_arrival = {node: self.model.continuous_var(lb=self.nodes[node].time_window[0], ub=self.nodes[node].time_window[1], name=f"a_{node}") for node in self.nodes}   
        self.var_speed = {edge: self.model.continuous_var(lb=self.speed_min, ub=self.speed_max, name=f"v_{edge[0]}_{edge[1]}") for edge in self.tour_edges}
        self.var_time = {edge: self.model.continuous_var(lb=0.0, name=f"v_{edge[0]}_{edge[1]}") for edge in self.tour_edges}
        self.var_y = {edge: self.model.continuous_var(lb=0.0, name=f"v_{edge[0]}_{edge[1]}") for edge in self.tour_edges}
        self.var_z = {edge: self.model.continuous_var(lb=0.0, name=f"v_{edge[0]}_{edge[1]}") for edge in self.tour_edges}
# ------------------------------------------------------------------
    def _build_initial_constraints(self):
        """Add initial node and edge constraints."""
        for edge in self.tour_edges:
            # t_ij * v_ij >= d_ij. Constraint (10)
            ct = self.model.add_constraint(self.var_time[edge] * self.var_speed[edge] >= self.distances[edge])
        
            # t_ij * t_ij >= z_ij. Constraint (9)
            self.model.add_constraint(self.var_time[edge]**2 <= self.distances[edge] * self.var_z[edge], ctname=f"t2_le_dz_{edge}")
        
            # v_ij * v_ij <= y_ij. Constraint (8) 
            self.model.add_constraint(self.var_speed[edge]**2 <= self.var_y[edge])
        
        # energy constraint. Constraint (7)
        self.model.add_constraint(sum(self.energy_coeff[0]*self.var_time[edge] 
                                      + self.energy_coeff[1]*self.distances[edge]*self.var_speed[edge] 
                                      + self.energy_coeff[2]*self.var_z[edge]/self.distances[edge]  
                                      + self.energy_coeff[3]*self.distances[edge]*self.var_y[edge] 
                                      for edge in self.tour_edges) <= self.max_energy, ctname="energy")
    
        # a_i <= a_{i-1} + t_{i(i-1)}. Constraint (5)
        for node in self.nodes and node !=(0,0):
            self.model.add_constraint(self.var_arrival[node] <= self.var_arrival[node-1]*self.var_time[(node-1,node)])
    # ------------------------------------------------------------------
    def _build_objective(self):
        """ Σ (gamma_i * a_i - gamma_i * l_i + I_i)."""
        obj_expr = self.model.sum(self.nodes[node].info_slope * self.var_arrival[node]
                                  - self.nodes[node].info_slope* self.nodes[node].time_window[0] 
                                  + self.nodes[node].info_at_lowest 
                                  for node in self.tour
                                  )
        self.model.maximize(obj_expr)


    # ------------------------------------------------------------------
    def update_solver(self, flows, warm_start):
        """
        Remove and add constraints related to changed nodes and edges in the tour.
        """
        for i in flows.nodes_out:
                self.model.remove_constraint(self.flow_constraints[i])

        # Remove old edge constraints
        for e in flow_out_edges:
            if e in self.edge_constraints:
                self.model.remove_constraint(self.edge_constraints[e])
                del self.edge_constraints[e]

        # Add updated edge constraints
        for (i, j) in flow_in_edges:
            d_ij = self.params["d"][(i, j)]
            ct = self.model.add_constraint(self.t[i, j] * self.v[i, j] >= d_ij)
            self.edge_constraints[(i, j)] = ct

        # Add updated node constraints
        for i in flow_in_nodes:
            s_i = self.params["s"][i]
            out_edges = self.params["out_edges"].get(i, [])
            in_edges = self.params["in_edges"].get(i, [])
            expr = (
                self.a[i]
                + s_i
                + self.model.sum(self.t[i, j] for j in out_edges)
                - self.model.sum(self.t[j, i] for j in in_edges)
            )
            ct = self.model.add_constraint(expr <= 0, ctname=f"flow_{i}")
            self.flow_constraints[i] = ct
        
        
        self.model.get_objective_expr().add_term(hybrid, 10)
        
        if warm_start:
            warm_start
        # Clear old solution before next solve. 
        self.model.clear_solution()


    # ------------------------------------------------------------------
    def warm_start(self, last_solution):
        """
        Set variable start values for warm start.
        last_solution: dict {var_name: value}
        """
        for var in self.model.iter_continuous_vars():
            if var.name in last_solution:
                try:
                    var.start = float(last_solution[var.name])
                except Exception:
                    pass

    # ------------------------------------------------------------------
    def solve(self, log_output=False):
        """Solve with CPLEX, return solution object."""
        sol = self.model.solve(log_output=log_output)
        if not sol:
            print("⚠️ No feasible solution found.")
        return sol

    # ------------------------------------------------------------------
    def get_solution_dict(self, sol):
        """Extract variable values to a dict for warm start reuse."""
        result = {}
        for var in self.model.iter_continuous_vars():
            result[var.name] = sol.get_value(var)
        return result


# --- Example data ---
N = [1, 2, 3, 4]
E = [(1, 2), (2, 3), (3, 4), (4, 1)]

params = {
    "d": {(i, j): 5 for (i, j) in E},
    "s": {i: 1 for i in N},
    "I": {i: 10 for i in N},
    "gamma": {i: 0.3 for i in N},
    "v_min": 0.1,
    "v_max": 5.0,
    "out_edges": {1: [2], 2: [3], 3: [4], 4: [1]},
    "in_edges": {1: [4], 2: [1], 3: [2], 4: [3]},
}

# --- Build and solve model ---
model = Solver(N, E, params)
sol = model.solve(log_output=True)
last_solution = model.get_solution_dict(sol)

# --- Update parameters and constraints ---
flow_in_nodes = [2]
flow_in_edges = [(2, 3)]
flow_out_nodes = [1]
flow_out_edges = [(1, 2)]
new_params = {
    "d": {(2, 3): 8},
    "s": {2: 0.5},
    "I": {2: 12},  # updated income parameter
    "gamma": {2: 0.4},
}

model.update_flow(flow_in_nodes, flow_in_edges, flow_out_nodes, flow_out_edges, new_params)
model.warm_start(last_solution)
sol2 = model.solve(log_output=True)
