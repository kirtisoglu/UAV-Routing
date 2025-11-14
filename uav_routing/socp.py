
from collections import namedtuple
from docplex.mp.model import Model

#self.idx = {n: i for i, n in enumerate(self.tour)}
class Solver: 
    
    """ Incremental, warm-start capable SOCP model for flow on a cycle-like graph. """ 
    def __init__(self, 
                 tour_nodes=None, 
                 tour_edges=None, 
                 distances=None, 
                 nodes=None, 
                 metadata=None, 
                 warm_start=None, 
                 threads=1, 
                 time_limit=None, 
                 log_output=False): 
        
        self.threads = threads 
        self.time_limit = time_limit 
        self.log_output = log_output 
        self.energy_coeff = [79.85628, 0.0166367, 321.8208, 0.0092426] 
        self.nodes = nodes 
        self.distances = distances    
        
        # TODO: no need for tour_edges, tour_nodes attributes. We don't even update them. Pass directly to the function.
        self.tour_edges = tour_edges
        self.tour_nodes = tour_nodes
        self.max_energy = metadata["max_energy"] 
        self.campaign_time = metadata["campaign_time"] 
        self.speed_max = metadata["speed_max"] 
        self.speed_min = metadata["speed_min"] 
        
        self.base = metadata["base"] 
        self.destination = metadata["destination"]
        
        self.model = Model(name="UAV-Tour-Speeds") 
        self.model.parameters.threads = threads 
        if time_limit: 
            self.model.parameters.timelimit = time_limit 

        # Variables 
        self.var_arrival = {} 
        self.var_speed = {} 
        self.var_time = {} 
        self.var_y = {} 
        self.var_z = {} 

        # Constraints 
        self.constraints = {"edges": {}, "global": {}} 

        # Objective  
        self.obj_terms = {} 
        self.obj_expr = 0

        # Energy LHS symbolic 
        self.energy_lhs = 0

        # Build initial model 
        self._build_initial_variables() 
        self._build_initial_constraints() 
        self._build_objective() 
        
        # TODO: after update_solver integration for the initial solver, warm_start should be an attribute
        if warm_start: 
            self.apply_warm_start(warm_start) 

        self.solution = self.solve() 
        self.objective_value = self.solution.get_objective_value() if self.solution else None 

    # --------------------- Build Methods --------------------- 
    def _build_initial_variables(self): 
        """
        Adds one constraint (arrival time) for each node in tour_nodes 
        and four constraints () for each edge in tour_edges.
        """
        # node variables
        for n in self.tour_nodes:
            node_dict = self.nodes[n]
            self.var_arrival[n] = self.model.continuous_var(lb=node_dict["time_window"][0], ub=node_dict["time_window"][1], name=f"a_{n}")
        # edge variables
        for e in self.tour_edges: 
            self.var_speed[e] = self.model.continuous_var(lb=self.speed_min, ub=self.speed_max, name=f"v_{e}") 
            self.var_time[e] = self.model.continuous_var(lb=0.0, name=f"t_{e}") 
            self.var_y[e] = self.model.continuous_var(lb=0.0, name=f"y_{e}") 
            self.var_z[e] = self.model.continuous_var(lb=0.0, name=f"z_{e}") 

    def _build_initial_constraints(self): 
        #TODO: we could do all this inside an update function
        m = self.model 
        
        # Edge constraints 
        for e in self.tour_edges: 
            d_ij = self.distances[e]
            #if d_ij == None:
            #    print(f"d_ij {d_ij} is None") 
            ct_tv = self.model.add_constraint(self.var_time[e]*self.var_speed[e] >= d_ij) 
            ct_t2 = self.model.add_constraint(self.var_time[e]**2 <= d_ij*self.var_z[e]) 
            ct_v2 = self.model.add_constraint(self.var_speed[e]**2 <= self.var_y[e]) 
            
            # time link
            prev, n = e
            if n != self.base:
                #print((prev,n))
                ct_tl = m.add_constraint(self.var_arrival[n] >= self.var_arrival[prev] + self.var_time[e], ctname=f"time_link_{e}") 
            else:
                ct_tl = m.add_constraint(self.var_arrival[prev] + self.var_time[e] <= self.campaign_time, ctname=f"time_link_{e}")
                
            self.constraints["edges"][e] = {"tv": ct_tv, "t2": ct_t2, "v2": ct_v2, "tl": ct_tl}
            
            self.energy_lhs += (self.energy_coeff[0]*self.var_time[e] + self.energy_coeff[1]*d_ij*self.var_speed[e]
                                + self.energy_coeff[2]*self.var_z[e]/d_ij + self.energy_coeff[3]*d_ij*self.var_y[e]) 
            
        ct_energy = m.add_constraint(self.energy_lhs <= self.max_energy, ctname="energy")          
        self.constraints["global"]["energy"] = ct_energy 


    def _build_objective(self): 
        for n in self.tour_nodes: 
            node_dict = self.nodes[n]
            term = node_dict["info_slope"]*self.var_arrival[n] - node_dict["info_slope"]*node_dict["time_window"][0] + node_dict["info_at_lowest"] 
            self.obj_terms[n] = term 
        self.obj_expr = self.model.sum(self.obj_terms.values()) 
        self.model.maximize(self.obj_expr) 

    # --------------------- Warm Start --------------------- 
    def apply_warm_start(self, last_solution: dict): 
        "remove outgoing nodes' info?"
        for var in self.model.iter_continuous_vars(): 
            val = last_solution.get(var.name) 
            if val is not None: 
                try: 
                    var.start = float(val) 
                except Exception: 
                    pass 

    # --------------------- Solve --------------------- 
    def solve(self): 
        try: 
            sol = self.model.solve(log_output=self.log_output) 
        except Exception as e: 
            print("Solver error:", e) 
            sol = None 
        if not sol: 
            print("⚠️ No feasible solution found.") 
        return sol 

    def get_solution_dict(self, sol): 
        return {var.name: sol.get_value(var) for var in self.model.iter_continuous_vars()} 

    # --------------------- Incremental Helpers --------------------- 
    def remove_edge(self, e): 
        d_ij = self.distances[e] 
        print("edge", e)
        for var in self.model.iter_continuous_vars():
            print(var.name, self.solution.get_value(var))
        
        self.energy_lhs -= (self.energy_coeff[0]*self.var_time[e] + self.energy_coeff[1]*d_ij*self.var_speed[e] 
                            + self.energy_coeff[2]*self.var_z[e]/d_ij + self.energy_coeff[3]*d_ij*self.var_y[e]) 

        for ct in self.constraints["edges"][e].values(): # remove key directly?
            self.model.remove_constraint(ct) 
        self.constraints["edges"].pop(e) 
        # remove variables   
        # TODO: why don't we keep self.variables = list(dict) 
        for vdict in [self.var_speed, self.var_time, self.var_y, self.var_z]: 
            vdict.pop(e, None) 

    def add_edge(self, e): 
        # add variables
        self.var_speed[e] = self.model.continuous_var(lb=self.speed_min, ub=self.speed_max, name=f"v_{e}") 
        self.var_time[e] = self.model.continuous_var(lb=0.0, name=f"t_{e}") 
        self.var_y[e] = self.model.continuous_var(lb=0.0, name=f"y_{e}") 
        self.var_z[e] = self.model.continuous_var(lb=0.0, name=f"z_{e}") 
        
        # add constraints
        d_ij = self.distances[e]
        ct_tv = self.model.add_constraint(self.var_time[e]*self.var_speed[e] >= d_ij) 
        ct_t2 = self.model.add_constraint(self.var_time[e]**2 <= d_ij*self.var_z[e]) 
        ct_v2 = self.model.add_constraint(self.var_speed[e]**2 <= self.var_y[e]) 
        
        # time link
        prev, n = e
        if n != self.base:
            #print((prev,n))
            ct_tl = self.model.add_constraint(self.var_arrival[n] >= self.var_arrival[prev] + self.var_time[e], ctname=f"time_link_{e}") 
        else:
            ct_tl = self.model.add_constraint(self.var_arrival[prev] + self.var_time[e]<=self.campaign_time, ctname=f"time_link_{e}")
                
        
        self.energy_lhs += (self.energy_coeff[0]*self.var_time[e] + self.energy_coeff[1]*d_ij*self.var_speed[e]
                            + self.energy_coeff[2]*self.var_z[e]/d_ij + self.energy_coeff[3]*d_ij*self.var_y[e]) 
                
        self.constraints["edges"][e] = {"tv": ct_tv, "t2": ct_t2, "v2": ct_v2, "tl": ct_tl}


    def remove_node(self, n):  
        self.model.remove(self.var_arrival[n]) 
        # TODO: why am I keeping this if I will add in manually
        self.var_arrival.pop(n,None)
        self.obj_terms.pop(n)
        
    def add_node(self, n): 
        node_dict = self.nodes[n] 
        self.var_arrival[n] = self.model.continuous_var(lb=node_dict["time_window"][0], ub=node_dict["time_window"][1], name=f"a_{n}")
        
        term = node_dict["info_slope"]*self.var_arrival[n]-node_dict["info_slope"]*node_dict["time_window"][0]+node_dict["info_at_lowest"] 
        self.obj_terms[n] = term 

    # --------------------- Incremental Update --------------------- 
    def update_solver(self, flows, warm_start=None, solve=True): 
        """
        Adding edge: four variables, three constraints (self.constraints?), energy term, tour edges
        Removing edge: six constraints (self.constraints?), four variables, energy term, tour edges
        Removing node: one constraint, obj term, one variable, tour nodes 
        Adding node: one variable, one costraint, obj term, tour nodes 
            arrival time variable (with time window bounds)
            obj func term 
            time_link constraint (includes t_ij. Add its edge variable first.)
        Update
        """
        
        for e in flows.edges_out: # energy, constraints, self.constraints, variables
            try:
                e not in self.var_speed
            except Exception:
                raise print(f"Removing failed: Model does not have any constraint for edge {e}!") 
            self.remove_edge(e)
            
        for n in flows.nodes_out:  # constraint, self.constraint, variable, self.variable, obj term ---- (t_ij removed. might be a problem)
            try:
                n not in self.var_arrival
            except Exception:
                raise print(f"Removing node failed: tour_nodes doesn't have node {n}")
            self.remove_node(n)  
            
        for n in flows.nodes_in:  # variables, obj term
            if n in self.var_arrival: 
                raise print(f"Adding node failed: node {n} is already in tour nodes") 
            self.add_node(n) 
            
        for e in flows.edges_in:  # variables, constraints, energy, self.constraints
            if e in self.var_speed: 
                raise print(f"Adding edge failed: Edge {e} is already in tour edges!")
            self.add_edge(e)
            

        # update global constraints and objective 
        # TODO: define a global checker. No need to check them again and again
        try: 
            self.constraints["global"]["energy"].set_left_expr(self.energy_lhs) 
        except Exception: 
            self.constraints["global"]["energy"] = self.model.add_constraint(self.energy_lhs <= self.max_energy) 
            
        self.obj_expr = self.model.sum(self.obj_terms.values()) 
        self.model.set_objective('max', self.obj_expr) 
        if warm_start: 
            self.apply_warm_start(warm_start) 
        if solve: 
            self.solution = self.solve() 
            self.objective_value = self.solution.get_objective_value() if self.solution else None
