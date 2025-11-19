
from collections import namedtuple
from docplex.mp.model import Model
from typing import Optional

class DistanceError(Exception):
    "Raised when distance d_ij == 0 for an edge (i,j)."
    
    
#self.idx = {n: i for i, n in enumerate(self.tour)}
class Solver: 
    
    """ Incremental, warm-start capable SOCP model for flow on a cycle-like graph. """ 
    def __init__(self, 
                 tour,
                 distances=None, 
                 nodes=None, 
                 metadata=None,
                 initial=True,
                 warm_start=None, 
                 threads=1, 
                 time_limit=None, 
                 log_output=False): 
        

        self.log_output = log_output 
        self.warm_start = warm_start
        self.nodes = nodes 
        self.distances = distances
        self.metadata = metadata
    
        self.energy_coeff = [79.85628, 0.0166367, 321.8208, 0.0092426] 
        self.max_energy = metadata["max_energy"] 
        self.campaign_time = metadata["campaign_time"] 
        self.speed_max = metadata["speed_max"] 
        self.speed_min = metadata["speed_min"] 
        self.base = metadata["base"] 

        
        # Model
        self.model = Model(name="UAV-Tour-Speeds") 
        self.model.parameters.threads = threads 
        if time_limit: 
            self.model.parameters.timelimit = time_limit 

        # Build and solve
        self.solve_socp(tour, initial=initial) 
    
    
    def copy(self, tour):
        """
        Returns a copy of the assignment.
        Does not duplicate the frozensets of nodes, just the parts dictionary.
        """
        return Solver(
                 tour=tour,
                 distances=self.distances.copy(), 
                 nodes=self.nodes.copy(), 
                 metadata=self.metadata.copy(),
                 initial=False,
                 warm_start=self.warm_start, 
                 log_output=self.log_output
                )

    
    
    def solve_socp(self, tour, initial):
    
        # Variables 
        self.var_arrival = {} 
        self.var_speed = {} 
        self.var_time = {} 
        self.var_y = {} 
        self.var_z = {} 

        self.tour_nodes = list(tour.nodes)
        self.tour_edges = list(tour.edges)
        
        # Build initial model 
        self._build_variables() 
        self._build_constraints() 
        self._build_objective() 

        if self.warm_start==True and initial==False:
            self.apply_warm_start() 
            
        self.solution = self.solve_model()
        self.obj_value = self.solution.get_objective_value() if self.solution else None
    
    # --------------------- Build Methods --------------------- 
    def _build_variables(self): 
        """
        Adds one constraint (arrival time) for each node in tour_nodes 
        and four constraints () for each edge in tour_edges.
        """
        # node variables
        for n in self.tour_nodes:
            node_dict = self.nodes[n]
            self.var_arrival[n] = self.model.continuous_var(lb=node_dict["time_window"][0], ub=node_dict["time_window"][1], name=str(f"a_{n}"))
            
        # edge variables
        for e in self.tour_edges: 
            self.var_speed[e] = self.model.continuous_var(lb=self.speed_min, ub=self.speed_max, name=f"v_{e}") 
            self.var_time[e] = self.model.continuous_var(lb=0.0, name=f"t_{e}") 
            self.var_y[e] = self.model.continuous_var(lb=0.0, name=f"y_{e}") 
            self.var_z[e] = self.model.continuous_var(lb=0.0, name=f"z_{e}") 

    def _build_constraints(self): 
        m=self.model
        energy_lhs = 0
        # Edge constraints 
        for e in self.tour_edges: 
            d_ij = self.distances[e]
            
            if d_ij == 0:
                raise DistanceError(f"Distance cannot be zero for edge {e}.")
            
            self.model.add_constraint(self.var_time[e]*self.var_speed[e] >= d_ij) 
            self.model.add_constraint(self.var_time[e]**2 <= d_ij*self.var_z[e]) 
            self.model.add_constraint(self.var_speed[e]**2 <= self.var_y[e]) 
            
            # time link
            prev, n = e
            if n != self.base:
                #print((prev,n))
                m.add_constraint(self.var_arrival[n] >= self.var_arrival[prev] + self.var_time[e], ctname=f"time_link_{e}") 
            else:
                m.add_constraint(self.var_arrival[prev] + self.var_time[e] <= self.campaign_time, ctname=f"time_link_{e}")
                
            energy_lhs += (self.energy_coeff[0]*self.var_time[e] + self.energy_coeff[1]*d_ij*self.var_speed[e]
                                + self.energy_coeff[2]*self.var_z[e]/d_ij + self.energy_coeff[3]*d_ij*self.var_y[e]) 
            
        m.add_constraint(energy_lhs <= self.max_energy, ctname="energy")          



    def _build_objective(self):
        obj_expr = 0 
        for n in self.tour_nodes: 
            node_dict = self.nodes[n]
            obj_expr += node_dict["info_slope"]*self.var_arrival[n] - node_dict["info_slope"]*node_dict["time_window"][0] + node_dict["info_at_lowest"] 
        self.model.maximize(obj_expr) 

    # --------------------- Solve --------------------- 
    def solve_model(self): 
        try: 
            sol = self.model.solve(log_output=self.log_output) 
        except Exception as e: 
            print("Solver error:", e) 
            sol = None 
        #if not sol: 
            #print("⚠️ No feasible solution found.") 
        return sol 

    # --------------------- Warm Start --------------------- 
    def apply_warm_start(self):
        last_solution = self.solution
        for var in self.model.iter_continuous_vars(): 
            val = last_solution.get(var.name) 
            if val is not None: 
                try: 
                    var.start = float(val) 
                except Exception: 
                    pass 

    def _get_solution_dict(self, sol): 
        return {var.name: sol.get_value(var) for var in self.model.iter_continuous_vars()} 


     