

from docplex.mp.model import Model

from dataclasses import dataclass, field
from typing import List, Dict, Tuple
import copy

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
    
    """ Incremental, warm-start capable SOCP model for flow on a cycle-like graph. """ 
    def __init__(self, 
                 tour,
                 graph, 
                 drone,
                 initial=True,
                 warm_start=None, 
                 threads=1, 
                 time_limit=None, 
                 log_output=False): 
        

        self.log_output = log_output 
        self.warm_start = warm_start
        self.graph = graph 
        self.drone = drone

        # Model
        self.model = Model(name="UAV-Tour-Speeds") 
        
        # Force single-threading for perfect reproducibility
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
                 graph=self.graph.copy(), 
                 drone=copy.deepcopy(self.drone),
                 initial=False,
                 warm_start=self.warm_start, 
                 log_output=self.log_output
                )

    
    
    def solve_socp(self, tour, initial):
    
        self.tour_nodes, self.tour_edges = self._get_ordered_tour_data(tour)
        
        # Variables 
        self.var_arrival = {} 
        self.var_time = {} 
        self.var_length = {}
        self.var_s = {}
        self.var_y = {} 
        self.var_z = {} 
        
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
            self.var_arrival[n] = self.model.continuous_var(lb=self.graph.nodes[n]["time_window"][0], ub=self.graph.nodes[n]["time_window"][1], name=str(f"a_{n}"))
            
        # edge variables
        for e in self.tour_edges:  
            self.var_time[e] = self.model.continuous_var(lb=0.0, name=f"t_{e}") 
            self.var_length[e] = self.model.continuous_var(lb=self.graph.edges[e]['distance'], name=f"L_{e}")
            self.var_s[e] = self.model.continuous_var(lb=0.0, name=f"s_{e}")
            self.var_y[e] = self.model.continuous_var(lb=0.0, name=f"y_{e}") 
            self.var_z[e] = self.model.continuous_var(lb=0.0, name=f"z_{e}") 

    def _build_constraints(self): 
        m=self.model
        energy_lhs = 0
        # Edge constraints 
        for e in self.tour_edges: 
            
            self.model.add_constraint(self.var_time[e]**2 <= self.var_z[e]*self.var_length[e]) 
            self.model.add_constraint(self.var_length[e]**2 <= self.var_time[e]*self.var_s[e]) 
            self.model.add_constraint(self.var_s[e]**2 <= self.var_length[e]*self.var_y[e]) 
            
            # flight length
            d_ij = self.graph.edges[e]['distance']
            m.add_constraint(self.var_length[e] <= self.var_time[e]*self.drone.speed_max, ctname=f"length_upper_{e}")
            m.add_constraint(self.var_time[e]*self.drone.speed_min <= self.var_length[e], ctname=f"length_lower_{e}")
            
            # time link
            prev, n = e
            if prev != self.drone.base:
                #print((prev,n))
                m.add_constraint(self.var_arrival[n] == self.var_arrival[prev] + self.var_time[e], ctname=f"time_link_{e}") 
            else:
                m.add_constraint(self.var_arrival[n] == self.var_time[e], ctname=f"time_link_base_{e}")
                       
            energy_lhs += self.drone.socp_energy_function(self.var_time[e], self.var_y[e], self.var_z[e])
        
        m.add_constraint(energy_lhs <= self.drone.max_energy, ctname="energy")          


    def _build_objective(self):
        obj_expr = 0 
        for n in self.tour_nodes: 
            obj_expr += self.graph.nodes[n]['info_slope']*(self.var_arrival[n] - self.graph.nodes[n]["time_window"][0]) + self.graph.nodes[n]["info_at_lowest"] 
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



    def _get_ordered_tour_data(self, tour_graph):
        base_id = self.drone.base
        num_nodes = len(tour_graph.nodes)
        
        # Handle the minimal case: Base only (should not happen in ILS, but for safety)
        if num_nodes == 1:
            return [base_id], []

        ordered_nodes = [base_id]
        current = base_id
        
        # We walk (num_nodes - 1) times to find all intermediate nodes
        for _ in range(num_nodes - 1):
            successors = list(tour_graph.successors(current))
            if not successors:
                raise ValueError(f"Node {current} has no successor. Graph is broken.")
            current = successors[0]
            ordered_nodes.append(current)
            
        # Reconstruct edges chronologically
        ordered_edges = []
        for i in range(len(ordered_nodes) - 1):
            ordered_edges.append((ordered_nodes[i], ordered_nodes[i+1]))
        
        # Close the loop: Final Node -> Base
        ordered_edges.append((ordered_nodes[-1], base_id))
        
        return ordered_nodes, ordered_edges


    def _get_solution_dict(self, sol): 
        return {var.name: sol.get_value(var) for var in self.model.iter_continuous_vars()} 



    def get_failure_reason(self):
        """Diagnoses why a tour failed."""
        # 1. Check if Euclidean distance already exceeds battery
        total_dist = sum(self.graph.edges[e]['distance'] for e in self.tour_edges)
        # Assuming drone.max_dist_at_optimal_speed is a pre-calculated float
        if total_dist > self.drone.max_distance():
            return "Physical_Distance_Too_Long"

        # 2. Check if Time Windows are logically impossible 
        # (e.g., distance between i and j takes longer than the gap between windows)
        for e in self.tour_edges:
            u, v = e
            dist = self.graph.edges[e]['distance']
            min_time = dist / self.drone.speed_max
            
            # If arriving at u at its earliest + travel time > latest window of v
            if self.graph.nodes[u]["time_window"][0] + min_time > self.graph.nodes[v]["time_window"][1]:
                return f"Time_Window_Violation"

        # 3. If the solver ran but failed
        solve_status = self.model.get_solve_status()
        return str(solve_status)
    
    
    
    def get_tour_data(self) -> TourResult:
        base = self.drone.base
        """Extracts solver results into a clean dataclass."""
        if not self.solution:
            return TourResult(sequence=self.tour_nodes, feasible=False)

        # Extract node data
        arrivals = {n: self.solution.get_value(self.var_arrival[n]) 
                    for n in self.tour_nodes}
        
        # Extract edge data
        lengths = {}
        times = {}
        loiter_energies = {}
        total_e = 0
        fixed_e = 0
        
        for e in self.tour_edges:
            t_val = self.solution.get_value(self.var_time[e])
            y_val = self.solution.get_value(self.var_y[e])
            z_val = self.solution.get_value(self.var_z[e])
            L_val = self.solution.get_value(self.var_length[e])
            
            lengths[e] = L_val
            times[e] = t_val
            # Calculate energy for this specific arc using your formula
            e_val = self.drone.socp_energy_function(t_val, y_val, z_val)
            loiter_energies[e] = e_val
            total_e += e_val
            
            #if e[1] == base:
            #    v = max(self.)
            #    last_edge_energy = 
                

        return TourResult(
            sequence=self.tour_nodes, # Or a custom sequence if nodes aren't in order
            feasible=True,
            objective=self.obj_value,
            total_energy=total_e,
            arrival_times=arrivals,
            lengths=lengths,
            energies=loiter_energies,
            times=times
        )
    
     #v = max(d / (t_max - t_cikis), v_opt)