
from typing import Any, Callable, Dict, Optional, Tuple, Set, List

from uav_routing.solver.socp import Solver



class InvalidStartError(Exception):
    """Raised when initial solution is not valid."""


class State:
    """  
    A State instance represents a state in a local search iteration.
    Attributes:
    tour: 
    drone:
    graph:
    solver: 
    parent:
    """
    
    __slots__ = (
                 "parent",
                 "solver",
                 "tour",
                 "graph", 
                 "drone",
                 "is_perturbation"
                )
    default_updaters = {}
    
    def __init__(self,
                 graph=None,
                 drone=None,
                 parent=None, 
                 tour=None,
                 warm_start=False,  # will pass this to solver
                 ):
    
        
        if parent is None:
            self._first_time(graph, drone, tour, warm_start)
        else:
            self._from_parent(parent, tour)
        
    @classmethod 
    def initial_state(cls,
                        graph,
                        drone,
                        initial_tour,
                        warm_start: Optional[bool]
                        ):
        "Creates a State object using a data generator and SOCP solver."
        
        return cls(graph = graph, 
                   tour = initial_tour, 
                   drone = drone,
                   warm_start = warm_start,
                )

    def _first_time(self, graph, drone, initial_tour, warm_start):

        self.is_perturbation = False
        self.parent = None
        self.graph = graph
        self.drone = drone
        self.tour = initial_tour    

        self.solver = Solver(tour=initial_tour,
                            graph=self.graph,
                            drone=self.drone, 
                            warm_start=warm_start,
                            )

        if self.solver.solution == None:
            raise InvalidStartError("Initial solution is not feasible.")
        
        
    def _from_parent(self, parent: "State", tour):
    
        self.is_perturbation = False
        self.parent = parent
        self.tour = tour
        self.graph = parent.graph
        self.drone = parent.drone


        
        self.solver = parent.solver.copy(tour)
        

    
    # call this function to create a State instance during the intermadiate steps. 
    def flip(self, tour) -> "State":
        """
        Returns the new state obtained by changing the tour.
        :param tour: a directed cycle starting and ending the depot node.
        :returns: the new :class:`State`
        :rtype: State
        """
        return self.__class__(parent=self, tour=tour)

    def print_variables(self):
        for var in self.solver.model.iter_continuous_vars():
            return print(var.name, self.solver.solution.get_value(var))
    
    @property
    def value(self):
        return self.solver.obj_value

"""    # we call this here and in socp. Do it only once.
    def is_valid_cycle(self) -> bool:

        Ensures the tour is a single, simple directed cycle containing the base.

        # Check degrees: every node must have exactly one in-edge and one out-edge
        for n in self.tour.nodes:
            if self.tour.out_degree(n) != 1 or self.tour.in_degree(n) != 1:
                return False
        
        # Check connectivity: walking from base must hit every node in self.tour
        try:
            # We use the helper from our operations module
            ordered = self.solver._get_ordered_tour_data(self.tour)
            return len(ordered) == len(self.tour.nodes)
        except Exception:
            return False"""
        
    
        