
from collections import namedtuple
from typing import Any, Callable, Dict, Optional, Tuple, Set, List
from dataclasses import dataclass, field

from uav_routing.data import grid_3x3
from uav_routing.socp import Solver


class InvalidStartError(Exception):
    """Raised when initial solution is not valid."""

class DistanceError(Exception):
    "Raised when distance d_ij == 0 for an edge (i,j)."
    
class State:
    """  
    A State instance represents a state in a local search iteration.
    Attributes:
    tour_nodes: a nonrepitative sequence of nodes of length >= 1
    flips: 
    """
    
    __slots__ = (
                 "parent",
                 "flows",  # used for updating solver
                 "solver",
                 "tour",
                 "base",
                 "nodes",
                 "solution",
                 "updaters",
                 "_cache",
                )
    default_updaters = {}
    
    def __init__(self,
                 nodes=None, # will pass this to solver
                 parent=None, 
                 tour=None,
                 updaters=None,
                 flows=None, 
                 solver_metadata=None,  # will pass this to solver
                 warm_start=False,  # will pass this to solver
                 use_default_updaters=True,
                 ):
        
        if parent is None:
            self._first_time(nodes, tour, solver_metadata, warm_start, updaters, use_default_updaters)
        else:
            self._from_parent(parent, flows, tour)
        
        self._cache = dict()
        
    @classmethod 
    def initial_state(cls,
                        data: Optional[Callable] = grid_3x3,
                        warm_start: Optional[bool] = False,
                        updaters: Optional[Dict[str, Callable]] = None,
                        use_default_updaters: bool = True,
                        ):
        "Creates a State object using a data generator and SOCP solver."
        nodes, cycle, metadata = data()
        
        return cls(nodes=nodes, 
                   tour=cycle, 
                   solver_metadata = metadata, 
                   warm_start=warm_start,
                   updaters=updaters,
                   use_default_updaters=use_default_updaters,
                )

    def _first_time(self, nodes, tour, solver_metadata, warm_start, updaters, use_default_updaters):

        self.base = solver_metadata["base"]
        # NOTE: no need for destination anymore. remove its attribute and its data
        self.nodes = nodes
        self.parent = None
        self.flows = None
        self.tour = tour    

        
        # TODO: remove updaters. needs a "step" attribute
        if updaters is None:
            updaters = dict()
        if use_default_updaters:
            self.updaters = self.default_updaters
        else:
            self.updaters = {}
        self.updaters.update(updaters)
        
        self.solver = Solver(tour=tour,
                            distances=self._calculate_distances(),
                            nodes=self.nodes, 
                            metadata=solver_metadata,
                            warm_start=warm_start,
                            )

        if self.solver.solution == None:
            raise InvalidStartError("Initial solution is not feasible.")
        
        
    def _from_parent(self, parent: "State", flows: dataclass, tour):
    
        self.parent = parent
        self.base = parent.base
        self.updaters = parent.updaters
        self.nodes = parent.nodes
         
        self.tour = tour
        self.flows = flows
        self.solver = parent.solver.copy(tour)
        

    
    # call this function to create a State instance during the intermadiate steps. 
    def flip(self, flows, tour) -> "State":
        """
        Returns the new state obtained by performing the given `flows`
        on this state.

        :param flows: dictionary assigning ...
        :returns: the new :class:`State`
        :rtype: State
        """
        return self.__class__(parent=self, flows=flows, tour=tour)

    def print_variables(self):
        for var in self.solver.model.iter_continuous_vars():
            return print(var.name, self.solver.solution.get_value(var))
    
    @property
    def value(self):
        return self.solver.obj_value
    

    
    def _calculate_distances(self):
        import math
        distances = {}
        
        for u in self.nodes:
            for v in self.nodes:
                if u!=v:
                    point1, point2 = self.nodes[u]["position"], self.nodes[v]["position"]
                    distance = math.dist(point1, point2)
                    if distance == 0:
                        raise DistanceError(f"Distance cannot be zero for edge {(u,v)}.")
                    
                    distances[(u,v)] = distance
                    # TODO: remove this later by changing the way we access the distance of a directed edge
                    distances[(v,u)] = distances[(u,v)]
        return distances
    


            
            