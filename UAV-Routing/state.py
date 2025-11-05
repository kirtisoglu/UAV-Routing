
from collections import namedtuple
from typing import Any, Callable, Dict, Optional, Tuple, Set, List
from dataclasses import dataclass, field

from .data import grid_3x3, generate_random_data
from .socp import solve_tour, get_solver, Solver


@dataclass
class Flows:
    nodes_out: set = field(default_factory=set)
    nodes_in: set = field(default_factory=set)
    edges_out: set = field(default_factory=set)
    edges_in: set = field(default_factory=set)
    
    
    
class State:
    """  
    A State instance represents a state in a local search iteration.
    Attributes:
    tour: a nonrepitative sequence of nodes of length >= 1
    flips: 
    """
    
    __slots__ = (
                 "parent",
                 "tour", # needed for operations
                 "unused_nodes",  # needed for operations
                 "pos_unused",  # needed for operations
                 "flows",  # used for updating solver
                 "solver", # 
                )
    
    def __init__(self,
                 nodes=None, # will pass this to solver
                 parent=None, 
                 tour=None,  
                 flows=None, 
                 solver_metadata=None,  # will pass this to solver
                 warm_start=None,  # will pass this to solver
                 ):
        
        if parent is None:
            self._first_time(nodes, tour, solver_metadata, warm_start)
        else:
            self._from_parent(parent, flows)
            

    # solve if solver is given. otherwise, return it without solving
    @classmethod 
    def initial_state(cls,
                      data: Callable = grid_3x3,
                      warm_start: Optional[bool] = False
                      ):
        "Creates a State object by using data generator and SOCP solver methods."
        nodes, tour, metadata = data()
        
        return cls(nodes=nodes, tour=tour, solver_metadata = metadata, warm_start=warm_start)
    
            
    def _first_time(self, nodes, tour, solver_metadata, warm_start):
        
        self.parent = None
        self.tour = tour        
        self.flows = Flows()
        self.unused_nodes = list(set(nodes.keys()) | set(tour)) #  
        self.pos_in_unused = {x: self.unused_nodes.index(x) for x in self.unused_nodes} 
        
        self.solver = Solver(tour=tour, 
                             distances=self._calculate_distances(),
                             edges=self._tour_edges(), # produce it in solver? we use it for only model initialization
                             nodes=nodes, 
                             metadata=solver_metadata,
                             warm_start=warm_start) 
    
                 
    def _from_parent(self, parent: "State", flows: Dict):
        
        self.parent = parent
        self.tour = parent.tour
        
        self.unused_nodes
        self.pos_in_unused
        self.flows = Flows()
        
        self.solver = parent.solver
        self.solver.update_model(flows)        
        
        return
    
    # call this function to create a State instance in the intermadiate steps. Then from_parent() function will be called by __init__()
    def flip(self, flows: Dict) -> "State":
        """
        Returns the new state obtained by performing the given `flows`
        on this state.

        :param flows: dictionary assigning ...
        :returns: the new :class:`State`
        :rtype: State
        """
        return self.__class__(parent=self, flows=flows)
    
    
    def _calculate_distances(self):
        import math
        distances = {}
        nodes = list(self.nodes.keys())
        for u,v in zip(nodes, nodes):
            point1, point2 = self.nodes[u].position, self.nodes[v].position
            distances[(u,v)] = math.dist(point1, point2)
        return distances
    
    def _tour_edges(self) -> List[Tuple]:
        return list({(i, i+1) for i in self.tour[0:-1]} + {(self.base, self.tour[0]), (self.tour[-1], 0)})
        
        
            
            