
from collections import namedtuple
from typing import Any, Callable, Dict, Optional, Tuple, Set, List
from dataclasses import dataclass, field

from uav_routing.data import grid_3x3
from uav_routing.socp import Solver

    
    
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
                 "complement",  # no really need for this. list(set(nodes.keys()) - set(tour.nodes))
                 "destination", 
                 "base",
                 "nodes",
                 "updaters",
                 "_cache",
                )
    default_updaters = {}
    
    def __init__(self,
                 nodes=None, # will pass this to solver
                 parent=None, 
                 tour=None,
                 complement=None,
                 updaters=None,
                 flows=None, 
                 solver_metadata=None,  # will pass this to solver
                 warm_start=False,  # will pass this to solver
                 use_default_updaters=True,
                 ):
        
        if parent is None:
            self._first_time(nodes, tour, solver_metadata, warm_start, updaters, use_default_updaters)
        else:
            self._from_parent(parent, flows, tour, complement)
        
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
        #TODO: flows uzerinden tanimla
        self.base = solver_metadata["base"]
        # TODO: no need for destination anymore. remove its attribute and its data
        self.destination = solver_metadata["destination"]
        self.nodes = nodes
        self.parent = None
        self.flows = None
       # move Flows dataclass in this module. 
       #self.flows = Flows(nodes_out=frozenset({random_node}),
       #                     nodes_in=frozenset({node_from_complement}),
       #                     edges_out=frozenset({e for e in cycle.edges if random_node in e}),
       #                     edges_in=frozenset({e for e in C.edges if node_from_complement in e})
       #                 )
        self.tour = tour    
        # TODO: change this later after removing the destination node
        self.complement = list(set(nodes.keys()) - (set(tour.nodes) - {self.destination}))
        
        # TODO: remove updaters. needs a "step" attribute
        if updaters is None:
            updaters = dict()
        if use_default_updaters:
            self.updaters = self.default_updaters
        else:
            self.updaters = {}
        self.updaters.update(updaters)
        

        try:
            self.solver = Solver(tour_nodes=list(self.tour.nodes), 
                                 tour_edges=list(self.tour.edges), 
                                 distances=self._calculate_distances(),
                                 nodes=self.nodes, 
                                 metadata=solver_metadata,
                                 warm_start=warm_start
                                )
        except:
            raise
        
        
    def _from_parent(self, parent: "State", flows: dataclass, tour, complement):
        
        self.flows = flows
        
        try:
            self.solver.update_solver(flows=self.flows)       
        except:
            Exception("Couldn't uptade the solver.")
        
        if self.solver.solution != None:
            self.parent = parent
            self.base = parent.base
            self.destination = parent.destination
            self.updaters = parent.updaters
            self.nodes = parent.nodes
            self.solver = parent.solver
            
            self.tour = tour
            self.complement = complement
        else:
            self = parent

    
    # call this function to create a State instance during the intermadiate steps. 
    def flip(self, flows, tour, complement) -> "State":
        """
        Returns the new state obtained by performing the given `flows`
        on this state.

        :param flows: dictionary assigning ...
        :returns: the new :class:`State`
        :rtype: State
        """
        return self.__class__(parent=self, flows=flows, tour=tour, complement=complement)

    def print_variables(self):
        for var in self.solver.model.iter_continuous_vars():
            return print(var.name, self.solver.solution.get_value(var))
    
    @property
    def value(self):
        return self.solver.objective_value
    
    def __getitem__(self, key: str) -> Any:
        """
        Allows accessing the values of updaters computed for this
        Partition instance.

        :param key: Property to access.
        :type key: str

        :returns: The value of the updater.
        :rtype: Any
        """
        if key not in self._cache:
            self._cache[key] = self.updaters[key](self)
        return self._cache[key]
    def __getattr__(self, key):
        return self[key]
    def keys(self):
        return self.updaters.keys()
    
    
    def _calculate_distances(self):
        import math
        distances = {}
        
        for u in self.nodes:
            for v in self.nodes:
                if u!=v:
                    point1, point2 = self.nodes[u]["position"], self.nodes[v]["position"]
                    distances[(u,v)] = math.dist(point1, point2)
                    # TODO: remove this later by changing the way we access the distance of a directed edge
                    distances[(v,u)] = distances[(u,v)]
        return distances
    


            
            