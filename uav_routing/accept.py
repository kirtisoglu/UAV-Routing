
from uav_routing.state import State


def always_accept(state: State) -> bool:
    
    if state.solver.solution != None:
        #print("accepting solution:")
        #print(state.solver.solution)
        return True
    else:
        #print("rejecting solution:")
        #print(state.solver.solution)
        return False