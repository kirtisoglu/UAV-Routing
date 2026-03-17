"""
accept.py
=========
Acceptance criteria for local search iterations.

Provides default acceptance functions used by the Iterator. Custom
acceptance functions (e.g. simulated annealing, tilted) are defined
in the Optimizer class.
"""

from uav_routing.local_search.state import State


def always_accept(state: State) -> bool:
    """Accept any proposed state whose SOCP subproblem has a feasible solution.

    Parameters
    ----------
    state : State
        The proposed search state.

    Returns
    -------
    bool
        True if the state has a feasible SOCP solution, False otherwise.
    """
    if state.solver.solution != None:
        return True
    else:
        return False