"""
local_search
=============
Metaheuristic framework for UAV routing: search state management,
move operators (add, remove, swap, replace, 2-opt), acceptance criteria,
initial tour heuristics (R1-R4), and optimization algorithms (ILS, SA,
hill climbing, tilted runs).
"""

from .accept import *
from .initial_solution import *
from .iterator import *
from .optimization import *
from .proposal import *
from .state import *