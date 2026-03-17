"""
solver
======
Optimization solvers for UAV routing: exact MISOCP (joint node selection
and speed optimization), fixed-tour SOCP subproblem, node/arc pruning,
and experimental analysis utilities.
"""

from .exact import *
from .socp import *
from .analysis import *
