# exact/socp_test.py  (QCP-friendly epigraph form)
from docplex.mp.model import Model
import numpy as np


A = np.array([[2.0, -1.0, 0.5],
              [0.0,  1.0, 1.0]])
b = np.array([0.5, -1.0])

m = Model(name="socp_epigraph_qcp")

# variables
x = m.continuous_var_list(3, name="x")
s = m.continuous_var(name="s", lb=0)   # epigraph of ||Ax+b||^2

# y = A x + b
y = [m.sum(A[i, j]*x[j] for j in range(3)) + float(b[i]) for i in range(2)]

# convex QCP: ||y||^2 <= s   (quadratic <= linear)
m.add_constraint(m.sum(yi*yi for yi in y) <= s)

# optional linear bounds
for j in range(3):
    m.add_constraint(-10 <= x[j])
    m.add_constraint(x[j] <= 10)

# objective: minimize s  (equivalent minimizers to minimizing t with ||y|| <= t)
m.minimize(s)

# good settings for continuous QCP
m.context.cplex_parameters.barrier.crossover = 0  # 0=off
m.context.cplex_parameters.lpmethod = 4           # barrier
m.context.cplex_parameters.qpmethod = 4           # barrier

sol = m.solve(log_output=True)
print("status:", m.get_solve_status())
print("obj s  :", sol[s])                     # this is ||Ax+b||^2 at optimum
print("t ( = sqrt(s) ) ~", sol[s]**0.5)      # the corresponding norm value
print("x      :", [sol[xj] for xj in x])
