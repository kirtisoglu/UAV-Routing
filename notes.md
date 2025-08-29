# Coding Plan

1. **Design a model, and code it via CPLEX.**
2. Generate a frozen complete graph, where each node holds

- coordinates
- info_max $(I_i^{max})$
- info_factor $(\alpha_i)$

Each edge will have attributes

- distance $(d_{ij})$

3. Generate a dummy data with a tour. It should have only one optimal solution for the model.
4. Define a Route class.

- A property for exact solution
- parent attribute
- First_time, from_parent, and from_random functions

1. Test all functions that have been defined so far.
2. Operations
3. A chain class
4. An optimizer class

A route is a nonrepeatative sequence of node ids that starts and ends at the same node

Route -> sequence,
