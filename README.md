# UAV Routing with Time Windows and Speed Optimization

A mixed-integer second-order cone programming (MISOCP) approach to UAV routing where a drone departs from a depot, visits a subset of nodes within time windows to maximize collected reward, and returns to the depot. Speed on each arc is a continuous decision variable, creating a joint node-selection and speed-optimization problem.

## Problem

Given a complete graph with time-windowed nodes, a depot, and a fixed-wing UAV with nonlinear power consumption `P(v) = c_0 + c_1 v^3 + c_2 / v`:

- **Select** which nodes to visit
- **Sequence** the tour starting and ending at the depot
- **Choose speeds** on each arc to minimize energy while respecting time windows
- **Maximize** total information collected (base reward + time-dependent slope)

Subject to: energy budget, campaign time horizon, speed bounds (stall to max), and node time windows.

## Approach

### SOCP Linearization

The nonlinear energy function is linearized using three rotated second-order cone constraints per arc, avoiding integer branching on speed:

1. `t^2 <= z * L` (models inverse speed)
2. `L^2 <= t * s` (models distance times speed)
3. `s^2 <= L * y` (models distance times speed squared)

This yields a linear energy expression `E = c_0 * t + c_1 * y + c_2 * z` that is exact at optimality.

### Solution Methods

- **Exact (Gurobi MISOCP)**: Joint node selection + speed optimization via mixed-integer SOCP with MTZ subtour elimination
- **Local Search**: SOCP-based evaluation of fixed tours combined with metaheuristics (simulated annealing, iterated local search with tabu, hill climbing, short bursts)

### Calibration

Solomon benchmark instances are calibrated to real-world Bayraktar TB2 drone specifications through spatial and time scaling. Two calibration methods:

- **Energy**: Scales so the reference tour consumes the full energy budget at optimum speed
- **Campaign**: Scales so the Solomon time horizon maps to the drone's sortie duration

All solver computations use dimensionless normalized variables for numerical stability. Physical units are recovered only at result extraction.

## Project Structure

```
uav_routing/
  environment/         # Problem instance setup
    graph.py           # Complete graph from Solomon data
    drone.py           # Bayraktar TB2 power model
    calibration.py     # Solomon-to-real-world scaling
    environment.py     # Normalized problem wrapper with (alpha, beta) scaling
    data.py            # Solomon format parser
    data_analysis.py   # Dataset visualization
    plot.py            # Graph and tour plotting

  solver/              # Optimization solvers
    exact.py           # Gurobi MISOCP (full problem)
    socp.py            # CPLEX SOCP (fixed-tour speed optimization)
    prune.py           # Node/arc feasibility preprocessing
    analysis.py        # Computational experiments and benchmarks

  local_search/        # Metaheuristic framework
    state.py           # Search state (tour + SOCP solution)
    proposal.py        # Move operators (add, remove, swap, replace, 2-opt)
    optimization.py    # SA, ILS, hill climbing, short bursts
    initial_tour.py    # Heuristic starting solutions
    accept.py          # Acceptance criteria

datasets/data/         # Solomon benchmark instances
paper/                 # Research paper
```

## Parameters

| Parameter | Description |
|-----------|-------------|
| `alpha`   | Energy budget multiplier (alpha=1 gives base budget from calibration) |
| `beta`    | Time horizon multiplier (beta=1 gives base campaign time) |
| `drone_sortie_time` | Mission duration in hours (default: 3h for TB2) |
| `calibration_method` | `"energy"` or `"campaign"` |

## Drone: Bayraktar TB2

| Specification | Value |
|---------------|-------|
| Stall speed   | 20 m/s |
| Max speed     | 61 m/s |
| Optimum speed | 44.31 m/s |
| Loiter speed  | 33.5 m/s |
| Parasitic drag (c_1) | 0.0905 |
| Induced drag (c_2) | 349,095 |

## Datasets

Solomon-format benchmark instances in `datasets/data/`:
- `r101.txt` -- Random customer locations (100 nodes)
- `rc101.txt` -- Mixed random/clustered (100 nodes)
- `c101.txt` -- Clustered customers (100 nodes)
- `50_r101.txt` -- Random (50 nodes)
- `50_c101.txt` -- Clustered (50 nodes)

## Dependencies

- **Gurobi** (exact solver + SOCP for local search)
- NetworkX, NumPy, SciPy, Pandas, Matplotlib, Pyvis, tqdm

## Installation

```bash
pip install -e .
```

Requires valid Gurobi and/or CPLEX licenses.
