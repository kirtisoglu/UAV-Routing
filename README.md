# UAV Routing with Time Windows for Maximum Information Collection

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
- **Local Search**: SOCP-based evaluation of fixed tours combined with metaheuristics (ILS with tabu perturbation, simulated annealing, hill climbing, tilted runs)

### Calibration

Solomon benchmark instances are calibrated to real-world Bayraktar TB2 drone specifications through spatial and time scaling. Two calibration methods:

- **Energy**: Scales so the reference tour consumes the full energy budget at optimum speed
- **Campaign**: Scales so the Solomon time horizon maps to the drone's sortie duration

All solver computations use dimensionless normalized variables for numerical stability. Physical units are recovered only at result extraction.

## Project Structure

```
uav_routing/
  environment/             # Problem instance setup
    graph.py               # Complete graph from Solomon data (random/positive/negative/zero slopes)
    drone.py               # Bayraktar TB2 power model
    calibration.py         # Solomon-to-real-world scaling
    environment.py         # Normalized problem wrapper with eta energy scaling
    data.py                # Solomon format parser
    data_analysis.py       # Dataset structural analysis utilities

  solver/                  # Optimization solvers
    exact.py               # Gurobi MISOCP (full problem)
    socp.py                # Gurobi SOCP (fixed-tour speed optimization)
    prune.py               # Node/arc feasibility preprocessing
    analysis.py            # Experimental analysis (gap convergence, histograms)

  local_search/            # Metaheuristic framework
    state.py               # Search state (tour + SOCP solution)
    initial_solution.py    # Initial tour heuristics (R1-R4)
    proposal.py            # Move operators (add, remove, swap, replace, 2-opt)
    optimization.py        # ILS, SA, hill climbing, tilted runs
    iterator.py            # Markov chain iteration framework
    accept.py              # Acceptance criteria

experiments/               # Computational experiments
  exact.ipynb              # Exact solver: scalability, slopes, eta, loitering, speed/arrival analysis
  local.ipynb              # Local search: metaheuristic comparison, hyperparameters, convergence, 200-node
  calibration.ipynb        # Calibration analysis
  data.ipynb               # Dataset structural analysis
  energy.ipynb             # Energy model derivation and visualization
  run_ils_section2.py      # ILS cross-validation runner script
  initial_tour_results.csv # Initial tour heuristic results

datasets/
  data/                    # Solomon benchmark instances (50 and 100 nodes)
  homberger_200/           # Gehring & Homberger 200-node instances (c1_2_1, r1_2_1, rc1_2_1)

pyproject.toml             # Package configuration
requirements.txt           # Python dependencies
LICENSE                    # License file
```

## Parameters

| Parameter              | Description                                                                      |
| ---------------------- | -------------------------------------------------------------------------------- |
| `eta`                | Energy budget multiplier (eta=1 gives full sortie energy from calibration)       |
| `slope`              | Information dynamics:`"random"`, `"positive"`, `"negative"`, or `"zero"` |
| `drone_sortie_time`  | Mission duration in hours (default: 3h for TB2)                                  |
| `calibration_method` | `"energy"` or `"campaign"`                                                   |

## Experiments

### Dataset Analysis (`experiments/data.ipynb`)

Structural analysis of Solomon-format datasets used in experiments:

| Section                     | Description                                                |
| --------------------------- | ---------------------------------------------------------- |
| 1. Instance summary         | Nodes, depot, campaign time, spatial spread, TW stats      |
| 2. Spatial layout           | Node positions colored by information value                |
| 3. Distance vs TW           | Spearman correlation between depot distance and TW opening |
| 4. TW structure             | Timeline, width distribution, tightness ratio              |
| 5. Temporal reachability    | Which nodes are reachable under d=t assumption             |
| 6. Information distribution | Info histograms and info-distance correlation              |
| 7. Slope regimes            | Visualization of random, positive, zero slope assignments  |
| 8. Calibration effects      | Before/after campaign-time calibration comparison          |
| 9. Energy landscape         | Info vs energy cost scatter per node                       |
| 10. Difficulty indicators   | Affordable nodes %, TW overlap %, spatial CV               |
| 11. Spatial visualization   | Graph plots for 50/100-node instances                      |
| 12. 200-node instances      | Gehring & Homberger dataset visualization                  |

### Energy Model (`experiments/energy.ipynb`)

Derives and visualizes the power consumption models:

- Rotary-wing energy model (unused, included for reference)
- Fixed-wing (Bayraktar TB2) energy model: `E = c_1 d v^2 + c_2 d / v^2`
- Plots of energy per meter vs speed, with theoretical optima (max-range speed, max-endurance speed)

### Exact Solver (`experiments/exact.ipynb`)

| Section               | Description                                                  |
| --------------------- | ------------------------------------------------------------ |
| 1. Scalability        | All 5 datasets, random slopes                                |
| 2. Slope comparison   | All 5 datasets x 4 slopes (random, positive, negative, zero) |
| 3. Gap convergence    | MIP gap vs solve time by slope (from Section 2 data)         |
| 4. Eta sweep          | All 5 datasets x 5 eta values                                |
| 5. Value of loitering | Full model vs no-loiter across eta values                    |
| 6. Speed distribution | Speed histograms by eta                                      |
| 7. Arrival time       | Arrival position within time windows by slope                |
| 8. Detailed run       | Single run with full solver output                           |
| 9. Reproducibility    | 5 Gurobi seeds, same instance                                |

### Local Search (`experiments/local.ipynb`)

| Section                     | Description                                                              |
| --------------------------- | ------------------------------------------------------------------------ |
| 1. Initial tours            | R1-R4 heuristics on all 5 datasets                                       |
| 2. Cross-validation         | ILS from each initial tour on all datasets                               |
| 3. Metaheuristic comparison | ILS, SA, hill climbing, tilted on all datasets                           |
| 3b. Convergence plots       | Side-by-side convergence curves                                          |
| 3c. Perturbation dynamics   | ILS current/best objective with perturbation events                      |
| 4. Hyperparameter sweep     | k_remove, t_improve, tabu_tenure grid                                    |
| 5. SA schedule comparison   | Jump-cycle, linear-cycle, logit-cycle                                    |
| 6. ILS vs Exact             | Multi-start ILS vs exact solver (gap, time)                              |
| 7. Slope comparison         | ILS under random, positive, negative, zero slopes                        |
| 8. Eta sweep                | ILS across eta values (3-seed averaging)                                 |
| 9. Multi-start ILS          | All 4 starts per dataset, keep global best                               |
| 10. Reproducibility         | R4 (search variance) and R2 (total variance) x 5 seeds                   |
| 11. 200-node scalability    | Gehring & Homberger instances (multi-start ILS)                          |
| 12. 200-node eta sweep      | Energy sensitivity on 200-node instances (multi-start ILS, 5 eta values) |

## Drone: Bayraktar TB2

| Specification        | Value     |
| -------------------- | --------- |
| Stall speed          | 20 m/s    |
| Max speed            | 61 m/s    |
| Optimum speed        | 44.31 m/s |
| Loiter speed         | 33.5 m/s  |
| Parasitic drag (c_1) | 0.0905    |
| Induced drag (c_2)   | 349,095   |

## Datasets

**Solomon benchmarks** in `datasets/data/`:

- `50_c101.txt` -- Clustered (50 nodes)
- `50_r101.txt` -- Random (50 nodes)
- `rc101.txt` -- Mixed random/clustered (100 nodes)
- `r101.txt` -- Random (100 nodes)
- `c101.txt` -- Clustered (100 nodes)

**Gehring & Homberger** in `datasets/homberger_200/`:

- `c1_2_1.txt` -- Clustered (200 nodes)
- `r1_2_1.txt` -- Random (200 nodes)
- `rc1_2_1.txt` -- Mixed (200 nodes)

## Installation

Requires Python >= 3.9 and a valid [Gurobi license](https://www.gurobi.com/academia/academic-program-and-licenses/) (free for academics).

```bash
git clone https://github.com/kirtisoglu/UAV-Routing.git
cd UAV-Routing
pip install -e .
```

This installs all dependencies: gurobipy, networkx, numpy, scipy, pandas, matplotlib, tqdm.

To reproduce the experiments, open the notebooks and run all cells:

```bash
cd experiments
jupyter notebook
```

The notebooks are committed with outputs so results can be inspected without re-running. To reproduce on your own machine, select **Kernel → Restart & Run All**. Exact solver experiments on 100-node instances may take 10+ minutes per run depending on hardware.
