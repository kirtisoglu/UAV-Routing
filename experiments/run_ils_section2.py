"""
Run ILS Section 2: Effect of Initial Tour on Final Solution.
Loads saved initial tours from initial_tour_results.csv and runs ILS from each.
"""
import sys
import os
import time
import ast
import random
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from uav_routing.environment import Environment
from uav_routing.environment.calibration import calibrate
from uav_routing.environment.graph import Graph, directed_cycle
from uav_routing.environment.drone import Drone
from uav_routing.local_search.state import State
from uav_routing.local_search.optimization import Optimizer
from uav_routing.local_search.proposal import universal_proposal
from uav_routing.solver.socp import Solver

# ILS parameters (same as notebook c2code cell)
ILS_STEPS = 200
ILS_T_IMPROVE = 30
ILS_K_REMOVE = 2
ILS_TABU_TENURE = 10

SORTIE_TIME = 3.0
SEED = 42

datasets = {
    "C101 (50)":  "../datasets/data/50_c101.txt",
    "R101 (50)":  "../datasets/data/50_r101.txt",
    "RC101 (100)": "../datasets/data/rc101.txt",
    "R101 (100)": "../datasets/data/r101.txt",
    "C101 (100)": "../datasets/data/c101.txt",
}


def make_instance(path, slope='random', graph_seed=1, eta=1.0):
    graph = Graph(path=path, slope=slope, seed=graph_seed)
    uav = Drone(base=graph.graph['base'])
    calib = calibrate(
        graph=graph,
        tour_length=len(graph.nodes) // 2,
        drone=uav,
        drone_sortie_time=SORTIE_TIME,
        calibration_method="campaign",
    )
    return Environment(calib, uav, eta=eta), graph, uav


def main():
    random.seed(SEED)

    # Load saved tours
    csv_path = os.path.join(os.path.dirname(__file__), "initial_tour_results.csv")
    df_tours = pd.read_csv(csv_path)

    ils_rows = []

    for ds_name, ds_path in datasets.items():
        abs_path = os.path.join(os.path.dirname(__file__), ds_path)
        instance, graph, uav = make_instance(abs_path)

        # Filter tours for this dataset
        ds_tours = df_tours[df_tours["Instance"] == ds_name]

        for _, row in ds_tours.iterrows():
            h_name = row["Heuristic"]
            tour_nodes_str = row["Tour nodes"]
            feasible = row["Feasible"]

            if not feasible:
                ils_rows.append({
                    "Instance": ds_name,
                    "Initial tour": h_name,
                    "Init obj": None,
                    "Final obj": None,
                    "Improvement (%)": None,
                    "Time (s)": None,
                    "Final tour len": None,
                })
                continue

            # Parse tour nodes from CSV string
            tour_nodes = ast.literal_eval(tour_nodes_str)
            depot = graph.graph['base']

            # Reconstruct tour: depot + saved nodes
            full_tour_nodes = [depot] + tour_nodes
            tour_graph = directed_cycle(full_tour_nodes, graph)

            # Verify feasibility
            solver = Solver(tour_graph, instance)
            if solver.solution is None:
                print(f"  WARNING: {ds_name} / {h_name} tour infeasible on reconstruct")
                ils_rows.append({
                    "Instance": ds_name,
                    "Initial tour": h_name,
                    "Init obj": "INFEASIBLE",
                    "Final obj": None,
                    "Improvement (%)": None,
                    "Time (s)": None,
                    "Final tour len": None,
                })
                continue

            init_obj = solver.get_tour_data().objective
            print(f"  {ds_name} / {h_name}: init_obj = {init_obj:.2f}, running ILS...")

            # Run ILS
            state = State.initial_state(instance, tour_graph)
            optimizer = Optimizer(
                proposal=lambda s: universal_proposal(s),
                initial_state=state,
                maximize=True,
            )

            t0 = time.time()
            for s in optimizer.run_ils(
                total_steps=ILS_STEPS,
                t_improve=ILS_T_IMPROVE,
                k_remove=ILS_K_REMOVE,
                tabu_tenure=ILS_TABU_TENURE,
            ):
                pass
            elapsed = time.time() - t0

            final_obj = optimizer._best_score
            improvement = ((final_obj - init_obj) / abs(init_obj) * 100) if init_obj else None

            final_tour_len = None
            if optimizer._best_state:
                final_tour_len = len(list(optimizer._best_state.tour.nodes)) - 1

            ils_rows.append({
                "Instance": ds_name,
                "Initial tour": h_name,
                "Init obj": round(init_obj, 2),
                "Final obj": round(final_obj, 2) if final_obj else None,
                "Improvement (%)": round(improvement, 1) if improvement is not None else None,
                "Time (s)": round(elapsed, 1),
                "Final tour len": final_tour_len,
            })

            print(f"    final_obj = {final_obj:.2f}, improvement = {improvement:.1f}%, "
                  f"time = {elapsed:.1f}s, tour_len = {final_tour_len}")

    # Build results table
    df_ils = pd.DataFrame(ils_rows)
    print("\n" + "=" * 80)
    print("Table 3: ILS from Different Initial Tours (All Datasets)")
    print("=" * 80)
    print(df_ils.to_string(index=False))

    # Save results
    out_path = os.path.join(os.path.dirname(__file__), "ils_section2_results.csv")
    df_ils.to_csv(out_path, index=False)
    print(f"\nResults saved to {out_path}")


if __name__ == "__main__":
    main()
