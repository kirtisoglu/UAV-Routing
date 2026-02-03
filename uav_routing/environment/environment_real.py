
# Orchestrator



import random

from typing import Optional
import pandas as pd

from .data import data_to_dict
from .drone import Drone
from .graph import dict_to_graph, nearest_neighbors_cycle



def build_environment(
                      data_path: str,
                      node_ratio: float = 1/2,
                      time_factor: float = 1,
                      energy_factor: float =1,
                    ):
    """_summary_

    Args:
        data_path (str): 
        node_ratio (float): 

    Returns:
        _type_: _description_
    """
    data, base = data_to_dict(data_path)
    graph = dict_to_graph(data, int(base))
    drone = Drone(base=int(base))
    
    conversion_factor, campaign_time = conversion_factor_with_nearest_tour(node_ratio, graph, drone)
    callibrated_graph = _callibrate_data(graph, conversion_factor)
    
    drone = drone.callibrate_time(time=campaign_time,
                                t_factor=time_factor,
                                e_factor=energy_factor)
    print('tour time', drone.tour_time)
    print('campaign time', campaign_time)
    report = get_calibration_report(graph, drone, conversion_factor)
    
    return callibrated_graph, drone, report




def _callibrate_data(graph, conversion_factor):
    "Calibrates a Solomon dataset (graph attributes?) to be used with a real time UAV."
    
    # Distance 1 unit = 1000 m is already converted during the graph construction, 
    # since it was needed for conversion factor calculation. 
    
    graph.graph['conversion_factor'] = conversion_factor
    import random
    u = random.choice(list(graph.nodes))
    print("random node", u)
    print("time window of u before scaling", graph.nodes[u]["time_window"])
    print("lowest info at u before scaling", graph.nodes[u]["info_at_lowest"])
    
    
    for node in graph.nodes:
        # 1. Scale Time Window
        tw = graph.nodes[node]['time_window']
        scaled_tw = [tw[0] * conversion_factor, tw[1] * conversion_factor]
        graph.nodes[node]['time_window'] = scaled_tw
        
        # 2. Scale Earliest Info
        i_e = graph.nodes[node]['info_at_lowest'] * conversion_factor
        graph.nodes[node]['info_at_lowest'] = i_e
        
        # 3. Calculate slope using SCALED values
        # This ensures gamma * (scaled_duration) = scaled_info
        slope = _get_random_slope(scaled_tw, i_e)
        graph.nodes[node]['info_slope'] = slope
    
    print("time window of u after scaling", graph.nodes[u]["time_window"])
    print("lowest info at u after scaling", graph.nodes[u]["info_at_lowest"])
    return graph



# distances have to be converted before applying this function.
def conversion_factor_with_nearest_tour(node_ratio, graph, drone: Drone):
    """
    Calculates a conversion factor to scale Solomon data for a given drone.
    The conversion factor is the ratio of the total energy used by the nearest 
    neighbor cycle with optimum speed covering a ratio of the nodes and the 
    maximum energy of the drone.
    """
    l = int(len(graph) * node_ratio)
    print("l", l)
    C = nearest_neighbors_cycle(graph, l)
    print("cycle nodes", C.nodes)
    print("cycle edges", C.edges)
    
    speed = drone.optimum_speed
    print("speed", speed)
    total_distance = sum(C.edges[e]['distance'] for e in C.edges)
    print("distance before calibration", total_distance)
    campaign_time = total_distance / speed
    print("time to complete the tour before calibration", campaign_time)
    data_campaign_time = graph.nodes[drone.base]['time_window'][1]
    print("data campaign time", data_campaign_time)
    factor = campaign_time / data_campaign_time
    print("factor", factor)
    return factor, campaign_time




# must be done after calibration to pick from scaled time window.
def _get_random_slope(time_window, I_e):
    """
    Calculates the boundary for the slope and returns a random 
    value within that boundary to ensure 0 <= r_i <= 2*I_e.
    """
    # Calculate the time window duration
    delta_t = time_window[1] - time_window[0]
    
    # Calculate the maximum allowable absolute slope
    # This ensures m stays in [-I_e/delta_t, I_e/delta_t]
    slope_bound = I_e / delta_t
    
    # Sample a random slope from the uniform distribution
    random_slope = random.uniform(-slope_bound, slope_bound)
    
    return random_slope




def get_calibration_report(graph, drone, conversion_factor):
    data = []
    
    # Calculate max_energy once before the loop to avoid shadowing issue
    energy = drone.max_energy
    
    # We iterate through nodes to see how specific attributes shifted
    for node in graph.nodes:
        # Original values (Assuming you haven't run calibration yet)
        tw_orig = graph.nodes[node]['time_window']
        info_orig = graph.nodes[node]['info_at_lowest']
        
        # Calibrated values
        tw_cal = [tw_orig[0] * conversion_factor, tw_orig[1] * conversion_factor]
        info_cal = info_orig * conversion_factor
        
        data.append({
            'Node': node,
            'Original TW': tw_orig,
            'Calibrated TW': tw_cal,
            'Original Info': info_orig,
            'Calibrated Info': info_cal,
            'Drone Max Energy': energy,
            'Drone Max Time': drone.max_time,
            'Conv Factor': conversion_factor,
        })
    
    return pd.DataFrame(data)

