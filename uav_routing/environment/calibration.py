
"""
calibration.py
==============
Calibrates Solomon benchmark instances to real-world Bayraktar TB2
drone specifications through spatial and time scaling.

Two calibration methods:
    - **energy**: Scales so the reference tour consumes the full energy
      budget at optimum speed.
    - **campaign**: Scales so the Solomon time horizon maps to the drone's
      sortie duration.

All solver computations use dimensionless normalized variables for
numerical stability. Physical units are recovered only at result extraction.
"""

import random
import pandas as pd
import networkx as nx
from typing import Optional
from dataclasses import dataclass

from .drone import Drone
from .graph import nearest_neighbor_tour, nearest_neighbor_tour_time



@dataclass
class CalibrationResult:
    """Container for calibration outputs.

    Attributes
    ----------
    graph : nx.Graph
        Calibrated graph with scaled time windows and distances.
    tour : nx.DiGraph
        Reference tour (directed cycle) over n/2 customers.
    spatial_scale : float
        Meters per Solomon coordinate unit.
    time_scale : float
        Seconds per Solomon time unit.
    tour_distance_drone : float
        Reference tour distance in meters.
    tour_dist_solomon : float
        Reference tour distance in Solomon units.
    solomon_tour_energy : float
        Reference tour energy in Joules (unscaled distances).
    scaled_tour_energy : float
        Reference tour energy in Joules (scaled distances).
    drone_campaign_time : float
        Mission duration in seconds.
    scaled_max_energy : float
        Energy budget in Joules (at eta=1).
    """
    graph: nx.Graph              # calibrated graph with scaled time windows and distances,
    tour: nx.Graph              # reference tour (DiGraph) over n/2 customers, with edge distances
    spatial_scale: float        # meters per Solomon coordinate unit
    time_scale: float           # seconds per Solomon time unit = spatial_scale / v_opt
    tour_distance_drone: float    # meters
    tour_dist_solomon: float       # Solomon units
    solomon_tour_energy: float      # Joules
    scaled_tour_energy: float       # Joules
    drone_campaign_time: float  # hours, for energy budget context in calibration_info
    scaled_max_energy: float     # Joules, for energy budget context in calibration_info
    

        
def calibrate(graph: nx.Graph,
              tour_length: int, 
              drone: Drone,
              drone_sortie_time: float = 3.0,   # hours
              calibration_method: str = "energy",
            ) -> CalibrationResult:
    """
    Calibrate the instance:
      - Compute spatial and time scale
      - Apply time scaling to time windows and spatial scaling to edge distances
      - Build reference tour over n/2 customers
      - Compute reference energy
      - Set T_max_base = campaign_time, E_max_base from reference tour
    """
    depot = graph.graph['base']
    

    if calibration_method == "energy":
        ref_tour = nearest_neighbor_tour_time(
                                     graph = graph, 
                                     depot = depot, 
                                     tour_length = tour_length
                                     )
        spatial_scale, time_scale, tour_dist_solomon, max_drone_distance, drone_campaign_time = calculate_scales_energy(graph, ref_tour, drone, drone_sortie_time)
        scaled_max_energy = drone.energy_function(drone.optimum_speed, tour_dist_solomon*spatial_scale)
        
    elif calibration_method == "campaign":
        ref_tour = nearest_neighbor_tour(
                                     graph = graph,
                                     depot = depot,
                                     tour_length = tour_length
                                     )
        spatial_scale, time_scale, tour_dist_solomon, max_drone_distance, drone_campaign_time = calculate_scales_campaign(graph, ref_tour, drone, drone_sortie_time)
        max_solomon_distance = graph.nodes[depot]['time_window'][1] * drone.optimum_speed
        scaled_max_energy = drone.power(drone.optimum_speed)*drone_campaign_time

    elif calibration_method == "uniform":
        ref_tour = nearest_neighbor_tour_time(
                                     graph = graph,
                                     depot = depot,
                                     tour_length = tour_length
                                     )
        spatial_scale, time_scale, tour_dist_solomon, max_drone_distance, drone_campaign_time = calculate_scales_uniform(graph, ref_tour, drone, drone_sortie_time)
        scaled_max_energy = drone.energy_function(drone.optimum_speed, tour_dist_solomon * spatial_scale)

    elif calibration_method == "hybrid":
        ref_tour = nearest_neighbor_tour(
                                     graph = graph,
                                     depot = depot,
                                     tour_length = tour_length
                                     )
        spatial_scale, time_scale, tour_dist_solomon, max_drone_distance, drone_campaign_time = calculate_scales_campaign(graph, ref_tour, drone, drone_sortie_time)
        scaled_max_energy = drone.energy_function(drone.optimum_speed, tour_dist_solomon * spatial_scale)


        
    solomon_tour_energy = drone.energy_function(drone.optimum_speed, tour_dist_solomon)
    
    
    new_graph = _callibrate_data(graph=graph, 
                                 spatial_scale=spatial_scale, 
                                 time_scale=time_scale,
                                 )
    return CalibrationResult(
        graph=new_graph,
        drone_campaign_time=drone_campaign_time,
        spatial_scale=spatial_scale,
        time_scale = time_scale,
        tour_dist_solomon = tour_dist_solomon,
        solomon_tour_energy = solomon_tour_energy,
        scaled_tour_energy = solomon_tour_energy * spatial_scale,
        tour_distance_drone = tour_dist_solomon * spatial_scale,
        tour=ref_tour,
        scaled_max_energy = scaled_max_energy
    )


def calculate_scales_energy(graph, tour, drone, drone_sortie_time):
    """Compute spatial and time scales using the energy calibration method.

    Sets spatial_scale so that the reference tour distance in meters equals
    the maximum range at optimum speed for the given sortie time.

    Returns
    -------
    tuple
        (spatial_scale, time_scale, tour_dist_solomon, max_drone_distance, drone_campaign_time)
    """
    data_campaign_time = graph.nodes[drone.base]['time_window'][1]
    drone_campaign_time = drone_sortie_time * 3600   
    tour_dist_solomon = sum(graph.edges[edge]['distance'] for edge in tour.edges)
    max_drone_distance = drone.optimum_speed * drone_sortie_time * 3600
    spatial_scale = max_drone_distance / tour_dist_solomon
    time_scale = spatial_scale / drone.optimum_speed                   
    return spatial_scale, time_scale, tour_dist_solomon, max_drone_distance, drone_campaign_time


def calculate_scales_uniform(graph, tour, drone, drone_sortie_time):
    """Like energy calibration but α = β (single scaler for distance and time)."""
    drone_campaign_time = drone_sortie_time * 3600
    tour_dist_solomon = sum(graph.edges[edge]['distance'] for edge in tour.edges)
    time_scale = drone_campaign_time / tour_dist_solomon
    spatial_scale = time_scale  # α = β, preserves Solomon's d = t
    max_drone_distance = drone.optimum_speed * drone_campaign_time
    return spatial_scale, time_scale, tour_dist_solomon, max_drone_distance, drone_campaign_time


def calculate_scales_campaign(graph, tour, drone, drone_sortie_time):
    """Compute spatial and time scales using the campaign-time calibration method.

    Sets time_scale so that the Solomon campaign time maps to the drone's
    sortie duration, then derives spatial_scale = time_scale * v_opt.

    Returns
    -------
    tuple
        (spatial_scale, time_scale, tour_dist_solomon, max_drone_distance, drone_campaign_time)
    """
    data_campaign_time = graph.nodes[drone.base]['time_window'][1]
    drone_campaign_time = drone_sortie_time * 3600
    tour_dist_solomon = sum(graph.edges[edge]['distance'] for edge in tour.edges)
    max_drone_distance = drone.optimum_speed * drone_sortie_time * 3600
    time_scale = drone_campaign_time / data_campaign_time
    spatial_scale = time_scale * drone.optimum_speed
    return spatial_scale, time_scale, tour_dist_solomon, max_drone_distance, drone_campaign_time


def _callibrate_data(graph, spatial_scale, time_scale):
    """Apply spatial and time scaling to a Solomon graph.

    Scales time windows by time_scale and edge distances by spatial_scale.
    Info slopes are rescaled to match the new time units.

    Parameters
    ----------
    graph : nx.Graph
        Original unscaled Solomon graph.
    spatial_scale : float
        Meters per Solomon distance unit.
    time_scale : float
        Seconds per Solomon time unit.

    Returns
    -------
    nx.Graph
        New graph with calibrated attributes.
    """
    new_graph = graph.copy()
    
    # 1. Scale Time Windows and add info_slope based on the scaled time windows
    for node in graph.nodes:
        tw = graph.nodes[node]['time_window']
        slope = graph.nodes[node]['info_slope']
        scaled_tw = (tw[0] * time_scale, tw[1] * time_scale)
        new_graph.nodes[node]['time_window'] = scaled_tw
        new_slope = slope * time_scale
        new_graph.nodes[node]['info_slope'] =  new_slope  
        
        if node == graph.graph['base']: continue
        info_at_earliest  = new_graph.nodes[node]['info_at_lowest']
        info_at_latest  = new_slope * (scaled_tw[1] - scaled_tw[0]) + info_at_earliest
    
    # 2. Scale Distances
    for edge in graph.edges:
        dist = graph.edges[edge]['distance']
        new_graph.edges[edge]['distance'] = dist * spatial_scale
        
    return new_graph




def calibration_info(old_graph: nx.Graph,
                     calib: CalibrationResult, 
                     drone: Drone):
    """
    Print a human-readable calibration summary.
    """
    n_customers = len(calib.graph.nodes) - 1
    print("=" * 55)
    print("  CALIBRATION SUMMARY")
    print("=" * 55)
    print(f"  Nodes (incl. depot)   : {len(calib.graph.nodes)}")
    print(f"  UAV loiter speed      : {drone.loiter_speed:.2f} m/s")
    print(f"  UAV loiter power      : {drone.loiter_power:.2f} W")
    print(f"  UAV optimum speed     : {drone.optimum_speed:.2f} m/s")
    print(f"  UAV cruise power      : {drone.power(drone.optimum_speed):.2f} W")
    print("-" * 55)
    print(f"  Spatial scale         : {calib.spatial_scale} m / Solomon unit")
    print(f"  Time scale            : {calib.time_scale:.4f} s / Solomon unit")
    print(f"  tour distance         : {calib.tour_dist_solomon:.2f} Solomon units")
    print(f"  Scaled tour distance  : {calib.tour_dist_solomon * calib.spatial_scale:.1f} m")
    print(f"  Solomon tour energy   : {calib.solomon_tour_energy:.2f} J")
    print(f"  scaled tour energy    : {calib.scaled_tour_energy:.2f} J")  
    print(f"  solomon tour time     : {calib.tour_dist_solomon / drone.optimum_speed:.2f} s")
    print(f"  scaled tour time      : {calib.tour_dist_solomon * calib.spatial_scale / drone.optimum_speed:.2f} s")
    print("-" * 55)
    print(f"  scaled max energy      : {calib.scaled_max_energy:.2f} J")
    print(f"  drone max energy      : {drone.power(drone.optimum_speed) * calib.drone_campaign_time:.2f} J")
    print(f"  Solomon campaign time : {old_graph.nodes[drone.base]['time_window'][1]:.2f} Solomon units")
    print(f"  Scaled campaign time  : {calib.graph.nodes[drone.base]['time_window'][1]:.1f} s")
    print("=" * 55)
    print("  (alpha, beta) GRID PREVIEW")
    print("-" * 55)
    #for alpha in [0.6, 0.8, 1.0]:
    #    for beta in [0.6, 0.8, 1.0]:
    #        T = alpha * calib.T_max_base * calib.time_scale / 3600
    #        E = beta * calib.tour_energy
    #        print(f"  α={alpha:.1f}, β={beta:.1f}  →  "
    #              f"T_max={T:.2f} h,  E_max={E:.1f} J")
    #print("=" * 55)
    node = random.choice(list(calib.graph.nodes))
    print(f"Example node {node} info:")
    print(f"  Original time window   : {old_graph.nodes[node]['time_window']}")
    print(f"  Calibrated time window : {calib.graph.nodes[node]['time_window']}")
    print(f"  Info at lowest         : {calib.graph.nodes[node]['info_at_lowest']}")
    print(f"  Info slope             : {calib.graph.nodes[node]['info_slope']:.4f} per second")

    
    print("-" * 55)
    #print(f"Energy available in {calib.drone_sortie_time}h : {E_available:.1f} J")
    #print(f"scaled tour energy                   : {E_tour:.1f} J")
    #print(f"Tour exhausts {coverage_ratio*100:.1f}% of 3h energy budget")
    #print(f"=> E_max (beta=1.0) covers ~{1/coverage_ratio:.2f}x the ref tour")
   #print("-" * 55)
    print(f"  Reference tour nodes  : {list(calib.tour.nodes)}"
          f" with (= n/2 = {n_customers//2})")

    
