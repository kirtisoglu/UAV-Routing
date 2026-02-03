

from .drone import Drone
from .graph import directed_cycle, dict_to_graph



     
def grid_3x3():
    """
    Generator that yields node attributes on a 3x3 grid.
    """
    
    drone = Drone(
                base= 0, 
                max_energy=6030, 
                campaign_time= 100, 
                speed_min= 20, 
                speed_max= 100,
    )
    
    
    tour_nodes = [0,3,6] # frozen???
    initial_tour = directed_cycle(tour_nodes)
    
    nodes = {} 

    for i in range(3):
        for j in range(3):
            
            position = (i, j)
            node = i + j * 3  # unique node id

            if i==0 and j==0:
                time_window=[0,0]
                slope=0
                info_at_lowest=0
            
            else:    
                if i == 1: 
                    time_window = [1000, 2000]
                    slope = 20
                    info_at_lowest = 100

                elif i == 2:  # rightmost column
                    time_window = [0, 2000]
                    slope = 100
                    info_at_lowest = 4000
                else:
                    time_window = [0, 2000]
                    slope = 20
                    info_at_lowest = 100
                    
            #slope = (info_at_highest - info_at_lowest) / (highest - lowest)
            nodes[node] = node_data = {"position":position,
                                        "info_at_lowest":info_at_lowest, 
                                        "info_slope":slope,
                                        "time_window":time_window
                                        }
    graph = dict_to_graph(nodes, base_id=0)
    return graph, initial_tour, drone
    


def tour_optimum_speed(max_energy=300):
    """
    A dummy data to observe whether Solver can find v_opt. 
    
    """
    drone = Drone(
                base = 0, 
                destination = 26,
                max_energy = max_energy, 
                campaign_time = 100000, 
                speed_min = 10, 
                speed_max = 50,
    )
     
    tour_nodes = [0,1,2,3]
    initial_tour = directed_cycle(tour_nodes)
    
    
    nodes = {} 
    
    for i in range(5):
        for j in range(5):

            nodes[i] = {"position":(i,j),
                        "info_at_lowest":10, 
                        "info_slope":0,
                        "time_window":[0,10000]
                        }
    graph = dict_to_graph(nodes, base_id=0)
    
    return graph, initial_tour, drone



def tour_info_increasing():
    """
    Info increasing in time, uav patrols as much as it can.
    v_ij =10 for every edge, where 99/10 of it is just for 
    patroling. Optimal cost is 330
    """
    
    drone = Drone(
                base = 0, 
                max_energy = 10000, 
                campaign_time = 10000, 
                speed_min = 10, 
                speed_max = 20,
                )
    
    tour_nodes = [0,1,2]
    initial_tour = directed_cycle(tour_nodes)

    nodes = {} 
    for i in range(3):

        nodes[i] = {"position":(0,i),
                    "info_at_lowest":10, 
                    "info_slope":10,
                    "time_window":[0,10*i]
                    }
    graph = dict_to_graph(nodes, base_id=0)
    
    return graph, initial_tour, drone


def tour_info_decreasing():
    """
    Info is decreasing in time. So, uav goes as fast as
    possible without waiting at all. v_ij = 10, where 
    patroling time is 0. Objective cost is 440.
    """
    drone = Drone(
                base = 0, 
                max_energy = 10000, 
                campaign_time = 10000, 
                speed_min = 1, 
                speed_max = 10,
    )
     
    tour_nodes = [0,1,2,3]
    initial_tour = directed_cycle(tour_nodes)

    
    nodes = {} 
    
    
    for i in range(4):

        nodes[i] = {"position":(0,i),
                    "info_at_lowest":110, 
                    "info_slope":-1/10,
                    "time_window":[0,1000]
                    }
        
    graph = dict_to_graph(nodes, base_id=0)
    
    return graph, initial_tour, drone


def grid_10x10_feasible():
    """
    Generator that yields node attributes on a 3x3 grid.
    """

    drone = Drone(
                base = 0, 
                max_energy = 6030, 
                campaign_time = 100, 
                speed_min = 20, 
                speed_max = 100,
                )
    
    tour_nodes = [0,3,6] # must be sorted?? frozen???
    initial_tour = directed_cycle(tour_nodes)
    
    nodes = {} 

    for i in range(3):
        for j in range(3):
            
            position = (i, j)
            node = i + j * 3  # unique node id

            if i==0 and j==0:
                time_window=[0,0]
                slope=0
                info_at_lowest=0
            
            else:    
                if i == 1: 
                    time_window = [1000, 2000]
                    slope = 20
                    info_at_lowest = 100

                elif i == 2:  # rightmost column
                    time_window = [0, 2000]
                    slope = 100
                    info_at_lowest = 4000
                else:
                    time_window = [0, 2000]
                    slope = 20
                    info_at_lowest = 100
                    
            #slope = (info_at_highest - info_at_lowest) / (highest - lowest)
            nodes[node]  = {"position":position,
                            "info_at_lowest":info_at_lowest, 
                            "info_slope":slope,
                            "time_window":time_window
                            }

    graph = dict_to_graph(nodes, base_id=0)
    
    return graph, initial_tour, drone










