
from collections import namedtuple


Node = namedtuple('Node', 'position info_at_lowest decay_factor time_window')

        
def grid_3x3():
    """
    Generator that yields node attributes on a 3x3 grid.
    """
    
    metadata = {
                "base": (0,0), 
                "max_energy":6030, 
                "campaign_time": 100, 
                "speed_min": 20, 
                "speed_max": 100,
                }
    
    tour = [3,6]
    nodes = {}
    
    for i in range(3):
        for j in range(3):
            
            position = (i, j)
            node = i + j * 3  # unique node id

            if i==0 and j==0:
                time_window=[0,0]
            elif i == 1: 
                time_window = [1000, 2000]
            else: 
                time_window = [0, 2000]
            
            if i == 2:  # rightmost column
                info_at_lowest = 4000
                slope = 100
            else:
                info_at_lowest = 100
                slope = 20
            
            
            #slope = (info_at_highest - info_at_lowest) / (highest - lowest)
            nodes[node] = Node(position=position,
                                info_at_lowest=info_at_lowest,
                                info_slope=slope,
                                time_window=time_window,
                            )
    # final destination
    nodes[9] = Node(position=(0,0),
                        info_at_lowest=0,
                        info_slope=0,
                        time_window=[0,100],
                    )
            
    return nodes, tour, metadata        
    



def initial_tour_greedy():
    
    "?????"
    
    return
        

def generate_random_data():
    return