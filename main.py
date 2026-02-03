import random 
import cProfile

from uav_routing.environment.environment_real import build_environment  

def main ():
    print("Starting UAV Routing Environment Setup...")
    
    random.seed(42)
    
    
    
    
    
if __name__ == "__main__":
    cProfile.run("main()", sort="tottime")