"""
drone.py
=================

Overview
--------
Processes data files line-by-line, validates field counts, and handles errors gracefully.
Supports CSV-like files with exactly 10 fields per valid line.

Key Functions:
- Drone_test: returns a test drone dataclass reoresenting a rotary-wing UAV.
- Drone: returns a fixed-wing drone dataclass reoresenting Bayraktar TB2.

Usage:

Dependencies:
- dataclasses 

Last update: 2025-12-20
"""

# TODO: Drone max energy and campaign time consistency check.

from dataclasses import dataclass, field, replace



@dataclass
class Drone_test:
    "Represents a rotary-wing UAV with its specifications."
    
    name = "Generic UAV"
    base: int
    max_energy: float
    campaign_time: float
    
    speed_min: float
    speed_max: float
    cruise_speed = 18.11 # m/s (approx. 65 km/h)
    endurance_time = None  # seconds, not specified
    
    hover_profile_coeff = 79.85628000000003 # Watts, mu_1. 
    forward_profile_coeff = 0.016636725000000005 # mu_2
    induced_power_coeff = 321.82080840000015  # mu_3
    parasitic_power_coeff = 0.009242624999999999  #  mu_4
    
    power_function: callable = field(init=None)
    energy_function: callable = field(init=None)
    socp_energy_function: callable = field(init=None)
    
    def __post_init__(self):
        self.power_function =lambda v: self.hover_profile_coeff + self.forward_profile_coeff * v**2 + self.induced_power_coeff / v + self.parasitic_power_coeff * v**3
        self.energy_function = lambda v, d: self.power_function(v) * d / v  # Joules
        self.socp_energy_function = lambda v, d, t, y, z: self.hover_profile_coeff*t + self.forward_profile_coeff*d*v + self.induced_power_coeff*z/d + self.parasitic_power_coeff*d*y





@dataclass
class Drone:
    """
    Represents the UAV Bayraktar TB2 with its specifications. 
    A fixed-wing drone manufactured by Turkey.
    """
    
    base: int 
    
    name: str = "Bayraktar TB2"
    speed_min: float = 20  
    speed_max: float = 61      
    optimum_speed: float = 44.31  
    
    c_0: float = 0 
    c_1: float = 0.0905
    c_2: float = 349095
    
    time_factor: float = 1.0
    energy_factor: float = 1.0
    tour_time: float = 0

    @property
    def max_energy(self):
        energy = self.energy_function(self.optimum_speed, self.max_distance())
        return self.energy_factor * energy
    
    @property
    def max_time(self):
        return self.time_factor*self.tour_time
    
    def power_function(self, v):
        """Calculates Power (Watts) for a given speed v (m/s)."""
        return self.c_0 + self.c_1 * (v**3) + self.c_2 / v

    def energy_function(self, v, d):
        """Calculates Energy (Joules) for distance d at speed v."""
        if v == 0: raise ValueError("speed is zero!")
        return self.power_function(v) * (d / v)

    
    def socp_energy_function(self, t, y, z):
        """Linearized energy function for the SOCP solver."""
        return self.c_0 * t + self.c_1 * y + self.c_2 * z

    def max_distance(self):
        """Calculates the maximum distance the drone can cover at optimum speed."""
        return self.max_time*self.optimum_speed
    
    
    def energy_per_meter(self):
        """Calculates energy consumption per meter at optimum speed."""
        return self.energy_function(self.optimum_speed, 1.0)
    
    @staticmethod
    def collected_info(arrival_time, earliest_time, slope, info_at_lowest):
        return slope*(arrival_time - earliest_time) + info_at_lowest


    def callibrate_time(self, time: float, 
                        t_factor: float, 
                        e_factor: float) -> "Drone":
        return replace(self, 
                       tour_time=time, 
                       time_factor=t_factor, 
                       energy_factor=e_factor)
    
    