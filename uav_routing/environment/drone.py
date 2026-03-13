"""
drone.py
========

Drone dataclasses representing UAV specifications and energy models.

Key Classes:
- Drone_test: Rotary-wing UAV (generic, for unit tests).
- Drone: Fixed-wing drone (Bayraktar TB2) with calibration support.

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
    
    speed_min: float = 20           # stall speed (m/s)
    speed_max: float = 61           # maximum speed (m/s)
    optimum_speed: float = 44.31    # optimal travel speed (m/s), e.g. 44
    loiter_speed: float = 33.5      # optimal loitering speed (m/s), e.g. 33.5
    
    c_0: float = 0                  # avionics power coefficient 
    c_1: float = 0.0905              # parasitic drag coefficient
    c_2: float = 349095             # induced drag coefficient



    @property
    def loiter_power(self):
        v = self.loiter_speed
        return self.c_0 + self.c_1 * v**3 + self.c_2 / v
    
    def power(self, v):
        """Calculates Power (Watts) for a given speed v (m/s)."""
        return self.c_0 + self.c_1 * v**3 + + self.c_2 / v
    
    def energy_function(self, v, d):
        """Calculates Energy (Joules) for distance d at speed v."""
        if v == 0: raise ValueError("speed is zero!")
        return (self.c_0 * d / v) + (self.c_1 * d * v**2) + (self.c_2 * d / v**2)
    
    def socp_energy_function(self, t, y, z):
        """Linearized energy function for the SOCP solver."""
        return self.c_0 * t + self.c_1 * y + self.c_2 * z
    
    def energy_per_meter(self, v):
        """Calculates energy consumption per meter at given speed v."""
        return self.energy_function(v, 1.0)
    
    @staticmethod
    def collected_info(arrival_time, earliest_time, slope, info_at_lowest):
        return slope*(arrival_time - earliest_time) + info_at_lowest


