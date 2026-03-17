"""
environment.py
==============
Normalized problem wrapper with eta energy scaling.

The Environment class takes a CalibrationResult and a Drone, applies the
eta energy budget multiplier, and precomputes normalized lookups for the
MISOCP solver (distances, time windows, speed bounds). All normalization
denominators are fixed at eta=1 for numerical stability.
"""

from typing import Optional
from functools import cached_property

from .graph import Graph
from .drone import Drone
from .calibration import CalibrationResult
from dataclasses import dataclass

#logger = logging.getLogger(__name__)
#logger.setLevel(logging.DEBUG)
#if not logger.handlers:
#    ch = logging.StreamHandler(sys.stdout)
#    ch.setLevel(logging.DEBUG)
#    fmt = logging.Formatter('%(asctime)s %(levelname)s [%(name)s] %(message)s')
#    ch.setFormatter(fmt)
#    logger.addHandler(ch)
#logger.debug('Reference tour: %d nodes, distance=%.1f, tour time=%.1f, factor=%.4f, speed=%.1f',l, total_distance, tour_time, factor, speed)


    


class Environment:
    """
    Applies eta (energy scaling) to a calibrated UAV routing instance.
    This is what we pass to the solver.

    calib (CalibrationResult): Calibrated graph and scaling parameters.
    drone (Drone): UAV platform.
    eta (float, optional): Scales energy budget. eta=1 means full sortie energy;
        eta<1 tightens the energy constraint. Defaults to 1.0.
    """

    def __init__(self,
                 calib: CalibrationResult,
                 drone: Drone,
                 eta: float = 1.0):

        self.calib = calib
        self.graph = calib.graph
        self.drone = drone
        self.eta = eta
        self._build_normalization()


    @cached_property
    def time_horizon(self) -> float:
        """Effective mission duration in seconds."""
        return self.calib.drone_campaign_time

    @cached_property
    def max_energy(self) -> float:
        """Effective energy budget in Joules."""
        return self.calib.scaled_max_energy * self.eta

    @cached_property
    def max_distance(self) -> float:
        """Maximum distance (meters) achievable at optimum speed with current energy budget."""
        return self.max_energy / self.drone.energy_per_meter(self.drone.optimum_speed)

    @cached_property
    def max_time(self):
        """Maximum flight time (seconds) at optimum speed with current energy budget."""
        return  self.max_distance / self.drone.optimum_speed

    @property
    def T_max_s(self) -> float:
        """Mission time horizon in normalized (dimensionless) units."""
        return self.time_horizon / self._t_norm

    def update_scaling(self, eta=None):
        """Update the energy budget multiplier and invalidate cached properties.

        Parameters
        ----------
        eta : float, optional
            New energy budget multiplier. If None, keeps current value.
        """
        if eta is not None:
            self.eta = eta
            self.__dict__.pop('max_energy', None)
            self.__dict__.pop('max_distance', None)
            self.__dict__.pop('max_time', None)
            

    def _build_normalization(self):
        """Compute fixed normalization denominators and scaled lookups once at init.
        These are purely for model stability and must not change when alpha/beta change."""
        
        v_opt = self.drone.optimum_speed

        # Fixed normalization base: alpha=1, beta=1
        self._d_norm = self.calib.scaled_max_energy / self.drone.energy_per_meter(v_opt)
        self._t_norm = self._d_norm / v_opt

        self.speed_max_s = self.drone.speed_max / v_opt
        self.speed_min_s = self.drone.speed_min / v_opt

        self.d_scaled  = {(i, j): self.graph[i][j]['distance'] / self._d_norm
                          for i, j in self.graph.edges()}
        # undirected graph — add both directions
        self.d_scaled.update({(j, i): v for (i, j), v in list(self.d_scaled.items())})

        self.tw_scaled = {i: (self.graph.nodes[i]['time_window'][0] / self._t_norm,
                              self.graph.nodes[i]['time_window'][1] / self._t_norm)
                          for i in self.graph.nodes()}
     
        
        



