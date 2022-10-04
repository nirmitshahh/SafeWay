import numpy as np
from typing import Optional, Tuple
from vehicles.vehicle import Vehicle


class VehicleController:
    """Controls vehicle behavior and decision making"""
    
    def __init__(self, vehicle: Vehicle):
        self.vehicle = vehicle
        self.waypoint_threshold = 0.5
        self.max_turn_rate = 0.15
    
    def update(self, dt: float):
        """Update vehicle control"""
        # Check if reached waypoint
        if self.vehicle.reached_waypoint(self.waypoint_threshold):
            self.vehicle.advance_path()
        
        # Get next waypoint
        waypoint = self.vehicle.get_next_waypoint()
        
        if waypoint:
            # Steer toward waypoint
            self._steer_to_waypoint(waypoint, dt)
            
            # Speed control
            if not self.vehicle.is_yielding:
                self.vehicle.accelerate(dt)
            else:
                self.vehicle.decelerate(dt, self.vehicle.preferred_speed * 0.5)
        else:
            # No path or reached destination
            self.vehicle.decelerate(dt, 0.0)
            if self.vehicle.speed < 0.1:
                self.vehicle.stop()
    
    def _steer_to_waypoint(self, waypoint: Tuple[float, float], dt: float):
        """Steer vehicle toward waypoint"""
        dx = waypoint[0] - self.vehicle.x
        dy = waypoint[1] - self.vehicle.y
        target_heading = np.arctan2(dy, dx)
        
        # Calculate heading difference
        angle_diff = target_heading - self.vehicle.heading
        
        # Normalize to [-pi, pi]
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        
        # Apply turn rate limit
        turn = np.clip(angle_diff, -self.max_turn_rate, self.max_turn_rate)
        self.vehicle.heading += turn
        
        # Normalize heading to [0, 2*pi]
        while self.vehicle.heading < 0:
            self.vehicle.heading += 2 * np.pi
        while self.vehicle.heading >= 2 * np.pi:
            self.vehicle.heading -= 2 * np.pi

