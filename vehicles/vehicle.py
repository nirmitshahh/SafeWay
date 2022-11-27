import numpy as np
from typing import List, Tuple, Optional
from enum import Enum


def normalize_angle(angle: float) -> float:
    """Normalize angle to [0, 2*pi]"""
    while angle < 0:
        angle += 2 * np.pi
    while angle >= 2 * np.pi:
        angle -= 2 * np.pi
    return angle


class Intent(Enum):
    STRAIGHT = "straight"
    LEFT = "left"
    RIGHT = "right"
    YIELD = "yield"
    MERGE = "merge"
    STOP = "stop"


class Vehicle:
    """Represents a single vehicle in the simulation"""
    
    def __init__(self, vehicle_id: int, x: float, y: float, 
                 heading: float = 0.0, speed: float = 0.0):
        self.id = vehicle_id
        self.x = x
        self.y = y
        self.heading = heading  # radians
        self.speed = speed  # units per second
        self.max_speed = 5.0
        self.acceleration = 2.0
        self.deceleration = 3.0
        
        # Path planning
        self.path: List[Tuple[float, float]] = []
        self.current_path_index = 0
        self.destination: Optional[Tuple[float, float]] = None
        
        # Intent and trajectory
        self.intent = Intent.STRAIGHT
        self.planned_trajectory: List[Tuple[float, float]] = []  # next N positions
        self.trajectory_horizon = 10  # number of steps ahead to plan
        
        # Behavior parameters
        self.aggressiveness = 0.5  # 0.0 = very cautious, 1.0 = aggressive
        self.preferred_speed = 4.0
        
        # State
        self.is_yielding = False
        self.yield_target = None  # vehicle ID we're yielding to
        
    def get_velocity(self) -> Tuple[float, float]:
        """Get velocity vector (vx, vy)"""
        vx = self.speed * np.cos(self.heading)
        vy = self.speed * np.sin(self.heading)
        return (vx, vy)
    
    def set_velocity(self, vx: float, vy: float):
        """Set velocity from vector"""
        self.speed = np.sqrt(vx**2 + vy**2)
        if self.speed > 0:
            self.heading = np.arctan2(vy, vx)
        self.speed = min(self.speed, self.max_speed)
    
    def update_position(self, dt: float):
        """Update position based on current velocity"""
        vx, vy = self.get_velocity()
        self.x += vx * dt
        self.y += vy * dt
    
    def set_path(self, path: List[Tuple[float, float]]):
        """Set the planned path"""
        self.path = path
        self.current_path_index = 0
    
    def get_next_waypoint(self) -> Optional[Tuple[float, float]]:
        """Get the next waypoint in the path"""
        if self.current_path_index < len(self.path):
            return self.path[self.current_path_index]
        return None
    
    def reached_waypoint(self, threshold: float = 0.5) -> bool:
        """Check if we've reached the current waypoint"""
        waypoint = self.get_next_waypoint()
        if waypoint is None:
            return True
        
        dist = np.sqrt((self.x - waypoint[0])**2 + (self.y - waypoint[1])**2)
        return dist < threshold
    
    def advance_path(self):
        """Move to next waypoint in path"""
        if self.reached_waypoint():
            self.current_path_index += 1
    
    def compute_trajectory(self, dt: float, steps: int = None) -> List[Tuple[float, float]]:
        """Compute predicted trajectory for next N steps"""
        if steps is None:
            steps = self.trajectory_horizon
        
        trajectory = []
        x, y = self.x, self.y
        vx, vy = self.get_velocity()
        
        # Simple constant velocity prediction
        for _ in range(steps):
            x += vx * dt
            y += vy * dt
            trajectory.append((x, y))
        
        return trajectory
    
    def update_trajectory(self, dt: float):
        """Update planned trajectory"""
        self.planned_trajectory = self.compute_trajectory(dt)
    
    def steer_toward(self, target_x: float, target_y: float):
        """Steer toward a target position"""
        dx = target_x - self.x
        dy = target_y - self.y
        dist = np.sqrt(dx**2 + dy**2)
        
        if dist < 0.1:  # already very close
            return
        
        target_heading = np.arctan2(dy, dx)
        
        # Smooth heading change
        angle_diff = target_heading - self.heading
        # Normalize to [-pi, pi]
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        
        max_turn_rate = 0.1  # radians per step
        turn = np.clip(angle_diff, -max_turn_rate, max_turn_rate)
        self.heading += turn
    
    def accelerate(self, dt: float):
        """Accelerate toward preferred speed"""
        if self.speed < self.preferred_speed:
            self.speed = min(self.speed + self.acceleration * dt, self.preferred_speed, self.max_speed)
    
    def decelerate(self, dt: float, target_speed: float = 0.0):
        """Decelerate"""
        if self.speed > target_speed:
            self.speed = max(self.speed - self.deceleration * dt, target_speed)
    
    def stop(self):
        """Stop the vehicle"""
        self.speed = 0.0
        self.intent = Intent.STOP
    
    def update_control(self, dt: float):
        """Update vehicle control (steering and speed)"""
        waypoint_threshold = 0.5
        max_turn_rate = 0.15
        
        # Check if reached waypoint
        if self.reached_waypoint(waypoint_threshold):
            self.advance_path()
        
        # Get next waypoint
        waypoint = self.get_next_waypoint()
        
        if waypoint:
            # Steer toward waypoint
            dx = waypoint[0] - self.x
            dy = waypoint[1] - self.y
            target_heading = np.arctan2(dy, dx)
            
            # Calculate heading difference
            angle_diff = target_heading - self.heading
            
            # Normalize to [-pi, pi]
            while angle_diff > np.pi:
                angle_diff -= 2 * np.pi
            while angle_diff < -np.pi:
                angle_diff += 2 * np.pi
            
            # Apply turn rate limit
            turn = np.clip(angle_diff, -max_turn_rate, max_turn_rate)
            self.heading += turn
            self.heading = normalize_angle(self.heading)
            
            # Speed control
            if not self.is_yielding:
                self.accelerate(dt)
            else:
                self.decelerate(dt, self.preferred_speed * 0.5)
        else:
            # No path or reached destination
            self.decelerate(dt, 0.0)
            if self.speed < 0.1:
                self.stop()

