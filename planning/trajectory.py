import numpy as np
from typing import List, Tuple


class TrajectoryPredictor:
    """Predicts future vehicle trajectories for collision detection"""
    
    def __init__(self, horizon: int = 10, dt: float = 0.1):
        self.horizon = horizon
        self.dt = dt
    
    def predict_constant_velocity(self, x: float, y: float, 
                                  vx: float, vy: float) -> List[Tuple[float, float]]:
        """Predict trajectory assuming constant velocity"""
        trajectory = []
        px, py = x, y
        
        for _ in range(self.horizon):
            px += vx * self.dt
            py += vy * self.dt
            trajectory.append((px, py))
        
        return trajectory
    
    def predict_with_acceleration(self, x: float, y: float,
                                  vx: float, vy: float,
                                  ax: float, ay: float) -> List[Tuple[float, float]]:
        """Predict trajectory with constant acceleration"""
        trajectory = []
        px, py = x, y
        cvx, cvy = vx, vy
        
        for _ in range(self.horizon):
            px += cvx * self.dt + 0.5 * ax * self.dt**2
            py += cvy * self.dt + 0.5 * ay * self.dt**2
            cvx += ax * self.dt
            cvy += ay * self.dt
            trajectory.append((px, py))
        
        return trajectory
    
    def time_to_collision(self, traj1: List[Tuple[float, float]],
                         traj2: List[Tuple[float, float]],
                         safety_radius: float) -> float:
        """Calculate time to collision between two trajectories"""
        min_steps = min(len(traj1), len(traj2))
        
        for i in range(min_steps):
            dist = np.sqrt((traj1[i][0] - traj2[i][0])**2 + 
                          (traj1[i][1] - traj2[i][1])**2)
            if dist < safety_radius:
                return i * self.dt
        
        return float('inf')

