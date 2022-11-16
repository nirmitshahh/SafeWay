import numpy as np
from typing import List, Dict, Tuple, Optional
from vehicles.vehicle import Vehicle, Intent
from v2v.message import V2VMessage


class ConflictResolver:
    """Handles conflict detection and resolution between vehicles"""
    
    def __init__(self, safety_buffer: float = 2.0, prediction_horizon: int = 10):
        self.safety_buffer = safety_buffer  # minimum distance between vehicles
        self.prediction_horizon = prediction_horizon
    
    def predict_collision(self, vehicle1: Vehicle, vehicle2: Vehicle, 
                         dt: float) -> Optional[float]:
        """
        Predict if two vehicles will collide
        Returns time to collision if collision predicted, None otherwise
        """
        # Get trajectories
        traj1 = vehicle1.compute_trajectory(dt, self.prediction_horizon)
        traj2 = vehicle2.compute_trajectory(dt, self.prediction_horizon)
        
        min_steps = min(len(traj1), len(traj2))
        
        for i in range(min_steps):
            dist = np.sqrt((traj1[i][0] - traj2[i][0])**2 + 
                          (traj1[i][1] - traj2[i][1])**2)
            if dist < self.safety_buffer:
                return i * dt  # time to collision
        
        return None
    
    def predict_collision_with_message(self, vehicle: Vehicle, 
                                       message: V2VMessage, dt: float) -> Optional[float]:
        """
        Predict collision using received V2V message
        """
        # Create temporary vehicle from message
        temp_vehicle = Vehicle(message.sender_id, message.position[0], message.position[1])
        temp_vehicle.set_velocity(message.velocity[0], message.velocity[1])
        temp_vehicle.heading = message.heading
        temp_vehicle.speed = message.speed
        
        # Use planned trajectory from message if available
        if message.planned_trajectory:
            traj1 = vehicle.compute_trajectory(dt, self.prediction_horizon)
            traj2 = message.planned_trajectory[:self.prediction_horizon]
            
            min_steps = min(len(traj1), len(traj2))
            for i in range(min_steps):
                dist = np.sqrt((traj1[i][0] - traj2[i][0])**2 + 
                              (traj1[i][1] - traj2[i][1])**2)
                if dist < self.safety_buffer:
                    return i * dt
        else:
            # Fallback to basic prediction
            return self.predict_collision(vehicle, temp_vehicle, dt)
        
        return None
    
    def should_yield(self, vehicle: Vehicle, other_messages: List[V2VMessage],
                    dt: float) -> Tuple[bool, Optional[int]]:
        """
        Determine if vehicle should yield to others
        Returns (should_yield, vehicle_id_to_yield_to)
        """
        closest_conflict = None
        min_ttc = float('inf')
        
        for message in other_messages:
            ttc = self.predict_collision_with_message(vehicle, message, dt)
            if ttc is not None and ttc < 2.0:  # collision within 2 seconds
                if ttc < min_ttc:
                    min_ttc = ttc
                    closest_conflict = message
        
        if closest_conflict:
            # Simple right-of-way: vehicle with lower ID has priority
            # (in real system, could use other rules like distance to intersection)
            if vehicle.id > closest_conflict.sender_id:
                return (True, closest_conflict.sender_id)
        
        return (False, None)
    
    def resolve_intersection_conflict(self, vehicle: Vehicle, 
                                     other_messages: List[V2VMessage],
                                     dt: float):
        """
        Resolve conflicts at intersections
        """
        should_yield, yield_to = self.should_yield(vehicle, other_messages, dt)
        
        if should_yield:
            vehicle.is_yielding = True
            vehicle.yield_target = yield_to
            vehicle.intent = Intent.YIELD
            # Slow down more aggressively at intersections
            vehicle.decelerate(dt, target_speed=vehicle.preferred_speed * 0.4)
        else:
            # Check if we can resume normal speed
            if vehicle.is_yielding and vehicle.yield_target == yield_to:
                # Clear yield state if threat has passed
                dist_to_threat = None
                for msg in other_messages:
                    if msg.sender_id == yield_to:
                        import numpy as np
                        dist_to_threat = np.sqrt((vehicle.x - msg.position[0])**2 + 
                                                (vehicle.y - msg.position[1])**2)
                        break
                
                if dist_to_threat is None or dist_to_threat > self.safety_buffer * 3:
                    vehicle.is_yielding = False
                    vehicle.yield_target = None
                    vehicle.intent = Intent.STRAIGHT
    
    def resolve_merge_conflict(self, vehicle: Vehicle, 
                              other_messages: List[V2VMessage],
                              dt: float):
        """
        Resolve conflicts during lane merges
        """
        should_yield, yield_to = self.should_yield(vehicle, other_messages, dt)
        
        if should_yield:
            vehicle.intent = Intent.MERGE
            vehicle.is_yielding = True
            vehicle.yield_target = yield_to
            # Slow down to let other vehicle pass
            vehicle.decelerate(dt, target_speed=vehicle.preferred_speed * 0.6)
        else:
            vehicle.intent = Intent.STRAIGHT
            vehicle.is_yielding = False
    
    def check_proximity_collision(self, vehicle: Vehicle,
                                  other_messages: List[V2VMessage],
                                  dt: float):
        """
        General proximity-based collision avoidance
        """
        for message in other_messages:
            dist = np.sqrt((vehicle.x - message.position[0])**2 + 
                          (vehicle.y - message.position[1])**2)
            
            if dist < self.safety_buffer * 1.5:  # too close
                # Emergency braking
                vehicle.decelerate(dt, target_speed=0.0)
                vehicle.intent = Intent.STOP
                return
        
        # If no immediate danger, resume normal operation
        if vehicle.intent == Intent.STOP and not vehicle.is_yielding:
            vehicle.intent = Intent.STRAIGHT

