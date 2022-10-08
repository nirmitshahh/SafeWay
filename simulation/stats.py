from typing import Dict, List
from collections import defaultdict


class Statistics:
    """Track simulation statistics"""
    
    def __init__(self):
        self.collisions = 0
        self.near_misses = 0
        self.yield_events = 0
        self.merge_events = 0
        self.vehicles_completed = 0
        self.total_distance_traveled = 0.0
        self.vehicle_distances: Dict[int, float] = defaultdict(float)
        self.vehicle_paths: Dict[int, List[tuple]] = defaultdict(list)
        self.messages_sent = 0
        self.messages_received = 0
        self.messages_dropped = 0
    
    def record_collision(self):
        """Record a collision event"""
        self.collisions += 1
    
    def record_near_miss(self):
        """Record a near miss"""
        self.near_miss += 1
    
    def record_yield(self):
        """Record a yield event"""
        self.yield_events += 1
    
    def record_merge(self):
        """Record a merge event"""
        self.merge_events += 1
    
    def record_vehicle_completion(self, vehicle_id: int):
        """Record vehicle reaching destination"""
        self.vehicles_completed += 1
    
    def update_vehicle_position(self, vehicle_id: int, x: float, y: float):
        """Update vehicle position for distance tracking"""
        if vehicle_id in self.vehicle_paths:
            last_pos = self.vehicle_paths[vehicle_id][-1] if self.vehicle_paths[vehicle_id] else None
            if last_pos:
                import math
                dist = math.sqrt((x - last_pos[0])**2 + (y - last_pos[1])**2)
                self.vehicle_distances[vehicle_id] += dist
                self.total_distance_traveled += dist
        
        self.vehicle_paths[vehicle_id].append((x, y))
    
    def record_message_sent(self):
        """Record a message being sent"""
        self.messages_sent += 1
    
    def record_message_received(self):
        """Record a message being received"""
        self.messages_received += 1
    
    def record_message_dropped(self):
        """Record a dropped message"""
        self.messages_dropped += 1
    
    def get_summary(self) -> Dict:
        """Get summary statistics"""
        return {
            'collisions': self.collisions,
            'near_misses': self.near_misses,
            'yield_events': self.yield_events,
            'merge_events': self.merge_events,
            'vehicles_completed': self.vehicles_completed,
            'total_distance': self.total_distance_traveled,
            'messages_sent': self.messages_sent,
            'messages_received': self.messages_received,
            'messages_dropped': self.messages_dropped
        }

