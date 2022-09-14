from dataclasses import dataclass
from typing import List, Tuple
from vehicles.vehicle import Intent
import json


@dataclass
class V2VMessage:
    """Message format for vehicle-to-vehicle communication"""
    sender_id: int
    position: Tuple[float, float]  # (x, y)
    velocity: Tuple[float, float]  # (vx, vy)
    heading: float  # radians
    speed: float
    intent: Intent
    planned_trajectory: List[Tuple[float, float]]  # next N positions
    timestamp: float
    
    def to_dict(self) -> dict:
        """Convert message to dictionary for serialization"""
        return {
            'sender_id': self.sender_id,
            'position': list(self.position),
            'velocity': list(self.velocity),
            'heading': self.heading,
            'speed': self.speed,
            'intent': self.intent.value,
            'planned_trajectory': [list(pos) for pos in self.planned_trajectory],
            'timestamp': self.timestamp
        }
    
    @staticmethod
    def from_dict(data: dict) -> 'V2VMessage':
        """Create message from dictionary"""
        from vehicles.vehicle import Intent
        return V2VMessage(
            sender_id=data['sender_id'],
            position=tuple(data['position']),
            velocity=tuple(data['velocity']),
            heading=data['heading'],
            speed=data['speed'],
            intent=Intent(data['intent']),
            planned_trajectory=[tuple(pos) for pos in data['planned_trajectory']],
            timestamp=data['timestamp']
        )

