import numpy as np
from typing import List, Dict, Optional, Tuple
from v2v.message import V2VMessage
import random


class CommunicationBus:
    """
    Simulates decentralized V2V communication network
    Handles broadcasting, range limits, latency, and packet loss
    """
    
    def __init__(self, broadcast_radius: float = 50.0, 
                 latency: float = 0.0, 
                 packet_drop_rate: float = 0.0):
        self.broadcast_radius = broadcast_radius
        self.latency = latency  # seconds
        self.packet_drop_rate = packet_drop_rate  # 0.0 to 1.0
        
        # Message queues for delayed delivery
        self.message_queue: List[Tuple[float, V2VMessage, int]] = []  # (delivery_time, message, receiver_id)
        self.current_time = 0.0
    
    def update_time(self, dt: float):
        """Update simulation time"""
        self.current_time += dt
    
    def broadcast(self, message: V2VMessage, sender_pos: Tuple[float, float],
                  all_vehicles: Dict[int, Tuple[float, float]]) -> List[V2VMessage]:
        """
        Broadcast a message to nearby vehicles
        Returns list of messages that should be delivered (after latency/packet loss)
        """
        delivered_messages = []
        
        for vehicle_id, vehicle_pos in all_vehicles.items():
            if vehicle_id == message.sender_id:
                continue
            
            # Check if within broadcast radius
            dist = np.sqrt((sender_pos[0] - vehicle_pos[0])**2 + 
                          (sender_pos[1] - vehicle_pos[1])**2)
            
            if dist <= self.broadcast_radius:
                # Check packet drop
                if random.random() > self.packet_drop_rate:
                    # Calculate delivery time with latency
                    delivery_time = self.current_time + self.latency
                    self.message_queue.append((delivery_time, message, vehicle_id))
        
        return delivered_messages
    
    def get_messages_for_vehicle(self, vehicle_id: int) -> List[V2VMessage]:
        """
        Get all messages that should be delivered to a vehicle at current time
        """
        messages = []
        remaining_queue = []
        
        for delivery_time, message, receiver_id in self.message_queue:
            if receiver_id == vehicle_id and delivery_time <= self.current_time:
                messages.append(message)
            else:
                remaining_queue.append((delivery_time, message, receiver_id))
        
        self.message_queue = remaining_queue
        return messages
    
    def clear_queue(self):
        """Clear all pending messages"""
        self.message_queue = []

