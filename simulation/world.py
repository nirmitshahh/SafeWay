import numpy as np
from typing import List, Dict, Optional
from simulation.map import Map
from vehicles.vehicle import Vehicle
from planning.pathfinder import PathFinder
from planning.conflict_resolver import ConflictResolver
from v2v.comm_bus import CommunicationBus
from v2v.message import V2VMessage


class World:
    """Main simulation world that manages all entities"""
    
    def __init__(self, map_obj: Map, use_v2v: bool = True,
                 broadcast_radius: float = 50.0,
                 latency: float = 0.0,
                 packet_drop_rate: float = 0.0):
        self.map = map_obj
        self.use_v2v = use_v2v
        self.vehicles: Dict[int, Vehicle] = {}
        self.pathfinder = PathFinder(map_obj)
        self.conflict_resolver = ConflictResolver()
        self.comm_bus = CommunicationBus(broadcast_radius, latency, packet_drop_rate)
        
        self.time = 0.0
        self.dt = 0.1  # time step in seconds
        self.step_count = 0
    
    def add_vehicle(self, vehicle: Vehicle, destination: Optional[Tuple[float, float]] = None):
        """Add a vehicle to the world"""
        self.vehicles[vehicle.id] = vehicle
        
        # Plan initial path if destination provided
        if destination:
            vehicle.destination = destination
            path = self.pathfinder.find_path((vehicle.x, vehicle.y), destination)
            if path:
                vehicle.set_path(path)
    
    def update(self):
        """Update simulation by one time step"""
        self.step_count += 1
        self.time += self.dt
        self.comm_bus.update_time(self.dt)
        
        # Update vehicle trajectories
        for vehicle in self.vehicles.values():
            vehicle.update_trajectory(self.dt)
        
        # V2V communication
        if self.use_v2v:
            self._process_v2v_communication()
        
        # Update vehicle behaviors
        for vehicle in self.vehicles.values():
            self._update_vehicle(vehicle)
        
        # Update positions
        for vehicle in self.vehicles.values():
            vehicle.update_position(self.dt)
    
    def _process_v2v_communication(self):
        """Process V2V message broadcasting and delivery"""
        # Collect all vehicle positions
        vehicle_positions = {vid: (v.x, v.y) for vid, v in self.vehicles.items()}
        
        # Broadcast messages from each vehicle
        for vehicle in self.vehicles.values():
            message = V2VMessage(
                sender_id=vehicle.id,
                position=(vehicle.x, vehicle.y),
                velocity=vehicle.get_velocity(),
                heading=vehicle.heading,
                speed=vehicle.speed,
                intent=vehicle.intent,
                planned_trajectory=vehicle.planned_trajectory,
                timestamp=self.time
            )
            self.comm_bus.broadcast(message, (vehicle.x, vehicle.y), vehicle_positions)
        
        # Deliver messages to vehicles
        for vehicle in self.vehicles.values():
            messages = self.comm_bus.get_messages_for_vehicle(vehicle.id)
            self._handle_received_messages(vehicle, messages)
    
    def _handle_received_messages(self, vehicle: Vehicle, messages: List[V2VMessage]):
        """Handle V2V messages received by a vehicle"""
        if not messages:
            return
        
        # Resolve conflicts using received messages
        self.conflict_resolver.resolve_intersection_conflict(vehicle, messages, self.dt)
        self.conflict_resolver.resolve_merge_conflict(vehicle, messages, self.dt)
        self.conflict_resolver.check_proximity_collision(vehicle, messages, self.dt)
    
    def _update_vehicle(self, vehicle: Vehicle):
        """Update a single vehicle's behavior"""
        # Check if reached waypoint
        if vehicle.reached_waypoint():
            vehicle.advance_path()
        
        # Get next waypoint and steer toward it
        waypoint = vehicle.get_next_waypoint()
        if waypoint:
            vehicle.steer_toward(waypoint[0], waypoint[1])
            
            # Accelerate if not yielding
            if not vehicle.is_yielding:
                vehicle.accelerate(self.dt)
        else:
            # Reached destination or no path
            vehicle.decelerate(self.dt, target_speed=0.0)
            if vehicle.speed < 0.1:
                vehicle.stop()
    
    def get_nearby_vehicles_for_rendering(self, vehicle_id: int) -> List[V2VMessage]:
        """Get nearby vehicles for visualization (used in baseline mode too)"""
        if vehicle_id not in self.vehicles:
            return []
        
        vehicle = self.vehicles[vehicle_id]
        nearby = []
        
        for other_id, other_vehicle in self.vehicles.items():
            if other_id == vehicle_id:
                continue
            
            dist = np.sqrt((vehicle.x - other_vehicle.x)**2 + 
                          (vehicle.y - other_vehicle.y)**2)
            
            if dist <= self.comm_bus.broadcast_radius:
                message = V2VMessage(
                    sender_id=other_vehicle.id,
                    position=(other_vehicle.x, other_vehicle.y),
                    velocity=other_vehicle.get_velocity(),
                    heading=other_vehicle.heading,
                    speed=other_vehicle.speed,
                    intent=other_vehicle.intent,
                    planned_trajectory=other_vehicle.planned_trajectory,
                    timestamp=self.time
                )
                nearby.append(message)
        
        return nearby

