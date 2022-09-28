import json
from typing import List, Dict
from vehicles.vehicle import Vehicle


def load_scenario(filepath: str) -> List[Dict]:
    """Load scenario from JSON file"""
    with open(filepath, 'r') as f:
        data = json.load(f)
    return data.get('vehicles', [])


def create_vehicles_from_scenario(scenario_data: List[Dict]) -> List[Vehicle]:
    """Create Vehicle objects from scenario data"""
    vehicles = []
    
    for vehicle_data in scenario_data:
        vehicle = Vehicle(
            vehicle_id=vehicle_data['id'],
            x=vehicle_data['spawn_x'],
            y=vehicle_data['spawn_y'],
            heading=vehicle_data.get('initial_heading', 0.0),
            speed=vehicle_data.get('initial_speed', 0.0)
        )
        
        vehicle.preferred_speed = vehicle_data.get('preferred_speed', 4.0)
        vehicle.aggressiveness = vehicle_data.get('aggressiveness', 0.5)
        vehicle.destination = (vehicle_data['destination_x'], vehicle_data['destination_y'])
        
        vehicles.append(vehicle)
    
    return vehicles

