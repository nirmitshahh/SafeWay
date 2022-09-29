#!/usr/bin/env python3
"""
SafeWay - Decentralized V2V Coordination System
Main entry point for the simulation
"""

import argparse
import sys
import pygame
from simulation.map import Map
from simulation.world import World
from simulation.scenario import load_scenario, create_vehicles_from_scenario
from visualization.renderer import Renderer


def main():
    parser = argparse.ArgumentParser(description='SafeWay V2V Coordination Simulation')
    parser.add_argument('--map', type=str, required=True,
                       help='Path to map JSON file')
    parser.add_argument('--scenario', type=str, required=True,
                       help='Path to scenario JSON file')
    parser.add_argument('--mode', type=str, choices=['baseline', 'v2v'], default='v2v',
                       help='Simulation mode: baseline (local sensing) or v2v (with intent sharing)')
    parser.add_argument('--radius', type=float, default=50.0,
                       help='V2V broadcast radius (default: 50.0)')
    parser.add_argument('--latency', type=float, default=0.0,
                       help='Communication latency in seconds (default: 0.0)')
    parser.add_argument('--packet-loss', type=float, default=0.0,
                       help='Packet drop rate 0.0-1.0 (default: 0.0)')
    parser.add_argument('--fps', type=int, default=30,
                       help='Simulation FPS (default: 30)')
    parser.add_argument('--speed', type=float, default=1.0,
                       help='Simulation speed multiplier (default: 1.0)')
    
    args = parser.parse_args()
    
    # Load map
    try:
        map_obj = Map.load_from_json(args.map)
    except Exception as e:
        print(f"Error loading map: {e}")
        sys.exit(1)
    
    # Load scenario
    try:
        scenario_data = load_scenario(args.scenario)
        vehicles = create_vehicles_from_scenario(scenario_data)
    except Exception as e:
        print(f"Error loading scenario: {e}")
        sys.exit(1)
    
    # Create world
    use_v2v = (args.mode == 'v2v')
    world = World(map_obj, use_v2v=use_v2v, 
                  broadcast_radius=args.radius,
                  latency=args.latency,
                  packet_drop_rate=args.packet_loss)
    
    # Add vehicles to world and plan paths
    from planning.pathfinder import PathFinder
    pathfinder = PathFinder(map_obj)
    
    for vehicle in vehicles:
        if vehicle.destination:
            path = pathfinder.find_path((vehicle.x, vehicle.y), vehicle.destination)
            if path:
                vehicle.set_path(path)
        world.add_vehicle(vehicle)
    
    # Create renderer
    renderer = Renderer()
    renderer.center_on_map(map_obj)
    
    # Main simulation loop
    running = True
    paused = False
    
    print("SafeWay Simulation Started")
    print(f"Mode: {args.mode}")
    print(f"Vehicles: {len(vehicles)}")
    print("Controls:")
    print("  SPACE - Pause/Resume")
    print("  ESC - Quit")
    
    while running:
        # Handle events
        running = renderer.handle_events()
        
        # Handle keyboard input
        keys = pygame.key.get_pressed()
        if keys[pygame.K_SPACE]:
            paused = not paused
            pygame.time.wait(200)  # Debounce
        
        if not paused:
            # Update simulation
            world.update()
        
        # Render
        renderer.clear()
        renderer.draw_map(map_obj)
        
        # Draw vehicles and V2V connections
        for vehicle in world.vehicles.values():
            nearby = world.get_nearby_vehicles_for_rendering(vehicle.id)
            if use_v2v:
                renderer.draw_v2v_connections(vehicle, nearby, args.radius)
            renderer.draw_vehicle(vehicle, show_trajectory=True, show_intent=True)
        
        # Draw UI
        renderer.draw_ui(world.time, world.step_count, args.mode.upper(), len(vehicles))
        
        renderer.update()
        renderer.tick(int(args.fps * args.speed))
    
    pygame.quit()
    print("Simulation ended")


if __name__ == '__main__':
    main()

