# SafeWay: Decentralized V2V Coordination System

SafeWay is a simulation system that demonstrates vehicle-to-vehicle (V2V) coordination for autonomous navigation. Vehicles share their intent, position, and planned trajectories with nearby vehicles to enable collaborative path planning and conflict resolution.

## Features

- 2D map visualization with real-time vehicle movement
- A* pathfinding for route planning
- V2V communication with configurable radius, latency, and packet loss
- Collision detection and conflict resolution
- Baseline mode (local sensing) and V2V mode (intent sharing) for comparison

## Installation

```bash
pip install -r requirements.txt
```

## Usage

```bash
python main.py --map data/maps/intersection.json --scenario data/scenarios/intersection_4cars.json
```

### Options

- `--map`: Path to map JSON file (required)
- `--scenario`: Path to scenario JSON file (required)
- `--mode`: `baseline` or `v2v` (default: `v2v`)
- `--radius`: V2V broadcast radius (default: 50.0)
- `--latency`: Communication latency in seconds (default: 0.0)
- `--packet-loss`: Packet drop rate 0.0-1.0 (default: 0.0)
- `--fps`: Frame rate (default: 30)
- `--speed`: Simulation speed multiplier (default: 1.0)

### Controls

- **SPACE**: Pause/Resume
- **ESC**: Quit

## Project Structure

```
SafeWay/
├── main.py              # Entry point
├── simulation/         # Simulation engine
├── vehicles/           # Vehicle system
├── planning/           # Path planning and conflict resolution
├── v2v/               # V2V communication
├── visualization/     # Rendering
└── data/              # Maps and scenarios
```

## Creating Maps and Scenarios

Maps define the road network (nodes, edges, intersections). Scenarios define vehicle spawn positions, destinations, and behaviors. See the examples in `data/maps/` and `data/scenarios/` for format.

## How It Works

1. Vehicles plan paths using A* pathfinding
2. Vehicles broadcast V2V messages with position, velocity, intent, and trajectory
3. Conflict resolution detects potential collisions and triggers yielding
4. Vehicles adjust speed and steering based on received messages
5. Visualization shows vehicles, paths, trajectories, and intent indicators
