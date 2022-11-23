# SafeWay: Decentralized Vehicle-to-Vehicle Coordination System

## Project Overview

SafeWay is a prototype simulation system designed to demonstrate the concept of decentralized vehicle-to-vehicle (V2V) coordination for autonomous navigation. Unlike traditional traffic management systems that rely on centralized infrastructure, SafeWay enables vehicles to make collaborative decisions through peer-to-peer communication, sharing intent, position, and planned trajectories with nearby vehicles.

This project was developed as a concept test to explore how autonomous vehicles can improve safety and efficiency through decentralized coordination, without requiring external infrastructure or real-world vehicle telemetry. All vehicle behavior, maps, and scenarios are simulated using synthetic data.

## Core Concept

The fundamental idea behind SafeWay is that vehicles can make better navigation decisions when they have access to the intentions and planned trajectories of nearby vehicles, rather than relying solely on local sensing of positions. By broadcasting their intent (turning, merging, yielding) and short-term trajectory predictions, vehicles can:

- **Predict and avoid collisions** before they occur
- **Negotiate right-of-way** at intersections and merges
- **Coordinate lane merges** smoothly
- Maintain safety through **decentralized decision-making**

The system demonstrates robustness, scalability, and fault tolerance through its decentralized architecture, where each vehicle makes decisions locally based on messages from neighbors, without requiring a central traffic controller.

## System Architecture

### Components

The SafeWay system is organized into several modular components:

#### 1. **Simulation Engine** (`simulation/`)
- **`map.py`**: Defines the road network data structure (nodes, edges, intersections) and map loading from JSON files
- **`world.py`**: Main simulation world that manages all entities, time stepping, and coordinates between components
- **`scenario.py`**: Loads and parses scenario files that define vehicle spawn positions, destinations, and behaviors
- **`stats.py`**: Tracks simulation statistics (collisions, yields, messages, etc.)

#### 2. **Vehicle System** (`vehicles/`)
- **`vehicle.py`**: Core vehicle class representing a single autonomous vehicle with position, velocity, heading, path, and intent
- **`controller.py`**: Vehicle controller that handles steering, acceleration, and waypoint following

#### 3. **Path Planning** (`planning/`)
- **`pathfinder.py`**: Implements A* pathfinding algorithm to find optimal routes through the road network
- **`conflict_resolver.py`**: Detects potential collisions and resolves conflicts through yielding and negotiation
- **`trajectory.py`**: Predicts future vehicle trajectories for collision detection

#### 4. **V2V Communication** (`v2v/`)
- **`message.py`**: Defines the V2V message format containing position, velocity, intent, and planned trajectory
- **`comm_bus.py`**: Simulates the communication network with configurable broadcast radius, latency, and packet drop rates

#### 5. **Visualization** (`visualization/`)
- **`renderer.py`**: 2D pygame-based renderer that displays the map, vehicles, paths, trajectories, and intent indicators

## How It Works

### Simulation Flow

1. **Initialization**: The system loads a map (road network) and a scenario (vehicle configurations)
2. **Path Planning**: Each vehicle uses A* pathfinding to compute a route from its spawn point to its destination
3. **Simulation Loop**: For each time step:
   - Vehicles update their planned trajectories based on current state
   - Vehicles broadcast V2V messages to nearby vehicles (within broadcast radius)
   - Vehicles receive and process messages from neighbors
   - Conflict resolution analyzes potential collisions and triggers yielding/negotiation
   - Vehicle controllers update steering and speed based on path and conflicts
   - Vehicle positions are updated
   - Visualization is rendered

### V2V Communication

Each vehicle periodically broadcasts messages containing:
- Current position (x, y)
- Velocity vector (vx, vy) and speed
- Heading angle
- Intent (straight, left turn, right turn, yield, merge, stop)
- Planned trajectory (next N positions)

The communication bus simulates:
- **Broadcast radius**: Only vehicles within a configurable distance receive messages
- **Latency**: Optional delay before messages are delivered
- **Packet loss**: Configurable probability that messages are dropped

### Conflict Resolution

The conflict resolver uses trajectory prediction to detect potential collisions:
1. For each nearby vehicle (via V2V messages), predict future positions
2. Check if trajectories intersect within a safety buffer
3. If collision predicted, determine right-of-way (currently based on vehicle ID)
4. Lower-priority vehicle yields by slowing down and setting intent to YIELD
5. Once threat passes, vehicle resumes normal operation

### Path Planning

The A* pathfinding algorithm:
1. Converts the road network into a graph of nodes and edges
2. Finds the optimal path from start to destination using heuristic search
3. Smooths the path to remove unnecessary waypoints
4. Vehicles follow waypoints sequentially, steering toward the next waypoint

## Running the Simulation

### Prerequisites

- Python 3.7 or higher
- Required packages (install via `pip install -r requirements.txt`):
  - numpy
  - matplotlib
  - pygame

### Basic Usage

```bash
python main.py --map data/maps/intersection.json --scenario data/scenarios/intersection_4cars.json
```

### Command Line Options

- `--map`: Path to map JSON file (required)
- `--scenario`: Path to scenario JSON file (required)
- `--mode`: Simulation mode - `baseline` (local sensing only) or `v2v` (with intent sharing). Default: `v2v`
- `--radius`: V2V broadcast radius in distance units. Default: 50.0
- `--latency`: Communication latency in seconds. Default: 0.0
- `--packet-loss`: Packet drop rate (0.0 to 1.0). Default: 0.0
- `--fps`: Simulation frame rate. Default: 30
- `--speed`: Simulation speed multiplier. Default: 1.0

### Example Commands

```bash
# Run intersection scenario with V2V enabled
python main.py --map data/maps/intersection.json --scenario data/scenarios/intersection_4cars.json

# Run in baseline mode (no V2V communication)
python main.py --map data/maps/intersection.json --scenario data/scenarios/intersection_4cars.json --mode baseline

# Test with communication latency and packet loss
python main.py --map data/maps/merge_lane.json --scenario data/scenarios/merge_2cars.json --latency 0.1 --packet-loss 0.1

# Run roundabout scenario
python main.py --map data/maps/roundabout.json --scenario data/scenarios/roundabout_4cars.json
```

### Controls

- **SPACE**: Pause/Resume simulation
- **ESC**: Quit simulation

## Creating Custom Maps

Maps are defined in JSON format with the following structure:

```json
{
  "nodes": [
    {"id": 0, "x": 0, "y": 50},
    {"id": 1, "x": 50, "y": 50}
  ],
  "edges": [
    {"from": 0, "to": 1, "width": 1.0}
  ],
  "intersections": [
    [1]
  ],
  "obstacles": [
    {"x": 25, "y": 25, "radius": 2.0}
  ],
  "spawn_points": [
    {"x": 10, "y": 50}
  ],
  "destinations": [
    {"x": 90, "y": 50}
  ]
}
```

- **nodes**: Road network nodes (intersections, waypoints) with unique IDs and coordinates
- **edges**: Connections between nodes representing roads/lanes
- **intersections**: Groups of node IDs that form intersections (highlighted in visualization)
- **obstacles**: Optional obstacles with position and radius
- **spawn_points**: Vehicle spawn locations
- **destinations**: Target locations for vehicles

## Creating Custom Scenarios

Scenarios define vehicle configurations:

```json
{
  "vehicles": [
    {
      "id": 0,
      "spawn_x": 10,
      "spawn_y": 50,
      "destination_x": 90,
      "destination_y": 50,
      "initial_heading": 0.0,
      "initial_speed": 3.0,
      "preferred_speed": 4.0,
      "aggressiveness": 0.5
    }
  ]
}
```

- **id**: Unique vehicle identifier
- **spawn_x/y**: Starting position
- **destination_x/y**: Target destination
- **initial_heading**: Starting orientation in radians
- **initial_speed**: Starting speed
- **preferred_speed**: Target cruising speed
- **aggressiveness**: Behavior parameter (0.0 = cautious, 1.0 = aggressive)

## Visualization Features

The visualization displays:

- **Road network**: Gray lines showing roads/edges, black circles for nodes
- **Intersections**: Orange highlighted circles
- **Vehicles**: Colored rectangles oriented by heading
- **Planned paths**: Darker colored lines showing full route
- **Trajectories**: Yellow lines showing short-term predicted positions
- **Intent indicators**: Colored circles showing vehicle intent:
  - Blue: Left turn
  - Green: Right turn
  - Orange: Yielding
  - Red: Stopped
- **V2V connections**: Light blue lines showing active communication links (V2V mode only)
- **UI panel**: Simulation time, step count, mode, vehicle count, and statistics

## Comparison: Baseline vs V2V Mode

The system supports two modes for comparison:

### Baseline Mode
- Vehicles can only sense positions of nearby vehicles (no intent/trajectory sharing)
- Collision avoidance relies on reactive behavior
- Less efficient coordination at intersections and merges

### V2V Mode
- Vehicles share intent and planned trajectories
- Proactive collision avoidance through trajectory prediction
- Better coordination and smoother traffic flow
- Demonstrates the benefits of decentralized V2V communication

## Technical Details

### Algorithms

- **Pathfinding**: A* algorithm with Euclidean distance heuristic
- **Trajectory Prediction**: Constant velocity model with configurable horizon
- **Collision Detection**: Time-to-collision calculation using predicted trajectories
- **Conflict Resolution**: Priority-based yielding with distance-based threat assessment

### Performance Considerations

- Simulation time step: 0.1 seconds (configurable)
- Trajectory prediction horizon: 10 steps (1 second ahead)
- Safety buffer: 2.0 distance units minimum separation
- Broadcast radius: 50.0 distance units (configurable)

## Project Structure

```
SafeWay/
├── main.py                 # Entry point
├── config.py              # Configuration constants
├── utils.py               # Utility functions
├── requirements.txt       # Python dependencies
├── simulation/           # Simulation engine
│   ├── map.py
│   ├── world.py
│   ├── scenario.py
│   └── stats.py
├── vehicles/             # Vehicle system
│   ├── vehicle.py
│   └── controller.py
├── planning/             # Path planning and conflict resolution
│   ├── pathfinder.py
│   ├── conflict_resolver.py
│   └── trajectory.py
├── v2v/                  # V2V communication
│   ├── message.py
│   └── comm_bus.py
├── visualization/        # Rendering
│   └── renderer.py
└── data/                 # Maps and scenarios
    ├── maps/
    └── scenarios/
```

## Limitations and Future Work

This is a prototype system with several simplifications:

- **Simplified physics**: Basic kinematic model, no complex vehicle dynamics
- **Simple negotiation**: Right-of-way based on vehicle ID (could use more sophisticated rules)
- **Synthetic data**: No real-world vehicle telemetry or road networks
- **2D only**: No elevation or 3D considerations
- **No sensor noise**: Perfect position and velocity information

Potential enhancements:
- More sophisticated negotiation protocols
- Integration with real road network data
- 3D visualization
- Sensor noise modeling
- More complex vehicle dynamics
- Machine learning for behavior optimization

## Conclusion

SafeWay demonstrates how decentralized V2V coordination can improve autonomous vehicle navigation through peer-to-peer communication. The system shows that vehicles can make better decisions when they share intent and trajectories, leading to safer and more efficient traffic flow. While this is a simplified prototype, it provides a foundation for exploring more advanced coordination strategies in autonomous vehicle systems.

## License

This project was developed for educational and research purposes.

