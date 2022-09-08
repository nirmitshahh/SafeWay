import json
import numpy as np
from typing import List, Tuple, Dict, Optional


class RoadNode:
    """Represents a node in the road network"""
    def __init__(self, node_id: int, x: float, y: float):
        self.id = node_id
        self.x = x
        self.y = y
        self.connections = []  # list of connected node IDs


class RoadEdge:
    """Represents an edge/lane between two nodes"""
    def __init__(self, from_node: int, to_node: int, width: float = 1.0):
        self.from_node = from_node
        self.to_node = to_node
        self.width = width


class Map:
    """2D map representing road network"""
    def __init__(self):
        self.nodes: Dict[int, RoadNode] = {}
        self.edges: List[RoadEdge] = []
        self.intersections: List[Tuple[int, ...]] = []  # groups of node IDs that form intersections
        self.obstacles: List[Tuple[float, float, float]] = []  # (x, y, radius)
        self.spawn_points: List[Tuple[float, float]] = []
        self.destinations: List[Tuple[float, float]] = []
        self.bounds = (0, 0, 100, 100)  # min_x, min_y, max_x, max_y
        
    def add_node(self, node_id: int, x: float, y: float):
        """Add a node to the map"""
        self.nodes[node_id] = RoadNode(node_id, x, y)
        
    def add_edge(self, from_node: int, to_node: int, width: float = 1.0):
        """Add an edge between two nodes"""
        if from_node in self.nodes and to_node in self.nodes:
            self.edges.append(RoadEdge(from_node, to_node, width))
            self.nodes[from_node].connections.append(to_node)
    
    def get_node_position(self, node_id: int) -> Optional[Tuple[float, float]]:
        """Get position of a node"""
        if node_id in self.nodes:
            return (self.nodes[node_id].x, self.nodes[node_id].y)
        return None
    
    def get_neighbors(self, node_id: int) -> List[int]:
        """Get connected neighbor nodes"""
        if node_id in self.nodes:
            return self.nodes[node_id].connections
        return []
    
    def distance(self, node1_id: int, node2_id: int) -> float:
        """Calculate Euclidean distance between two nodes"""
        pos1 = self.get_node_position(node1_id)
        pos2 = self.get_node_position(node2_id)
        if pos1 and pos2:
            return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        return float('inf')
    
    @staticmethod
    def load_from_json(filepath: str) -> 'Map':
        """Load map from JSON file"""
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        map_obj = Map()
        
        # Load nodes
        for node_data in data.get('nodes', []):
            map_obj.add_node(node_data['id'], node_data['x'], node_data['y'])
        
        # Load edges
        for edge_data in data.get('edges', []):
            map_obj.add_edge(edge_data['from'], edge_data['to'], 
                           edge_data.get('width', 1.0))
        
        # Load intersections
        map_obj.intersections = [tuple(inter) for inter in data.get('intersections', [])]
        
        # Load obstacles
        for obs in data.get('obstacles', []):
            map_obj.obstacles.append((obs['x'], obs['y'], obs.get('radius', 0.5)))
        
        # Load spawn points and destinations
        map_obj.spawn_points = [(sp['x'], sp['y']) for sp in data.get('spawn_points', [])]
        map_obj.destinations = [(dest['x'], dest['y']) for dest in data.get('destinations', [])]
        
        # Calculate bounds
        if map_obj.nodes:
            xs = [node.x for node in map_obj.nodes.values()]
            ys = [node.y for node in map_obj.nodes.values()]
            map_obj.bounds = (min(xs) - 5, min(ys) - 5, max(xs) + 5, max(ys) + 5)
        
        return map_obj

