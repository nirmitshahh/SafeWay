import numpy as np
from typing import List, Tuple, Optional, Dict
from simulation.map import Map
import heapq


class PathFinder:
    """A* pathfinding algorithm for vehicles"""
    
    def __init__(self, map_obj: Map):
        self.map = map_obj
    
    def heuristic(self, node1_id: int, node2_id: int) -> float:
        """Heuristic function for A* (Euclidean distance)"""
        return self.map.distance(node1_id, node2_id)
    
    def find_closest_node(self, x: float, y: float) -> Optional[int]:
        """Find the closest node to a given position"""
        min_dist = float('inf')
        closest_node = None
        
        for node_id, node in self.map.nodes.items():
            dist = np.sqrt((node.x - x)**2 + (node.y - y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_node = node_id
        
        return closest_node
    
    def find_path(self, start_pos: Tuple[float, float], 
                  end_pos: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Find path from start to end using A* algorithm
        Returns list of (x, y) positions
        """
        start_node = self.find_closest_node(start_pos[0], start_pos[1])
        end_node = self.find_closest_node(end_pos[0], end_pos[1])
        
        if start_node is None or end_node is None:
            return []
        
        if start_node == end_node:
            return [end_pos]
        
        # A* algorithm
        # Use priority queue for open set
        open_set = [(0, start_node)]
        came_from: Dict[int, Optional[int]] = {start_node: None}
        g_score: Dict[int, float] = {start_node: 0}
        f_score: Dict[int, float] = {start_node: self.heuristic(start_node, end_node)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == end_node:
                # Reconstruct path
                path_nodes = []
                node = current
                while node is not None:
                    pos = self.map.get_node_position(node)
                    if pos:
                        path_nodes.append(pos)
                    node = came_from.get(node)
                path_nodes.reverse()
                
                # Add exact end position
                path_nodes[-1] = end_pos
                return path_nodes
            
            for neighbor_id in self.map.get_neighbors(current):
                tentative_g = g_score[current] + self.map.distance(current, neighbor_id)
                
                if neighbor_id not in g_score or tentative_g < g_score[neighbor_id]:
                    came_from[neighbor_id] = current
                    g_score[neighbor_id] = tentative_g
                    f_score[neighbor_id] = tentative_g + self.heuristic(neighbor_id, end_node)
                    heapq.heappush(open_set, (f_score[neighbor_id], neighbor_id))
        
        # No path found
        return []
    
    def find_path_smooth(self, start_pos: Tuple[float, float], 
                        end_pos: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Find path and smooth it"""
        path = self.find_path(start_pos, end_pos)
        if path:
            return self.smooth_path(path)
        return path
    
    def smooth_path(self, path: List[Tuple[float, float]], 
                   lookahead: int = 3) -> List[Tuple[float, float]]:
        """
        Smooth the path by removing unnecessary waypoints
        Uses simple line-of-sight checking
        """
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # Try to skip ahead
            for j in range(min(i + lookahead, len(path) - 1), i, -1):
                if self._has_line_of_sight(path[i], path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
            else:
                i += 1
                if i < len(path):
                    smoothed.append(path[i])
        
        return smoothed
    
    def _has_line_of_sight(self, pos1: Tuple[float, float], 
                          pos2: Tuple[float, float]) -> bool:
        """Check if there's a clear line of sight between two positions"""
        # Simple check - could be enhanced with obstacle checking
        # For now, just return True (assume road network is connected)
        return True

