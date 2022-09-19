import pygame
import numpy as np
from typing import List, Dict, Tuple, Optional
from simulation.map import Map
from vehicles.vehicle import Vehicle, Intent
from v2v.message import V2VMessage


class Renderer:
    """2D visualization renderer for the simulation"""
    
    def __init__(self, width: int = 1200, height: int = 800, scale: float = 1.0):
        pygame.init()
        self.width = width
        self.height = height
        self.scale = scale
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("SafeWay - V2V Coordination Simulation")
        self.clock = pygame.time.Clock()
        
        # Colors
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.GRAY = (128, 128, 128)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)
        self.YELLOW = (255, 255, 0)
        self.ORANGE = (255, 165, 0)
        self.PURPLE = (128, 0, 128)
        
        # Vehicle colors
        self.vehicle_colors = [
            (255, 100, 100), (100, 255, 100), (100, 100, 255),
            (255, 255, 100), (255, 100, 255), (100, 255, 255)
        ]
        
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        
        # Camera/viewport
        self.offset_x = 0
        self.offset_y = 0
        self.zoom = 1.0
    
    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to screen coordinates"""
        screen_x = int((x + self.offset_x) * self.zoom * self.scale + self.width / 2)
        screen_y = int((y + self.offset_y) * self.zoom * self.scale + self.height / 2)
        return (screen_x, screen_y)
    
    def screen_to_world(self, screen_x: int, screen_y: int) -> Tuple[float, float]:
        """Convert screen coordinates to world coordinates"""
        x = (screen_x - self.width / 2) / (self.zoom * self.scale) - self.offset_x
        y = (screen_y - self.height / 2) / (self.zoom * self.scale) - self.offset_y
        return (x, y)
    
    def center_on_map(self, map_obj: Map):
        """Center camera on map"""
        if map_obj.bounds:
            min_x, min_y, max_x, max_y = map_obj.bounds
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2
            self.offset_x = -center_x
            self.offset_y = -center_y
            
            # Auto-zoom to fit map
            map_width = max_x - min_x
            map_height = max_y - min_y
            zoom_x = self.width / (map_width * self.scale) * 0.8
            zoom_y = self.height / (map_height * self.scale) * 0.8
            self.zoom = min(zoom_x, zoom_y)
    
    def draw_map(self, map_obj: Map):
        """Draw the road network"""
        # Draw edges (roads)
        for edge in map_obj.edges:
            from_pos = map_obj.get_node_position(edge.from_node)
            to_pos = map_obj.get_node_position(edge.to_node)
            if from_pos and to_pos:
                start = self.world_to_screen(from_pos[0], from_pos[1])
                end = self.world_to_screen(to_pos[0], to_pos[1])
                pygame.draw.line(self.screen, self.GRAY, start, end, 3)
        
        # Draw nodes (intersections)
        for node_id, node in map_obj.nodes.items():
            pos = self.world_to_screen(node.x, node.y)
            pygame.draw.circle(self.screen, self.BLACK, pos, 5)
        
        # Draw intersections (highlight)
        for intersection in map_obj.intersections:
            for node_id in intersection:
                pos = map_obj.get_node_position(node_id)
                if pos:
                    screen_pos = self.world_to_screen(pos[0], pos[1])
                    pygame.draw.circle(self.screen, self.ORANGE, screen_pos, 8, 2)
        
        # Draw obstacles
        for obs_x, obs_y, radius in map_obj.obstacles:
            pos = self.world_to_screen(obs_x, obs_y)
            pygame.draw.circle(self.screen, self.RED, pos, int(radius * self.zoom * self.scale))
    
    def draw_vehicle(self, vehicle: Vehicle, show_trajectory: bool = True,
                    show_intent: bool = True):
        """Draw a vehicle"""
        color = self.vehicle_colors[vehicle.id % len(self.vehicle_colors)]
        pos = self.world_to_screen(vehicle.x, vehicle.y)
        
        # Draw vehicle body (rectangle oriented by heading)
        size = 8
        cos_h = np.cos(vehicle.heading)
        sin_h = np.sin(vehicle.heading)
        
        corners = [
            (pos[0] + size * cos_h - size * sin_h, pos[1] + size * sin_h + size * cos_h),
            (pos[0] + size * cos_h + size * sin_h, pos[1] + size * sin_h - size * cos_h),
            (pos[0] - size * cos_h + size * sin_h, pos[1] - size * sin_h - size * cos_h),
            (pos[0] - size * cos_h - size * sin_h, pos[1] - size * sin_h + size * cos_h)
        ]
        pygame.draw.polygon(self.screen, color, corners)
        pygame.draw.polygon(self.screen, self.BLACK, corners, 2)
        
        # Draw planned path
        if vehicle.path and show_trajectory:
            path_points = [self.world_to_screen(p[0], p[1]) for p in vehicle.path]
            if len(path_points) > 1:
                pygame.draw.lines(self.screen, (color[0]//2, color[1]//2, color[2]//2), 
                                False, path_points, 2)
        
        # Draw planned trajectory (short-term)
        if vehicle.planned_trajectory and show_trajectory:
            traj_points = [self.world_to_screen(p[0], p[1]) 
                          for p in vehicle.planned_trajectory[:5]]
            if len(traj_points) > 1:
                pygame.draw.lines(self.screen, self.YELLOW, False, traj_points, 1)
        
        # Draw intent indicator
        if show_intent:
            intent_offset = 15
            intent_pos = (pos[0] + intent_offset * cos_h, 
                         pos[1] + intent_offset * sin_h)
            
            if vehicle.intent == Intent.LEFT:
                pygame.draw.circle(self.screen, self.BLUE, 
                                 (int(intent_pos[0]), int(intent_pos[1])), 4)
            elif vehicle.intent == Intent.RIGHT:
                pygame.draw.circle(self.screen, self.GREEN, 
                                 (int(intent_pos[0]), int(intent_pos[1])), 4)
            elif vehicle.intent == Intent.YIELD:
                pygame.draw.circle(self.screen, self.ORANGE, 
                                 (int(intent_pos[0]), int(intent_pos[1])), 4)
            elif vehicle.intent == Intent.STOP:
                pygame.draw.circle(self.screen, self.RED, 
                                 (int(intent_pos[0]), int(intent_pos[1])), 4)
        
        # Draw vehicle ID
        id_text = self.small_font.render(str(vehicle.id), True, self.BLACK)
        self.screen.blit(id_text, (pos[0] + 10, pos[1] - 10))
    
    def draw_v2v_connections(self, vehicle: Vehicle, nearby_vehicles: List[V2VMessage],
                            comm_radius: float):
        """Draw V2V communication links"""
        vehicle_pos = self.world_to_screen(vehicle.x, vehicle.y)
        
        for msg in nearby_vehicles:
            other_pos = self.world_to_screen(msg.position[0], msg.position[1])
            dist = np.sqrt((vehicle.x - msg.position[0])**2 + 
                          (vehicle.y - msg.position[1])**2)
            
            if dist <= comm_radius:
                # Draw communication link (dashed line)
                pygame.draw.line(self.screen, (200, 200, 255), 
                               vehicle_pos, other_pos, 1)
    
    def draw_ui(self, time: float, step: int, mode: str, num_vehicles: int):
        """Draw UI overlay"""
        # Background panel
        panel_rect = pygame.Rect(10, 10, 300, 120)
        pygame.draw.rect(self.screen, (240, 240, 240, 200), panel_rect)
        pygame.draw.rect(self.screen, self.BLACK, panel_rect, 2)
        
        # Text
        y_offset = 20
        time_text = self.font.render(f"Time: {time:.2f}s", True, self.BLACK)
        self.screen.blit(time_text, (20, y_offset))
        
        step_text = self.font.render(f"Step: {step}", True, self.BLACK)
        self.screen.blit(step_text, (20, y_offset + 25))
        
        mode_text = self.font.render(f"Mode: {mode}", True, self.BLACK)
        self.screen.blit(mode_text, (20, y_offset + 50))
        
        vehicles_text = self.font.render(f"Vehicles: {num_vehicles}", True, self.BLACK)
        self.screen.blit(vehicles_text, (20, y_offset + 75))
        
        # Legend
        legend_y = self.height - 150
        legend_rect = pygame.Rect(10, legend_y, 200, 140)
        pygame.draw.rect(self.screen, (240, 240, 240, 200), legend_rect)
        pygame.draw.rect(self.screen, self.BLACK, legend_rect, 2)
        
        legend_text = self.small_font.render("Intent Colors:", True, self.BLACK)
        self.screen.blit(legend_text, (20, legend_y + 10))
        
        pygame.draw.circle(self.screen, self.BLUE, (30, legend_y + 35), 4)
        self.screen.blit(self.small_font.render("Left", True, self.BLACK), (40, legend_y + 30))
        
        pygame.draw.circle(self.screen, self.GREEN, (30, legend_y + 55), 4)
        self.screen.blit(self.small_font.render("Right", True, self.BLACK), (40, legend_y + 50))
        
        pygame.draw.circle(self.screen, self.ORANGE, (30, legend_y + 75), 4)
        self.screen.blit(self.small_font.render("Yield", True, self.BLACK), (40, legend_y + 70))
        
        pygame.draw.circle(self.screen, self.RED, (30, legend_y + 95), 4)
        self.screen.blit(self.small_font.render("Stop", True, self.BLACK), (40, legend_y + 90))
    
    def clear(self):
        """Clear the screen"""
        self.screen.fill(self.WHITE)
    
    def update(self):
        """Update display"""
        pygame.display.flip()
    
    def handle_events(self) -> bool:
        """Handle pygame events, return False if should quit"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
        return True
    
    def tick(self, fps: int = 60):
        """Tick the clock"""
        self.clock.tick(fps)

