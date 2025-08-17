#!/usr/bin/env python3
"""
Map Visualizer Component
สำหรับแสดงผลแผนที่และข้อมูลหุ่นยนต์
"""

import numpy as np
import math
from typing import List, Dict, Tuple, Optional

class MapVisualizer:
    """Class สำหรับจัดการการแสดงผลแผนที่"""
    
    def __init__(self, map_size: int = 30, cell_size: float = 5.0):
        self.map_size = map_size
        self.cell_size = cell_size
        self.map_data = np.full((map_size, map_size), -1, dtype=np.int8)
        
        # Color scheme
        self.colors = {
            'unknown': '#404040',
            'free': '#90EE90',
            'obstacle': '#8B0000',
            'robot': '#FF0000',
            'waypoint': '#FFD700',
            'waypoint_completed': '#00FF00',
            'path': '#0000FF',
            'sensor_beam': '#FFA500',
            'grid': '#606060'
        }
        
    def update_map_cell(self, x: int, y: int, value: int):
        """อัพเดทค่าของ cell ในแผนที่"""
        if 0 <= x < self.map_size and 0 <= y < self.map_size:
            self.map_data[y, x] = value
            
    def update_map_from_data(self, map_data: np.ndarray):
        """อัพเดทแผนที่จากข้อมูลที่ได้รับ"""
        if map_data.shape == self.map_data.shape:
            self.map_data = map_data.copy()
            
    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """แปลงพิกัดโลกเป็นพิกัด grid"""
        grid_x = int(world_x / self.cell_size + self.map_size // 2)
        grid_y = int(world_y / self.cell_size + self.map_size // 2)
        return grid_x, grid_y
        
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """แปลงพิกัด grid เป็นพิกัดโลก"""
        world_x = (grid_x - self.map_size // 2) * self.cell_size
        world_y = (grid_y - self.map_size // 2) * self.cell_size
        return world_x, world_y
        
    def canvas_to_world(self, canvas_x: int, canvas_y: int, display_size: int) -> Tuple[float, float]:
        """แปลงพิกัด canvas เป็นพิกัดโลก"""
        # Convert canvas coordinates to grid coordinates
        grid_x = int(canvas_x / (display_size / self.map_size))
        grid_y = int(canvas_y / (display_size / self.map_size))
        
        # Convert to world coordinates
        return self.grid_to_world(grid_x, grid_y)
        
    def world_to_canvas(self, world_x: float, world_y: float, display_size: int) -> Tuple[int, int]:
        """แปลงพิกัดโลกเป็นพิกัด canvas"""
        grid_x, grid_y = self.world_to_grid(world_x, world_y)
        
        cell_size_pixels = display_size / self.map_size
        canvas_x = int(grid_x * cell_size_pixels)
        canvas_y = int(grid_y * cell_size_pixels)
        
        return canvas_x, canvas_y
        
    def is_valid_grid_position(self, grid_x: int, grid_y: int) -> bool:
        """ตรวจสอบว่าตำแหน่ง grid ถูกต้องหรือไม่"""
        return 0 <= grid_x < self.map_size and 0 <= grid_y < self.map_size
        
    def get_map_cell_color(self, grid_x: int, grid_y: int) -> str:
        """ได้สีของ cell ตามค่าใน map"""
        if not self.is_valid_grid_position(grid_x, grid_y):
            return self.colors['unknown']
            
        cell_value = self.map_data[grid_y, grid_x]
        
        if cell_value == -1:  # Unknown
            return self.colors['unknown']
        elif cell_value == 0:  # Free
            return self.colors['free']
        else:  # Obstacle
            return self.colors['obstacle']
            
    def draw_grid(self, canvas, display_size: int, show_coordinates: bool = False):
        """วาดเส้น grid บน canvas"""
        cell_size_pixels = display_size / self.map_size
        
        # Draw map cells
        for x in range(self.map_size):
            for y in range(self.map_size):
                x1 = x * cell_size_pixels
                y1 = y * cell_size_pixels
                x2 = x1 + cell_size_pixels
                y2 = y1 + cell_size_pixels
                
                color = self.get_map_cell_color(x, y)
                
                canvas.create_rectangle(x1, y1, x2, y2, 
                                       fill=color, 
                                       outline=self.colors['grid'], 
                                       width=0.5)
                
                # Show coordinates if requested
                if show_coordinates and (x % 5 == 0 and y % 5 == 0):
                    world_x, world_y = self.grid_to_world(x, y)
                    canvas.create_text(x1 + cell_size_pixels/2, y1 + cell_size_pixels/2,
                                      text=f"{world_x:.0f},{world_y:.0f}",
                                      fill='white', font=('Arial', 6))
                                      
    def draw_robot(self, canvas, robot_x: float, robot_y: float, heading: float, 
                   display_size: int, show_sensors: bool = True, sensor_data: List[float] = None):
        """วาดหุ่นยนต์บน canvas"""
        canvas_x, canvas_y = self.world_to_canvas(robot_x, robot_y, display_size)
        cell_size_pixels = display_size / self.map_size
        robot_size = cell_size_pixels * 1.5
        
        # Check if robot is within canvas bounds
        if canvas_x < 0 or canvas_x > display_size or canvas_y < 0 or canvas_y > display_size:
            return
            
        # Draw robot circle
        canvas.create_oval(canvas_x - robot_size/2, canvas_y - robot_size/2,
                          canvas_x + robot_size/2, canvas_y + robot_size/2,
                          fill=self.colors['robot'], outline='white', width=2)
                          
        # Draw heading indicator
        heading_rad = math.radians(heading)
        hx = canvas_x + robot_size * math.cos(heading_rad)
        hy = canvas_y + robot_size * math.sin(heading_rad)
        
        canvas.create_line(canvas_x, canvas_y, hx, hy, 
                          fill='white', width=3,
                          arrow='last', arrowshape=(8, 10, 4))
                          
        # Draw sensor beams if requested
        if show_sensors and sensor_data and len(sensor_data) >= 4:
            self.draw_sensor_beams(canvas, robot_x, robot_y, heading, 
                                  sensor_data, display_size)
                                  
    def draw_sensor_beams(self, canvas, robot_x: float, robot_y: float, heading: float,
                         sensor_data: List[float], display_size: int):
        """วาด sensor beams"""
        sensor_angles = [0, 90, 180, 270]  # Front, Right, Back, Left
        sensor_colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4']
        
        robot_canvas_x, robot_canvas_y = self.world_to_canvas(robot_x, robot_y, display_size)
        
        for i, (angle_offset, color, distance) in enumerate(zip(sensor_angles, sensor_colors, sensor_data)):
            if distance > 0 and distance < 200:  # Valid reading
                total_angle = heading + angle_offset
                angle_rad = math.radians(total_angle)
                
                # Calculate end point of beam
                end_world_x = robot_x + distance * math.cos(angle_rad)
                end_world_y = robot_y + distance * math.sin(angle_rad)
                end_canvas_x, end_canvas_y = self.world_to_canvas(end_world_x, end_world_y, display_size)
                
                # Draw beam line
                canvas.create_line(robot_canvas_x, robot_canvas_y, 
                                  end_canvas_x, end_canvas_y,
                                  fill=color, width=2, dash=(5, 5))
                                  
                # Draw obstacle point
                canvas.create_oval(end_canvas_x-3, end_canvas_y-3, 
                                  end_canvas_x+3, end_canvas_y+3,
                                  fill=color, outline='white')
                                  
    def draw_waypoints(self, canvas, waypoints: List[Dict], display_size: int, 
                      show_numbers: bool = True, show_path: bool = True):
        """วาด waypoints บน canvas"""
        if not waypoints:
            return
            
        canvas_points = []
        
        for i, wp in enumerate(waypoints):
            canvas_x, canvas_y = self.world_to_canvas(wp['x'], wp['y'], display_size)
            canvas_points.extend([canvas_x, canvas_y])
            
            # Check if waypoint is within bounds
            if 0 <= canvas_x <= display_size and 0 <= canvas_y <= display_size:
                # Choose color based on completion status
                color = self.colors['waypoint_completed'] if wp.get('completed', False) else self.colors['waypoint']
                
                # Draw waypoint marker
                canvas.create_oval(canvas_x-8, canvas_y-8, canvas_x+8, canvas_y+8,
                                  fill=color, outline='white', width=2)
                
                # Draw waypoint number
                if show_numbers:
                    canvas.create_text(canvas_x, canvas_y, text=str(i+1), 
                                      fill='black', font=('Arial', 8, 'bold'))
                                      
        # Draw path between waypoints
        if show_path and len(canvas_points) >= 4:
            canvas.create_line(canvas_points, fill=self.colors['path'], 
                              width=2, dash=(5, 5))
                              
    def draw_path_history(self, canvas, path_points: List[Tuple[float, float]], 
                         display_size: int, max_points: int = 100):
        """วาดเส้นทางที่หุ่นยนต์เดินมา"""
        if len(path_points) < 2:
            return
            
        # Limit number of points to avoid performance issues
        if len(path_points) > max_points:
            path_points = path_points[-max_points:]
            
        canvas_points = []
        for world_x, world_y in path_points:
            canvas_x, canvas_y = self.world_to_canvas(world_x, world_y, display_size)
            canvas_points.extend([canvas_x, canvas_y])
            
        if len(canvas_points) >= 4:
            canvas.create_line(canvas_points, fill='#ADD8E6', width=1, smooth=True)
            
    def get_map_statistics(self) -> Dict[str, int]:
        """คำนวณสถิติของแผนที่"""
        unknown_cells = np.sum(self.map_data == -1)
        free_cells = np.sum(self.map_data == 0)
        obstacle_cells = np.sum(self.map_data == 1)
        total_cells = self.map_size * self.map_size
        
        return {
            'total_cells': total_cells,
            'unknown_cells': unknown_cells,
            'free_cells': free_cells,
            'obstacle_cells': obstacle_cells,
            'known_cells': free_cells + obstacle_cells,
            'completion_percentage': ((free_cells + obstacle_cells) / total_cells) * 100
        }
        
    def find_closest_waypoint(self, world_x: float, world_y: float, 
                             waypoints: List[Dict], max_distance: float = 20.0) -> Optional[int]:
        """หา waypoint ที่ใกล้ที่สุดกับตำแหน่งที่กำหนด"""
        min_distance = float('inf')
        closest_index = None
        
        for i, wp in enumerate(waypoints):
            distance = math.sqrt((wp['x'] - world_x)**2 + (wp['y'] - world_y)**2)
            if distance < min_distance and distance <= max_distance:
                min_distance = distance
                closest_index = i
                
        return closest_index
        
    def is_position_free(self, world_x: float, world_y: float, 
                        safety_margin: float = 10.0) -> bool:
        """ตรวจสอบว่าตำแหน่งที่กำหนดเป็นพื้นที่ว่างหรือไม่"""
        grid_x, grid_y = self.world_to_grid(world_x, world_y)
        
        # Check if position is within map bounds
        if not self.is_valid_grid_position(grid_x, grid_y):
            return False
            
        # Check main cell
        if self.map_data[grid_y, grid_x] == 1:  # Obstacle
            return False
            
        # Check surrounding cells for safety margin
        margin_cells = int(safety_margin / self.cell_size)
        for dx in range(-margin_cells, margin_cells + 1):
            for dy in range(-margin_cells, margin_cells + 1):
                check_x = grid_x + dx
                check_y = grid_y + dy
                
                if self.is_valid_grid_position(check_x, check_y):
                    if self.map_data[check_y, check_x] == 1:  # Obstacle
                        return False
                        
        return True
        
    def clear_map(self):
        """ล้างข้อมูลแผนที่"""
        self.map_data.fill(-1)
        
    def export_map_data(self) -> Dict:
        """ส่งออกข้อมูลแผนที่"""
        return {
            'map_size': self.map_size,
            'cell_size': self.cell_size,
            'map_data': self.map_data.tolist(),
            'statistics': self.get_map_statistics()
        }
        
    def import_map_data(self, data: Dict):
        """นำเข้าข้อมูลแผนที่"""
        if data.get('map_size') == self.map_size:
            self.map_data = np.array(data['map_data'], dtype=np.int8)
            self.cell_size = data.get('cell_size', self.cell_size)
            
    def create_mini_map(self, center_x: float, center_y: float, 
                       radius: int = 10) -> np.ndarray:
        """สร้าง mini map รอบๆ ตำแหน่งที่กำหนด"""
        center_grid_x, center_grid_y = self.world_to_grid(center_x, center_y)
        
        mini_map = np.full((radius*2 + 1, radius*2 + 1), -1, dtype=np.int8)
        
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                source_x = center_grid_x + dx
                source_y = center_grid_y + dy
                target_x = dx + radius
                target_y = dy + radius
                
                if self.is_valid_grid_position(source_x, source_y):
                    mini_map[target_y, target_x] = self.map_data[source_y, source_x]
                    
        return mini_map