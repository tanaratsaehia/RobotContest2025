#!/usr/bin/env python3
"""
Real-time Robot Mapping Visualization System
แสดงแผนที่และข้อมูลหุ่นยนต์แบบเรียลไทม์
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import numpy as np
import threading
import queue
import time
from datetime import datetime
import json
import struct
from collections import deque
import math

class RealTimeMappingUI:
    def __init__(self):
        # ================================
        # Configuration
        # ================================
        self.MAP_SIZE = 100  # Grid size
        self.CELL_SIZE = 10  # cm per cell
        self.DISPLAY_SIZE = 800  # Display window size in pixels
        self.GRID_SCALE = self.DISPLAY_SIZE / self.MAP_SIZE
        
        # Serial configuration
        self.ser = None
        self.serial_thread = None
        self.running = False
        self.data_queue = queue.Queue()
        
        # Map data
        self.map_data = np.full((self.MAP_SIZE, self.MAP_SIZE), -1, dtype=np.int8)
        self.confidence_map = np.zeros((self.MAP_SIZE, self.MAP_SIZE), dtype=np.uint8)
        
        # Robot state
        self.robot_x = self.MAP_SIZE * self.CELL_SIZE / 2
        self.robot_y = self.MAP_SIZE * self.CELL_SIZE / 2
        self.robot_heading = 0
        self.sensor_distances = [0, 0, 0, 0]  # F, R, B, L
        self.sensor_valid = [False, False, False, False]
        
        # Path history
        self.path_history = deque(maxlen=1000)
        self.explored_area = 0
        
        # UI colors
        self.COLORS = {
            'unknown': '#404040',
            'free': '#90EE90',
            'occupied': '#8B0000',
            'robot': '#FF0000',
            'path': '#0000FF',
            'sensor_beam': '#FFD700',
            'grid': '#606060',
            'background': '#2C2C2C'
        }
        
        # Statistics
        self.stats = {
            'packets_received': 0,
            'last_update': time.time(),
            'mapping_time': 0,
            'obstacles_found': 0
        }
        
        # Create UI
        self.setup_ui()
        
    def setup_ui(self):
        """สร้าง GUI หลัก"""
        self.root = tk.Tk()
        self.root.title("🗺️ Real-time Robot Mapping System")
        self.root.configure(bg=self.COLORS['background'])
        
        # Set window size and position
        window_width = 1200
        window_height = 900
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        x = (screen_width - window_width) // 2
        y = (screen_height - window_height) // 2
        self.root.geometry(f"{window_width}x{window_height}+{x}+{y}")
        
        # Create main frames
        self.create_control_panel()
        self.create_map_display()
        self.create_info_panel()
        self.create_status_bar()
        
        # Bind keyboard events
        self.root.bind('<KeyPress>', self.on_key_press)
        
        # Start update loop
        self.update_display()
        
    def create_control_panel(self):
        """สร้าง Control Panel"""
        control_frame = tk.Frame(self.root, bg=self.COLORS['background'])
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        # Connection controls
        conn_frame = tk.LabelFrame(control_frame, text="🔌 Connection", 
                                  bg=self.COLORS['background'], fg='white',
                                  font=('Arial', 10, 'bold'))
        conn_frame.pack(side=tk.LEFT, padx=5)
        
        # Port selection
        tk.Label(conn_frame, text="Port:", bg=self.COLORS['background'], 
                fg='white').grid(row=0, column=0, padx=5)
        
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, 
                                       width=15, state='readonly')
        self.port_combo.grid(row=0, column=1, padx=5)
        
        # Buttons
        self.scan_btn = tk.Button(conn_frame, text="🔍 Scan", 
                                 command=self.scan_ports,
                                 bg='#4CAF50', fg='white', font=('Arial', 9))
        self.scan_btn.grid(row=0, column=2, padx=5)
        
        self.connect_btn = tk.Button(conn_frame, text="📡 Connect", 
                                    command=self.toggle_connection,
                                    bg='#2196F3', fg='white', font=('Arial', 9))
        self.connect_btn.grid(row=0, column=3, padx=5)
        
        # Mapping controls
        map_frame = tk.LabelFrame(control_frame, text="🗺️ Mapping", 
                                 bg=self.COLORS['background'], fg='white',
                                 font=('Arial', 10, 'bold'))
        map_frame.pack(side=tk.LEFT, padx=20)
        
        tk.Button(map_frame, text="▶️ Start", command=self.start_mapping,
                 bg='#4CAF50', fg='white', width=10).pack(side=tk.LEFT, padx=5)
        
        tk.Button(map_frame, text="⏸️ Pause", command=self.pause_mapping,
                 bg='#FF9800', fg='white', width=10).pack(side=tk.LEFT, padx=5)
        
        tk.Button(map_frame, text="⏹️ Stop", command=self.stop_mapping,
                 bg='#F44336', fg='white', width=10).pack(side=tk.LEFT, padx=5)
        
        tk.Button(map_frame, text="🔄 Clear Map", command=self.clear_map,
                 bg='#9C27B0', fg='white', width=10).pack(side=tk.LEFT, padx=5)
        
        tk.Button(map_frame, text="💾 Save Map", command=self.save_map,
                 bg='#607D8B', fg='white', width=10).pack(side=tk.LEFT, padx=5)
        
        # Robot control
        control_frame2 = tk.LabelFrame(control_frame, text="🤖 Manual Control", 
                                      bg=self.COLORS['background'], fg='white',
                                      font=('Arial', 10, 'bold'))
        control_frame2.pack(side=tk.LEFT, padx=20)
        
        # Create arrow button layout
        btn_frame = tk.Frame(control_frame2, bg=self.COLORS['background'])
        btn_frame.pack()
        
        # Forward button
        tk.Button(btn_frame, text="⬆", command=lambda: self.send_command('w'),
                 bg='#4CAF50', fg='white', font=('Arial', 16), 
                 width=3, height=1).grid(row=0, column=1, padx=2, pady=2)
        
        # Left, Stop, Right buttons
        tk.Button(btn_frame, text="⬅", command=lambda: self.send_command('a'),
                 bg='#4CAF50', fg='white', font=('Arial', 16), 
                 width=3, height=1).grid(row=1, column=0, padx=2, pady=2)
        
        tk.Button(btn_frame, text="⏹", command=lambda: self.send_command('x'),
                 bg='#F44336', fg='white', font=('Arial', 16), 
                 width=3, height=1).grid(row=1, column=1, padx=2, pady=2)
        
        tk.Button(btn_frame, text="➡", command=lambda: self.send_command('d'),
                 bg='#4CAF50', fg='white', font=('Arial', 16), 
                 width=3, height=1).grid(row=1, column=2, padx=2, pady=2)
        
        # Backward button
        tk.Button(btn_frame, text="⬇", command=lambda: self.send_command('s'),
                 bg='#4CAF50', fg='white', font=('Arial', 16), 
                 width=3, height=1).grid(row=2, column=1, padx=2, pady=2)
        
    def create_map_display(self):
        """สร้างหน้าจอแสดงแผนที่"""
        main_frame = tk.Frame(self.root, bg=self.COLORS['background'])
        main_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Map canvas
        self.canvas = tk.Canvas(main_frame, width=self.DISPLAY_SIZE, 
                               height=self.DISPLAY_SIZE,
                               bg=self.COLORS['unknown'], 
                               highlightthickness=2,
                               highlightbackground='white')
        self.canvas.pack()
        
        # Initialize map grid
        self.map_image = tk.PhotoImage(width=self.DISPLAY_SIZE, 
                                       height=self.DISPLAY_SIZE)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.map_image)
        
        # Robot indicator
        self.robot_marker = None
        self.robot_heading_line = None
        self.sensor_lines = []
        
    def create_info_panel(self):
        """สร้าง Information Panel"""
        info_frame = tk.Frame(self.root, bg=self.COLORS['background'])
        info_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        
        # Robot Status
        status_frame = tk.LabelFrame(info_frame, text="🤖 Robot Status",
                                    bg=self.COLORS['background'], fg='white',
                                    font=('Arial', 11, 'bold'))
        status_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.status_labels = {}
        status_items = [
            ('Position X', '0.0 cm'),
            ('Position Y', '0.0 cm'),
            ('Heading', '0.0°'),
            ('Mode', 'IDLE'),
            ('Battery', 'N/A')
        ]
        
        for i, (label, value) in enumerate(status_items):
            tk.Label(status_frame, text=f"{label}:", 
                    bg=self.COLORS['background'], fg='white',
                    font=('Arial', 9)).grid(row=i, column=0, sticky='w', padx=5)
            self.status_labels[label] = tk.Label(status_frame, text=value,
                                                bg=self.COLORS['background'], 
                                                fg='#00FF00',
                                                font=('Arial', 9, 'bold'))
            self.status_labels[label].grid(row=i, column=1, sticky='w', padx=5)
        
        # Sensor Readings
        sensor_frame = tk.LabelFrame(info_frame, text="📡 Sensors (cm)",
                                    bg=self.COLORS['background'], fg='white',
                                    font=('Arial', 11, 'bold'))
        sensor_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.sensor_labels = {}
        sensors = ['Front', 'Right', 'Back', 'Left']
        colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4']
        
        for i, (sensor, color) in enumerate(zip(sensors, colors)):
            tk.Label(sensor_frame, text=f"{sensor}:", 
                    bg=self.COLORS['background'], fg='white',
                    font=('Arial', 9)).grid(row=i, column=0, sticky='w', padx=5)
            self.sensor_labels[sensor] = tk.Label(sensor_frame, text="--",
                                                 bg=self.COLORS['background'], 
                                                 fg=color,
                                                 font=('Arial', 9, 'bold'))
            self.sensor_labels[sensor].grid(row=i, column=1, sticky='w', padx=5)
        
        # Map Statistics
        stats_frame = tk.LabelFrame(info_frame, text="📊 Statistics",
                                   bg=self.COLORS['background'], fg='white',
                                   font=('Arial', 11, 'bold'))
        stats_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.stats_labels = {}
        stats_items = [
            ('Explored', '0%'),
            ('Obstacles', '0'),
            ('Path Length', '0 cm'),
            ('Mapping Time', '00:00'),
            ('Packets', '0')
        ]
        
        for i, (label, value) in enumerate(stats_items):
            tk.Label(stats_frame, text=f"{label}:", 
                    bg=self.COLORS['background'], fg='white',
                    font=('Arial', 9)).grid(row=i, column=0, sticky='w', padx=5)
            self.stats_labels[label] = tk.Label(stats_frame, text=value,
                                               bg=self.COLORS['background'], 
                                               fg='#FFA500',
                                               font=('Arial', 9, 'bold'))
            self.stats_labels[label].grid(row=i, column=1, sticky='w', padx=5)
        
        # Log display
        log_frame = tk.LabelFrame(info_frame, text="📝 Activity Log",
                                 bg=self.COLORS['background'], fg='white',
                                 font=('Arial', 11, 'bold'))
        log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.log_text = tk.Text(log_frame, height=15, width=35,
                               bg='#1E1E1E', fg='#00FF00',
                               font=('Consolas', 8))
        scrollbar = tk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.log("System initialized")
        
    def create_status_bar(self):
        """สร้าง Status Bar"""
        self.status_bar = tk.Label(self.root, text="Ready", 
                                  bg='#333333', fg='white',
                                  anchor=tk.W, relief=tk.SUNKEN)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
    def scan_ports(self):
        """สแกนหา Serial ports"""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.set(port_list[0])
            self.log(f"Found {len(port_list)} ports")
            
            # Show port details
            for port in ports:
                if 'CH340' in port.description or 'CP210' in port.description:
                    self.log(f"ESP32 likely at {port.device}")
        else:
            self.log("No ports found")
            
    def toggle_connection(self):
        """เชื่อมต่อ/ตัดการเชื่อมต่อ Serial"""
        if not self.ser or not self.ser.is_open:
            self.connect_serial()
        else:
            self.disconnect_serial()
            
    def connect_serial(self):
        """เชื่อมต่อ Serial port"""
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Please select a port")
            return
            
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            self.running = True
            
            # Start serial reading thread
            self.serial_thread = threading.Thread(target=self.read_serial, 
                                                 daemon=True)
            self.serial_thread.start()
            
            self.connect_btn.config(text="🔌 Disconnect", bg='#F44336')
            self.status_bar.config(text=f"Connected to {port}")
            self.log(f"Connected to {port}")
            
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            self.log(f"Connection failed: {e}")
            
    def disconnect_serial(self):
        """ตัดการเชื่อมต่อ Serial"""
        self.running = False
        if self.ser:
            self.ser.close()
            
        self.connect_btn.config(text="📡 Connect", bg='#2196F3')
        self.status_bar.config(text="Disconnected")
        self.log("Disconnected")
        
    def read_serial(self):
        """อ่านข้อมูลจาก Serial (ทำงานใน thread แยก)"""
        buffer = ""
        
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting).decode('utf-8', 
                                                                     errors='ignore')
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            self.process_serial_data(line)
                            
            except Exception as e:
                if self.running:
                    self.log(f"Read error: {e}")
                    
            time.sleep(0.01)
            
    def process_serial_data(self, line):
        """ประมวลผลข้อมูลที่รับมา"""
        try:
            # Parse different message types
            if '[STATUS]' in line:
                self.parse_status_message(line)
            elif '[TELEMETRY]' in line:
                self.parse_telemetry_message(line)
            elif '[MAP]' in line:
                self.parse_map_message(line)
            elif line.startswith('{'):  # JSON format
                self.parse_json_message(line)
            else:
                # Log raw message
                if len(line) < 100:  # Don't log very long messages
                    self.log(f"RX: {line}")
                    
        except Exception as e:
            self.log(f"Parse error: {e}")
            
    def parse_status_message(self, line):
        """Parse status message format:
        [STATUS] Robot: (x,y) heading° | Sensors: Fx Rx Bx Lx | Mode: xxx
        """
        try:
            parts = line.split('|')
            
            # Parse position
            if 'Robot:' in parts[0]:
                pos_part = parts[0].split('Robot:')[1].strip()
                # Extract coordinates and heading
                import re
                match = re.search(r'\(([\d.]+),([\d.]+)\)\s+([\d.]+)°', pos_part)
                if match:
                    self.robot_x = float(match.group(1))
                    self.robot_y = float(match.group(2))
                    self.robot_heading = float(match.group(3))
                    
            # Parse sensors
            if len(parts) > 1 and 'Sensors:' in parts[1]:
                sensor_part = parts[1].split('Sensors:')[1].strip()
                values = re.findall(r'[FRBL]([\d.]+)', sensor_part)
                if len(values) >= 4:
                    self.sensor_distances = [float(v) for v in values]
                    self.sensor_valid = [v > 0 for v in self.sensor_distances]
                    
            # Update data queue
            self.data_queue.put({
                'type': 'status',
                'x': self.robot_x,
                'y': self.robot_y,
                'heading': self.robot_heading,
                'sensors': self.sensor_distances
            })
            
            self.stats['packets_received'] += 1
            
        except Exception as e:
            self.log(f"Status parse error: {e}")
            
    def parse_telemetry_message(self, line):
        """Parse telemetry messages"""
        # Extract battery voltage, IMU status, etc.
        if 'Bat:' in line:
            import re
            match = re.search(r'Bat:([\d.]+)V', line)
            if match:
                voltage = float(match.group(1))
                self.status_labels['Battery'].config(text=f"{voltage:.1f}V")
                
    def parse_map_message(self, line):
        """Parse map data messages"""
        # Handle compressed map data
        pass
        
    def parse_json_message(self, line):
        """Parse JSON formatted messages"""
        try:
            data = json.loads(line)
            
            if 'x' in data and 'y' in data:
                self.robot_x = data['x']
                self.robot_y = data['y']
                
            if 'heading' in data:
                self.robot_heading = data['heading']
                
            if 'sensors' in data:
                self.sensor_distances = data['sensors']
                
            if 'map' in data:
                # Update local map data
                self.update_map_from_data(data['map'])
                
        except json.JSONDecodeError:
            pass
            
    def update_display(self):
        """อัพเดทการแสดงผลแบบเรียลไทม์"""
        # Process queued data
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                
                if data['type'] == 'status':
                    # Update robot position
                    self.update_robot_display(data)
                    
                    # Update map based on sensors
                    self.update_map_from_sensors()
                    
                    # Add to path history
                    grid_x = int(data['x'] / self.CELL_SIZE)
                    grid_y = int(data['y'] / self.CELL_SIZE)
                    self.path_history.append((grid_x, grid_y))
                    
            except queue.Empty:
                break
                
        # Redraw map
        self.draw_map()
        
        # Update statistics
        self.update_statistics()
        
        # Schedule next update
        self.root.after(50, self.update_display)  # 20 FPS
        
    def draw_map(self):
        """วาดแผนที่บน Canvas"""
        # Create image data
        img_data = []
        
        for y in range(self.MAP_SIZE):
            row = []
            for x in range(self.MAP_SIZE):
                cell_value = self.map_data[y, x]
                
                if cell_value == -1:  # Unknown
                    color = self.COLORS['unknown']
                elif cell_value == 0:  # Free
                    color = self.COLORS['free']
                else:  # Occupied
                    color = self.COLORS['occupied']
                    
                row.append(color)
            img_data.append(row)
            
        # Update canvas
        self.canvas.delete("all")
        
        # Draw grid cells
        for y in range(self.MAP_SIZE):
            for x in range(self.MAP_SIZE):
                x1 = x * self.GRID_SCALE
                y1 = y * self.GRID_SCALE
                x2 = x1 + self.GRID_SCALE
                y2 = y1 + self.GRID_SCALE
                
                self.canvas.create_rectangle(x1, y1, x2, y2,
                                           fill=img_data[y][x],
                                           outline=self.COLORS['grid'],
                                           width=0.5)
                                           
        # Draw path history
        if len(self.path_history) > 1:
            points = []
            for x, y in self.path_history:
                px = x * self.GRID_SCALE + self.GRID_SCALE/2
                py = y * self.GRID_SCALE + self.GRID_SCALE/2
                points.extend([px, py])
                
            if len(points) > 2:
                self.canvas.create_line(points, fill=self.COLORS['path'], 
                                       width=2, smooth=True)
                                       
        # Draw robot
        self.draw_robot()
        
        # Draw sensor beams
        self.draw_sensor_beams()
        
    def draw_robot(self):
        """วาดตัวหุ่นยนต์"""
        # Convert robot position to display coordinates
        robot_grid_x = self.robot_x / self.CELL_SIZE
        robot_grid_y = self.robot_y / self.CELL_SIZE
        
        rx = robot_grid_x * self.GRID_SCALE
        ry = robot_grid_y * self.GRID_SCALE
        
        # Robot size
        robot_size = self.GRID_SCALE * 2
        
        # Draw robot circle
        self.canvas.create_oval(rx - robot_size/2, ry - robot_size/2,
                               rx + robot_size/2, ry + robot_size/2,
                               fill=self.COLORS['robot'], outline='white', width=2)
                               
        # Draw heading indicator
        heading_rad = math.radians(self.robot_heading)
        hx = rx + robot_size * math.cos(heading_rad)
        hy = ry + robot_size * math.sin(heading_rad)
        
        self.canvas.create_line(rx, ry, hx, hy, fill='white', width=3,
                               arrow=tk.LAST, arrowshape=(10, 12, 5))
                               
    def draw_sensor_beams(self):
        """วาด sensor beams"""
        if not all(self.sensor_valid):
            return
            
        robot_grid_x = self.robot_x / self.CELL_SIZE
        robot_grid_y = self.robot_y / self.CELL_SIZE
        
        rx = robot_grid_x * self.GRID_SCALE
        ry = robot_grid_y * self.GRID_SCALE
        
        sensor_angles = [0, 90, 180, 270]  # F, R, B, L
        sensor_colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4']
        
        for i, (angle, color, distance) in enumerate(zip(sensor_angles, 
                                                         sensor_colors, 
                                                         self.sensor_distances)):
            if self.sensor_valid[i] and distance > 0:
                # Calculate beam end point
                total_angle = math.radians(self.robot_heading + angle)
                beam_length = (distance / self.CELL_SIZE) * self.GRID_SCALE
                
                ex = rx + beam_length * math.cos(total_angle)
                ey = ry + beam_length * math.sin(total_angle)
                
                # Draw beam
                self.canvas.create_line(rx, ry, ex, ey, fill=color, 
                                       width=1, dash=(5, 5))
                                       
                # Draw obstacle point
                if distance < 400:  # Max sensor range
                    self.canvas.create_oval(ex-3, ey-3, ex+3, ey+3,
                                          fill=color, outline='white')
                                          
    def update_robot_display(self, data):
        """อัพเดทข้อมูลหุ่นยนต์บนหน้าจอ"""
        # Update position labels
        self.status_labels['Position X'].config(text=f"{data['x']:.1f} cm")
        self.status_labels['Position Y'].config(text=f"{data['y']:.1f} cm")
        self.status_labels['Heading'].config(text=f"{data['heading']:.1f}°")
        
        # Update sensor labels
        sensors = ['Front', 'Right', 'Back', 'Left']
        for i, sensor in enumerate(sensors):
            if i < len(data['sensors']):
                value = data['sensors'][i]
                if value > 0:
                    self.sensor_labels[sensor].config(text=f"{value:.1f}")
                else:
                    self.sensor_labels[sensor].config(text="--")
                    
    def update_map_from_sensors(self):
        """อัพเดทแผนที่จากข้อมูล sensors"""
        robot_grid_x = int(self.robot_x / self.CELL_SIZE)
        robot_grid_y = int(self.robot_y / self.CELL_SIZE)
        
        # Mark robot position as free
        if 0 <= robot_grid_x < self.MAP_SIZE and 0 <= robot_grid_y < self.MAP_SIZE:
            self.map_data[robot_grid_y, robot_grid_x] = 0
            self.confidence_map[robot_grid_y, robot_grid_x] = 255
            
        # Process each sensor
        sensor_angles = [0, 90, 180, 270]
        
        for i, (angle, distance) in enumerate(zip(sensor_angles, self.sensor_distances)):
            if not self.sensor_valid[i] or distance <= 0:
                continue
                
            # Calculate sensor beam direction
            total_angle = math.radians(self.robot_heading + angle)
            dx = math.cos(total_angle)
            dy = math.sin(total_angle)
            
            # Trace beam and mark cells
            max_distance = min(distance, 400)  # Max sensor range
            
            for d in range(0, int(max_distance), self.CELL_SIZE):
                # Calculate point along beam
                px = self.robot_x + d * dx
                py = self.robot_y + d * dy
                
                gx = int(px / self.CELL_SIZE)
                gy = int(py / self.CELL_SIZE)
                
                if 0 <= gx < self.MAP_SIZE and 0 <= gy < self.MAP_SIZE:
                    # Mark as free space
                    self.map_data[gy, gx] = 0
                    self.confidence_map[gy, gx] = min(255, self.confidence_map[gy, gx] + 10)
                    
            # Mark obstacle point
            if distance < 400:  # Valid obstacle detection
                obs_x = self.robot_x + distance * dx
                obs_y = self.robot_y + distance * dy
                
                obs_gx = int(obs_x / self.CELL_SIZE)
                obs_gy = int(obs_y / self.CELL_SIZE)
                
                if 0 <= obs_gx < self.MAP_SIZE and 0 <= obs_gy < self.MAP_SIZE:
                    self.map_data[obs_gy, obs_gx] = 1
                    self.confidence_map[obs_gy, obs_gx] = min(255, self.confidence_map[obs_gy, obs_gx] + 20)
                    self.stats['obstacles_found'] += 1
                    
    def update_map_from_data(self, map_data):
        """อัพเดทแผนที่จากข้อมูลที่ส่งมา"""
        try:
            # Handle different map data formats
            if isinstance(map_data, list):
                # Simple list format
                for entry in map_data:
                    if len(entry) >= 3:
                        x, y, value = entry[:3]
                        if 0 <= x < self.MAP_SIZE and 0 <= y < self.MAP_SIZE:
                            self.map_data[y, x] = value
            elif isinstance(map_data, dict):
                # Dictionary format with coordinates
                for coord, value in map_data.items():
                    x, y = map(int, coord.split(','))
                    if 0 <= x < self.MAP_SIZE and 0 <= y < self.MAP_SIZE:
                        self.map_data[y, x] = value
        except Exception as e:
            self.log(f"Map update error: {e}")
            
    def update_statistics(self):
        """อัพเดทสถิติต่างๆ"""
        # Calculate explored percentage
        known_cells = np.sum(self.map_data != -1)
        total_cells = self.MAP_SIZE * self.MAP_SIZE
        explored_percent = (known_cells / total_cells) * 100
        
        # Calculate path length
        path_length = 0
        if len(self.path_history) > 1:
            for i in range(1, len(self.path_history)):
                p1 = self.path_history[i-1]
                p2 = self.path_history[i]
                distance = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2) * self.CELL_SIZE
                path_length += distance
                
        # Calculate mapping time
        mapping_time = int(time.time() - self.stats['last_update'])
        minutes = mapping_time // 60
        seconds = mapping_time % 60
        
        # Update labels
        self.stats_labels['Explored'].config(text=f"{explored_percent:.1f}%")
        self.stats_labels['Obstacles'].config(text=str(self.stats['obstacles_found']))
        self.stats_labels['Path Length'].config(text=f"{path_length:.0f} cm")
        self.stats_labels['Mapping Time'].config(text=f"{minutes:02d}:{seconds:02d}")
        self.stats_labels['Packets'].config(text=str(self.stats['packets_received']))
        
    def send_command(self, command):
        """ส่งคำสั่งไปยังหุ่นยนต์"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(command.encode())
                self.log(f"TX: {command}")
            except Exception as e:
                self.log(f"Send error: {e}")
        else:
            self.log("Not connected")
            
    def on_key_press(self, event):
        """จัดการกับการกดปุ่มคีย์บอร์ด"""
        key = event.keysym.lower()
        
        command_map = {
            'w': 'w',  # Forward
            'up': 'w',
            's': 's',  # Backward  
            'down': 's',
            'a': 'a',  # Left
            'left': 'a',
            'd': 'd',  # Right
            'right': 'd',
            'x': 'x',  # Stop
            'space': 'x'
        }
        
        if key in command_map:
            self.send_command(command_map[key])
            
    def start_mapping(self):
        """เริ่มการทำแผนที่"""
        self.send_command('M')  # Mapping mode
        self.status_labels['Mode'].config(text="MAPPING", fg='#00FF00')
        self.stats['last_update'] = time.time()
        self.log("Mapping started")
        
    def pause_mapping(self):
        """หยุดชั่วคราว"""
        self.send_command('P')  # Pause
        self.status_labels['Mode'].config(text="PAUSED", fg='#FFA500')
        self.log("Mapping paused")
        
    def stop_mapping(self):
        """หยุดการทำแผนที่"""
        self.send_command('S')  # Stop
        self.status_labels['Mode'].config(text="STOPPED", fg='#FF0000')
        self.log("Mapping stopped")
        
    def clear_map(self):
        """ล้างแผนที่"""
        self.map_data.fill(-1)
        self.confidence_map.fill(0)
        self.path_history.clear()
        self.stats['obstacles_found'] = 0
        self.stats['packets_received'] = 0
        self.log("Map cleared")
        
    def save_map(self):
        """บันทึกแผนที่"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"robot_map_{timestamp}.json"
            
            map_export = {
                'timestamp': timestamp,
                'map_size': self.MAP_SIZE,
                'cell_size': self.CELL_SIZE,
                'robot_position': {
                    'x': self.robot_x,
                    'y': self.robot_y,
                    'heading': self.robot_heading
                },
                'map_data': self.map_data.tolist(),
                'confidence_map': self.confidence_map.tolist(),
                'path_history': list(self.path_history),
                'statistics': self.stats
            }
            
            with open(filename, 'w') as f:
                json.dump(map_export, f, indent=2)
                
            self.log(f"Map saved: {filename}")
            messagebox.showinfo("Save Complete", f"Map saved as {filename}")
            
        except Exception as e:
            self.log(f"Save error: {e}")
            messagebox.showerror("Save Error", str(e))
            
    def load_map(self, filename):
        """โหลดแผนที่"""
        try:
            with open(filename, 'r') as f:
                map_data = json.load(f)
                
            self.map_data = np.array(map_data['map_data'], dtype=np.int8)
            self.confidence_map = np.array(map_data['confidence_map'], dtype=np.uint8)
            self.path_history = deque(map_data['path_history'], maxlen=1000)
            
            if 'robot_position' in map_data:
                pos = map_data['robot_position']
                self.robot_x = pos['x']
                self.robot_y = pos['y']
                self.robot_heading = pos['heading']
                
            self.log(f"Map loaded: {filename}")
            
        except Exception as e:
            self.log(f"Load error: {e}")
            messagebox.showerror("Load Error", str(e))
            
    def log(self, message):
        """เพิ่มข้อความลงใน log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}\n"
        
        self.log_text.insert(tk.END, log_message)
        self.log_text.see(tk.END)
        
        # Keep log size manageable
        lines = int(self.log_text.index('end-1c').split('.')[0])
        if lines > 1000:
            self.log_text.delete('1.0', '500.0')
            
    def on_closing(self):
        """จัดการเมื่อปิดโปรแกรม"""
        if self.running:
            self.disconnect_serial()
        self.root.destroy()
        
    def run(self):
        """เริ่มการทำงานของโปรแกรม"""
        # Bind close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Auto scan ports on startup
        self.scan_ports()
        
        # Start main loop
        self.root.mainloop()

# ==============================================================================
# Test Data Generator (for testing without robot)
# ==============================================================================

class TestDataGenerator:
    """สำหรับทดสอบโปรแกรมโดยไม่ต้องมีหุ่นยนต์จริง"""
    
    def __init__(self, ui):
        self.ui = ui
        self.running = False
        self.thread = None
        
        # Simulated robot state
        self.sim_x = 500
        self.sim_y = 500
        self.sim_heading = 0
        self.sim_speed = 2
        
    def start_simulation(self):
        """เริ่มการจำลองข้อมูล"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._simulate_data, daemon=True)
            self.thread.start()
            self.ui.log("Test simulation started")
            
    def stop_simulation(self):
        """หยุดการจำลอง"""
        self.running = False
        self.ui.log("Test simulation stopped")
        
    def _simulate_data(self):
        """สร้างข้อมูลจำลอง"""
        while self.running:
            try:
                # Simulate robot movement
                self.sim_heading += (np.random.random() - 0.5) * 10  # Random turn
                self.sim_x += self.sim_speed * math.cos(math.radians(self.sim_heading))
                self.sim_y += self.sim_speed * math.sin(math.radians(self.sim_heading))
                
                # Keep robot in bounds
                self.sim_x = max(50, min(950, self.sim_x))
                self.sim_y = max(50, min(950, self.sim_y))
                
                # Simulate sensor readings
                sensors = []
                for angle in [0, 90, 180, 270]:
                    # Simulate distance based on position (simple obstacle simulation)
                    base_distance = 100 + np.random.random() * 200
                    if self.sim_x < 100 or self.sim_x > 900:  # Near walls
                        base_distance = min(base_distance, 50)
                    if self.sim_y < 100 or self.sim_y > 900:
                        base_distance = min(base_distance, 50)
                        
                    sensors.append(base_distance)
                
                # Send simulated status message
                status_msg = (f"[STATUS] Robot: ({self.sim_x:.1f},{self.sim_y:.1f}) "
                            f"{self.sim_heading:.1f}° | Sensors: "
                            f"F{sensors[0]:.1f} R{sensors[1]:.1f} "
                            f"B{sensors[2]:.1f} L{sensors[3]:.1f} | Mode: AUTO")
                
                self.ui.process_serial_data(status_msg)
                
                # Occasionally send telemetry
                if np.random.random() < 0.1:
                    voltage = 7.2 + np.random.random() * 1.2
                    telem_msg = f"[TELEMETRY] Bat:{voltage:.1f}V Temp:25.3C IMU:OK"
                    self.ui.process_serial_data(telem_msg)
                    
                time.sleep(0.1)  # 10Hz update rate
                
            except Exception as e:
                print(f"Simulation error: {e}")
                
# ==============================================================================
# Main Application
# ==============================================================================

def main():
    """ฟังก์ชันหลักของโปรแกรม"""
    try:
        # Create main application
        app = RealTimeMappingUI()
        
        # Optional: Add test data generator for development
        if '--test' in sys.argv or '--demo' in sys.argv:
            test_gen = TestDataGenerator(app)
            
            # Add test button to control panel
            test_btn = tk.Button(app.root, text="🧪 Start Test", 
                               command=test_gen.start_simulation,
                               bg='#795548', fg='white')
            test_btn.pack(side=tk.RIGHT, padx=5)
            
            stop_test_btn = tk.Button(app.root, text="⏹️ Stop Test", 
                                    command=test_gen.stop_simulation,
                                    bg='#795548', fg='white')
            stop_test_btn.pack(side=tk.RIGHT, padx=5)
            
            app.log("Test mode enabled - use test buttons to simulate robot")
        
        # Run application
        app.run()
        
    except Exception as e:
        print(f"Application error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    import sys
    main()