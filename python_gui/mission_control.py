#!/usr/bin/env python3
"""
Mission Control GUI - Emergency Robot Mapping System
สำหรับควบคุมและแสดงผลระบบ mapping แบบเรียลไทม์
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import queue
import time
import json
import math
from datetime import datetime
import numpy as np

# Import custom modules
from map_visualizer import MapVisualizer
from serial_handler import SerialHandler

class MissionControlGUI:
    def __init__(self):
        # ================================
        # Configuration
        # ================================
        self.MAP_SIZE = 30  # Grid size
        self.CELL_SIZE = 5.0  # cm per cell
        self.DISPLAY_SIZE = 600  # Display window size
        
        # Data storage
        self.robot_position = {'x': 0, 'y': 0, 'heading': 0}
        self.sensor_data = [0, 0, 0, 0]
        self.map_data = np.full((self.MAP_SIZE, self.MAP_SIZE), -1, dtype=np.int8)
        self.waypoints = []
        self.mission_active = False
        self.mapping_active = False
        
        # Communication
        self.serial_handler = None
        self.data_queue = queue.Queue()
        
        # Statistics
        self.stats = {
            'packets_received': 0,
            'last_update': time.time(),
            'mission_start_time': 0,
            'waypoints_completed': 0
        }
        
        # Create main window
        self.setup_gui()
        
        # Start update loop
        self.update_display()
        
    def setup_gui(self):
        """สร้าง GUI หลัก"""
        self.root = tk.Tk()
        self.root.title("🤖 Emergency Robot Mission Control")
        self.root.geometry("1400x900")
        self.root.configure(bg='#2C2C2C')
        
        # Create main frames
        self.create_control_panel()
        self.create_map_display()
        self.create_info_panel()
        self.create_status_bar()
        
        # Bind events
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def create_control_panel(self):
        """สร้าง Control Panel"""
        control_frame = tk.Frame(self.root, bg='#2C2C2C')
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        # Connection section
        conn_frame = tk.LabelFrame(control_frame, text="🔌 Connection", 
                                  bg='#2C2C2C', fg='white', font=('Arial', 10, 'bold'))
        conn_frame.pack(side=tk.LEFT, padx=5)
        
        tk.Label(conn_frame, text="Port:", bg='#2C2C2C', fg='white').grid(row=0, column=0, padx=5)
        
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, 
                                       width=15, state='readonly')
        self.port_combo.grid(row=0, column=1, padx=5)
        
        tk.Button(conn_frame, text="🔍 Scan", command=self.scan_ports,
                 bg='#4CAF50', fg='white', font=('Arial', 9)).grid(row=0, column=2, padx=5)
        
        self.connect_btn = tk.Button(conn_frame, text="📡 Connect", 
                                    command=self.toggle_connection,
                                    bg='#2196F3', fg='white', font=('Arial', 9))
        self.connect_btn.grid(row=0, column=3, padx=5)
        
        # Robot control section
        control_frame2 = tk.LabelFrame(control_frame, text="🤖 Robot Control", 
                                      bg='#2C2C2C', fg='white', font=('Arial', 10, 'bold'))
        control_frame2.pack(side=tk.LEFT, padx=20)
        
        btn_frame = tk.Frame(control_frame2, bg='#2C2C2C')
        btn_frame.pack()
        
        # Control buttons
        tk.Button(btn_frame, text="🗺️ Start Mapping", command=self.start_mapping,
                 bg='#4CAF50', fg='white', width=15).grid(row=0, column=0, padx=5, pady=2)
        
        tk.Button(btn_frame, text="⏹️ Stop Mapping", command=self.stop_mapping,
                 bg='#FF9800', fg='white', width=15).grid(row=0, column=1, padx=5, pady=2)
        
        tk.Button(btn_frame, text="🚀 Start Mission", command=self.start_mission,
                 bg='#2196F3', fg='white', width=15).grid(row=1, column=0, padx=5, pady=2)
        
        tk.Button(btn_frame, text="🛑 Emergency Stop", command=self.emergency_stop,
                 bg='#F44336', fg='white', width=15).grid(row=1, column=1, padx=5, pady=2)
        
        # File operations section
        file_frame = tk.LabelFrame(control_frame, text="💾 File Operations", 
                                  bg='#2C2C2C', fg='white', font=('Arial', 10, 'bold'))
        file_frame.pack(side=tk.LEFT, padx=20)
        
        tk.Button(file_frame, text="📁 Load Map", command=self.load_map,
                 bg='#607D8B', fg='white', width=12).pack(pady=2)
        
        tk.Button(file_frame, text="💾 Save Map", command=self.save_map,
                 bg='#607D8B', fg='white', width=12).pack(pady=2)
        
        tk.Button(file_frame, text="🗑️ Clear All", command=self.clear_all,
                 bg='#9C27B0', fg='white', width=12).pack(pady=2)
        
    def create_map_display(self):
        """สร้างหน้าจอแสดงแผนที่"""
        main_frame = tk.Frame(self.root, bg='#2C2C2C')
        main_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Map canvas
        self.canvas = tk.Canvas(main_frame, width=self.DISPLAY_SIZE, 
                               height=self.DISPLAY_SIZE, bg='#404040', 
                               highlightthickness=2, highlightbackground='white')
        self.canvas.pack()
        
        # Bind click events for waypoint placement
        self.canvas.bind("<Button-1>", self.on_map_click)
        self.canvas.bind("<Button-3>", self.on_map_right_click)
        
        # Instructions
        instruction_frame = tk.Frame(main_frame, bg='#2C2C2C')
        instruction_frame.pack(fill=tk.X, pady=5)
        
        tk.Label(instruction_frame, text="🖱️ Left click: Add waypoint | Right click: Remove waypoint",
                bg='#2C2C2C', fg='white', font=('Arial', 10)).pack()
        
    def create_info_panel(self):
        """สร้าง Information Panel"""
        info_frame = tk.Frame(self.root, bg='#2C2C2C', width=300)
        info_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        info_frame.pack_propagate(False)
        
        # Robot status
        status_frame = tk.LabelFrame(info_frame, text="🤖 Robot Status",
                                    bg='#2C2C2C', fg='white', font=('Arial', 11, 'bold'))
        status_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.status_labels = {}
        status_items = [
            ('Position X', '0.0 cm'),
            ('Position Y', '0.0 cm'),
            ('Heading', '0.0°'),
            ('State', 'DISCONNECTED'),
            ('Connection', '❌ Offline')
        ]
        
        for i, (label, value) in enumerate(status_items):
            tk.Label(status_frame, text=f"{label}:", bg='#2C2C2C', fg='white',
                    font=('Arial', 9)).grid(row=i, column=0, sticky='w', padx=5)
            self.status_labels[label] = tk.Label(status_frame, text=value,
                                                bg='#2C2C2C', fg='#00FF00',
                                                font=('Arial', 9, 'bold'))
            self.status_labels[label].grid(row=i, column=1, sticky='w', padx=5)
        
        # Sensor readings
        sensor_frame = tk.LabelFrame(info_frame, text="📡 Sensors (cm)",
                                    bg='#2C2C2C', fg='white', font=('Arial', 11, 'bold'))
        sensor_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.sensor_labels = {}
        sensors = ['Front', 'Right', 'Back', 'Left']
        colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4']
        
        for i, (sensor, color) in enumerate(zip(sensors, colors)):
            tk.Label(sensor_frame, text=f"{sensor}:", bg='#2C2C2C', fg='white',
                    font=('Arial', 9)).grid(row=i, column=0, sticky='w', padx=5)
            self.sensor_labels[sensor] = tk.Label(sensor_frame, text="--",
                                                 bg='#2C2C2C', fg=color,
                                                 font=('Arial', 9, 'bold'))
            self.sensor_labels[sensor].grid(row=i, column=1, sticky='w', padx=5)
        
        # Mission status
        mission_frame = tk.LabelFrame(info_frame, text="🎯 Mission Status",
                                     bg='#2C2C2C', fg='white', font=('Arial', 11, 'bold'))
        mission_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.mission_labels = {}
        mission_items = [
            ('Waypoints', '0'),
            ('Completed', '0'),
            ('Progress', '0%'),
            ('Status', 'IDLE')
        ]
        
        for i, (label, value) in enumerate(mission_items):
            tk.Label(mission_frame, text=f"{label}:", bg='#2C2C2C', fg='white',
                    font=('Arial', 9)).grid(row=i, column=0, sticky='w', padx=5)
            self.mission_labels[label] = tk.Label(mission_frame, text=value,
                                                 bg='#2C2C2C', fg='#FFA500',
                                                 font=('Arial', 9, 'bold'))
            self.mission_labels[label].grid(row=i, column=1, sticky='w', padx=5)
        
        # Statistics
        stats_frame = tk.LabelFrame(info_frame, text="📊 Statistics",
                                   bg='#2C2C2C', fg='white', font=('Arial', 11, 'bold'))
        stats_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.stats_labels = {}
        stats_items = [
            ('Map Completion', '0%'),
            ('Known Cells', '0'),
            ('Runtime', '00:00'),
            ('Packets RX', '0')
        ]
        
        for i, (label, value) in enumerate(stats_items):
            tk.Label(stats_frame, text=f"{label}:", bg='#2C2C2C', fg='white',
                    font=('Arial', 9)).grid(row=i, column=0, sticky='w', padx=5)
            self.stats_labels[label] = tk.Label(stats_frame, text=value,
                                               bg='#2C2C2C', fg='#87CEEB',
                                               font=('Arial', 9, 'bold'))
            self.stats_labels[label].grid(row=i, column=1, sticky='w', padx=5)
        
        # Activity log
        log_frame = tk.LabelFrame(info_frame, text="📝 Activity Log",
                                 bg='#2C2C2C', fg='white', font=('Arial', 11, 'bold'))
        log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.log_text = tk.Text(log_frame, height=12, width=35,
                               bg='#1E1E1E', fg='#00FF00',
                               font=('Consolas', 8))
        scrollbar = tk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Initial log entry
        self.log("🚀 Mission Control System initialized")
        
    def create_status_bar(self):
        """สร้าง Status Bar"""
        self.status_bar = tk.Label(self.root, text="Ready - Select COM port and connect",
                                  bg='#333333', fg='white', anchor=tk.W, relief=tk.SUNKEN)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
    # ==================== CONNECTION METHODS ====================
    def scan_ports(self):
        """สแกนหา Serial ports"""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.set(port_list[0])
            self.log(f"📡 Found {len(port_list)} ports")
            
            # Auto-detect ESP32 ports
            for port in ports:
                if any(keyword in port.description.lower() for keyword in ['ch340', 'cp210', 'esp32']):
                    self.port_combo.set(port.device)
                    self.log(f"🎯 ESP32 detected at {port.device}")
                    break
        else:
            self.log("❌ No ports found")
            
    def toggle_connection(self):
        """เชื่อมต่อ/ตัดการเชื่อมต่อ Serial"""
        if not self.serial_handler or not self.serial_handler.is_connected():
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
            self.serial_handler = SerialHandler(port, 115200, self.data_queue)
            self.serial_handler.start()
            
            self.connect_btn.config(text="🔌 Disconnect", bg='#F44336')
            self.status_bar.config(text=f"Connected to {port}")
            self.status_labels['Connection'].config(text="✅ Online", fg='#00FF00')
            self.log(f"✅ Connected to {port}")
            
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            self.log(f"❌ Connection failed: {e}")
            
    def disconnect_serial(self):
        """ตัดการเชื่อมต่อ Serial"""
        if self.serial_handler:
            self.serial_handler.stop()
            self.serial_handler = None
            
        self.connect_btn.config(text="📡 Connect", bg='#2196F3')
        self.status_bar.config(text="Disconnected")
        self.status_labels['Connection'].config(text="❌ Offline", fg='#FF0000')
        self.log("📴 Disconnected")
        
    # ==================== ROBOT CONTROL METHODS ====================
    def start_mapping(self):
        """เริ่มการทำแผนที่"""
        if self.send_command("m"):
            self.mapping_active = True
            self.log("🗺️ Mapping started")
            
    def stop_mapping(self):
        """หยุดการทำแผนที่"""
        if self.send_command("m"):
            self.mapping_active = False
            self.log("⏹️ Mapping stopped")
            
    def start_mission(self):
        """เริ่มภาระกิจ"""
        if len(self.waypoints) == 0:
            messagebox.showwarning("Warning", "No waypoints defined! Click on map to add waypoints.")
            return
            
        if self.send_command("g"):
            self.mission_active = True
            self.stats['mission_start_time'] = time.time()
            self.log(f"🚀 Mission started with {len(self.waypoints)} waypoints")
            
    def emergency_stop(self):
        """หยุดฉุกเฉิน"""
        if self.send_command("x"):
            self.mission_active = False
            self.mapping_active = False
            self.log("🛑 Emergency stop activated")
            
    def send_command(self, command):
        """ส่งคำสั่งไปยัง Control Station"""
        if not self.serial_handler or not self.serial_handler.is_connected():
            self.log("❌ Not connected")
            return False
            
        try:
            self.serial_handler.send(command)
            return True
        except Exception as e:
            self.log(f"❌ Send error: {e}")
            return False
            
    # ==================== MAP INTERACTION METHODS ====================
    def on_map_click(self, event):
        """จัดการการคลิกบนแผนที่"""
        # Convert canvas coordinates to world coordinates
        canvas_x = event.x
        canvas_y = event.y
        
        # Convert to grid coordinates
        grid_x = int(canvas_x / (self.DISPLAY_SIZE / self.MAP_SIZE))
        grid_y = int(canvas_y / (self.DISPLAY_SIZE / self.MAP_SIZE))
        
        # Convert to world coordinates (center of map is 0,0)
        world_x = (grid_x - self.MAP_SIZE // 2) * self.CELL_SIZE
        world_y = (grid_y - self.MAP_SIZE // 2) * self.CELL_SIZE
        
        # Add waypoint
        self.add_waypoint(world_x, world_y)
        
    def on_map_right_click(self, event):
        """จัดการการคลิกขวาบนแผนที่ (ลบ waypoint)"""
        canvas_x = event.x
        canvas_y = event.y
        
        # Find closest waypoint
        min_distance = float('inf')
        closest_waypoint = None
        
        for i, wp in enumerate(self.waypoints):
            # Convert waypoint to canvas coordinates
            grid_x = int(wp['x'] / self.CELL_SIZE + self.MAP_SIZE // 2)
            grid_y = int(wp['y'] / self.CELL_SIZE + self.MAP_SIZE // 2)
            wp_canvas_x = grid_x * (self.DISPLAY_SIZE / self.MAP_SIZE)
            wp_canvas_y = grid_y * (self.DISPLAY_SIZE / self.MAP_SIZE)
            
            distance = math.sqrt((canvas_x - wp_canvas_x)**2 + (canvas_y - wp_canvas_y)**2)
            if distance < min_distance and distance < 20:  # 20 pixel tolerance
                min_distance = distance
                closest_waypoint = i
                
        if closest_waypoint is not None:
            self.remove_waypoint(closest_waypoint)
            
    def add_waypoint(self, x, y):
        """เพิ่ม waypoint"""
        waypoint = {'x': x, 'y': y, 'completed': False}
        self.waypoints.append(waypoint)
        
        # Send waypoint to robot
        if self.serial_handler and self.serial_handler.is_connected():
            waypoint_cmd = f"1{x},{y}"
            self.serial_handler.send(waypoint_cmd)
            
        self.log(f"📍 Waypoint {len(self.waypoints)} added at ({x:.1f}, {y:.1f})")
        self.update_mission_display()
        
    def remove_waypoint(self, index):
        """ลบ waypoint"""
        if 0 <= index < len(self.waypoints):
            wp = self.waypoints.pop(index)
            self.log(f"🗑️ Waypoint removed: ({wp['x']:.1f}, {wp['y']:.1f})")
            self.update_mission_display()
            
    def clear_all(self):
        """ล้างข้อมูลทั้งหมด"""
        if messagebox.askyesno("Confirm", "Clear all waypoints and map data?"):
            self.waypoints.clear()
            self.map_data.fill(-1)
            self.mission_active = False
            self.mapping_active = False
            
            if self.send_command("c"):  # Clear command
                self.log("🗑️ All data cleared")
                
            self.update_mission_display()
            
    # ==================== FILE OPERATIONS ====================
    def save_map(self):
        """บันทึกแผนที่และ waypoints"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Save Mission Data"
        )
        
        if filename:
            try:
                data = {
                    'timestamp': datetime.now().isoformat(),
                    'map_size': self.MAP_SIZE,
                    'cell_size': self.CELL_SIZE,
                    'robot_position': self.robot_position,
                    'map_data': self.map_data.tolist(),
                    'waypoints': self.waypoints,
                    'stats': self.stats
                }
                
                with open(filename, 'w') as f:
                    json.dump(data, f, indent=2)
                    
                self.log(f"💾 Mission data saved: {filename}")
                messagebox.showinfo("Success", f"Mission data saved successfully!")
                
            except Exception as e:
                self.log(f"❌ Save error: {e}")
                messagebox.showerror("Error", f"Failed to save: {e}")
                
    def load_map(self):
        """โหลดแผนที่และ waypoints"""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Load Mission Data"
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                    
                # Validate data
                if data.get('map_size') != self.MAP_SIZE:
                    messagebox.showwarning("Warning", "Map size mismatch - data may not display correctly")
                    
                # Load data
                self.map_data = np.array(data['map_data'], dtype=np.int8)
                self.waypoints = data.get('waypoints', [])
                
                if 'robot_position' in data:
                    self.robot_position = data['robot_position']
                    
                self.log(f"📂 Mission data loaded: {filename}")
                self.log(f"📍 Loaded {len(self.waypoints)} waypoints")
                
                self.update_mission_display()
                messagebox.showinfo("Success", "Mission data loaded successfully!")
                
            except Exception as e:
                self.log(f"❌ Load error: {e}")
                messagebox.showerror("Error", f"Failed to load: {e}")
                
    # ==================== DISPLAY UPDATE METHODS ====================
    def update_display(self):
        """อัพเดทการแสดงผลแบบเรียลไทม์"""
        try:
            # Process incoming data
            while not self.data_queue.empty():
                try:
                    data = self.data_queue.get_nowait()
                    self.process_received_data(data)
                except queue.Empty:
                    break
                    
            # Update displays
            self.update_map_display()
            self.update_status_display()
            self.update_statistics()
            
        except Exception as e:
            self.log(f"❌ Display update error: {e}")
            
        # Schedule next update
        self.root.after(100, self.update_display)  # 10 FPS
        
    def process_received_data(self, data):
        """ประมวลผลข้อมูลที่รับมา"""
        try:
            line = data.strip()
            
            if '[STATUS]' in line:
                self.parse_status_message(line)
            elif 'Waypoint' in line and 'added' in line:
                self.log(f"✅ {line}")
            elif 'Mission' in line:
                self.log(f"🎯 {line}")
            elif line:
                self.log(f"📨 {line}")
                
            self.stats['packets_received'] += 1
            self.stats['last_update'] = time.time()
            
        except Exception as e:
            self.log(f"❌ Data processing error: {e}")
            
    def parse_status_message(self, line):
        """แยกข้อมูลจาก status message"""
        try:
            # Parse: [STATUS] Robot: (x,y) heading° | State: state | Map: completion%
            parts = line.split('|')
            
            if len(parts) >= 2:
                # Parse position
                robot_part = parts[0].split('Robot:')[1].strip()
                # Extract coordinates and heading
                import re
                match = re.search(r'\(([-\d.]+),([-\d.]+)\)\s+([-\d.]+)°', robot_part)
                if match:
                    self.robot_position['x'] = float(match.group(1))
                    self.robot_position['y'] = float(match.group(2))
                    self.robot_position['heading'] = float(match.group(3))
                    
                # Parse state
                if len(parts) > 1 and 'State:' in parts[1]:
                    state = parts[1].split('State:')[1].strip()
                    self.status_labels['State'].config(text=state)
                    
                # Parse map completion
                if len(parts) > 2 and 'Map:' in parts[2]:
                    completion = parts[2].split('Map:')[1].strip().replace('%', '')
                    try:
                        completion_val = float(completion)
                        self.stats_labels['Map Completion'].config(text=f"{completion_val:.1f}%")
                    except:
                        pass
                        
        except Exception as e:
            self.log(f"❌ Status parse error: {e}")
            
    def update_map_display(self):
        """อัพเดทการแสดงแผนที่"""
        self.canvas.delete("all")
        
        # Draw grid
        cell_size_pixels = self.DISPLAY_SIZE / self.MAP_SIZE
        
        # Draw map cells
        for x in range(self.MAP_SIZE):
            for y in range(self.MAP_SIZE):
                x1 = x * cell_size_pixels
                y1 = y * cell_size_pixels
                x2 = x1 + cell_size_pixels
                y2 = y1 + cell_size_pixels
                
                cell_value = self.map_data[y, x]
                
                if cell_value == -1:  # Unknown
                    color = '#404040'
                elif cell_value == 0:  # Free
                    color = '#90EE90'
                else:  # Obstacle
                    color = '#8B0000'
                    
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline='#606060', width=0.5)
                
        # Draw robot
        robot_grid_x = self.robot_position['x'] / self.CELL_SIZE + self.MAP_SIZE // 2
        robot_grid_y = self.robot_position['y'] / self.CELL_SIZE + self.MAP_SIZE // 2
        
        if 0 <= robot_grid_x < self.MAP_SIZE and 0 <= robot_grid_y < self.MAP_SIZE:
            rx = robot_grid_x * cell_size_pixels
            ry = robot_grid_y * cell_size_pixels
            robot_size = cell_size_pixels * 1.5
            
            # Robot circle
            self.canvas.create_oval(rx - robot_size/2, ry - robot_size/2,
                                   rx + robot_size/2, ry + robot_size/2,
                                   fill='red', outline='white', width=2)
                                   
            # Heading indicator
            heading_rad = math.radians(self.robot_position['heading'])
            hx = rx + robot_size * math.cos(heading_rad)
            hy = ry + robot_size * math.sin(heading_rad)
            
            self.canvas.create_line(rx, ry, hx, hy, fill='white', width=3,
                                   arrow=tk.LAST, arrowshape=(8, 10, 4))
                                   
        # Draw waypoints
        for i, wp in enumerate(self.waypoints):
            wp_grid_x = wp['x'] / self.CELL_SIZE + self.MAP_SIZE // 2
            wp_grid_y = wp['y'] / self.CELL_SIZE + self.MAP_SIZE // 2
            
            if 0 <= wp_grid_x < self.MAP_SIZE and 0 <= wp_grid_y < self.MAP_SIZE:
                wx = wp_grid_x * cell_size_pixels
                wy = wp_grid_y * cell_size_pixels
                
                # Waypoint marker
                color = '#00FF00' if wp.get('completed', False) else '#FFD700'
                self.canvas.create_oval(wx-8, wy-8, wx+8, wy+8,
                                       fill=color, outline='white', width=2)
                
                # Waypoint number
                self.canvas.create_text(wx, wy, text=str(i+1), fill='black',
                                       font=('Arial', 8, 'bold'))
                
        # Draw path between waypoints
        if len(self.waypoints) > 1:
            points = []
            for wp in self.waypoints:
                wp_grid_x = wp['x'] / self.CELL_SIZE + self.MAP_SIZE // 2
                wp_grid_y = wp['y'] / self.CELL_SIZE + self.MAP_SIZE // 2
                points.extend([wp_grid_x * cell_size_pixels, wp_grid_y * cell_size_pixels])
                
            if len(points) >= 4:
                self.canvas.create_line(points, fill='#0000FF', width=2, dash=(5, 5))
                
    def update_status_display(self):
        """อัพเดทการแสดงสถานะ"""
        # Update position labels
        self.status_labels['Position X'].config(text=f"{self.robot_position['x']:.1f} cm")
        self.status_labels['Position Y'].config(text=f"{self.robot_position['y']:.1f} cm")
        self.status_labels['Heading'].config(text=f"{self.robot_position['heading']:.1f}°")
        
        # Update sensor labels
        sensors = ['Front', 'Right', 'Back', 'Left']
        for i, sensor in enumerate(sensors):
            if i < len(self.sensor_data):
                value = self.sensor_data[i]
                if value > 0:
                    self.sensor_labels[sensor].config(text=f"{value:.1f}")
                else:
                    self.sensor_labels[sensor].config(text="--")
                    
    def update_mission_display(self):
        """อัพเดทการแสดงสถานะภาระกิจ"""
        # Count completed waypoints
        completed = sum(1 for wp in self.waypoints if wp.get('completed', False))
        total = len(self.waypoints)
        progress = (completed / total * 100) if total > 0 else 0
        
        self.mission_labels['Waypoints'].config(text=str(total))
        self.mission_labels['Completed'].config(text=str(completed))
        self.mission_labels['Progress'].config(text=f"{progress:.1f}%")
        
        if self.mission_active:
            self.mission_labels['Status'].config(text="ACTIVE", fg='#00FF00')
        elif total > 0:
            self.mission_labels['Status'].config(text="READY", fg='#FFA500')
        else:
            self.mission_labels['Status'].config(text="IDLE", fg='#87CEEB')
            
    def update_statistics(self):
        """อัพเดทสถิติ"""
        # Calculate known cells
        known_cells = np.sum(self.map_data != -1)
        self.stats_labels['Known Cells'].config(text=str(known_cells))
        
        # Calculate runtime
        if self.stats['mission_start_time'] > 0:
            runtime = int(time.time() - self.stats['mission_start_time'])
            minutes = runtime // 60
            seconds = runtime % 60
            self.stats_labels['Runtime'].config(text=f"{minutes:02d}:{seconds:02d}")
            
        # Update packet count
        self.stats_labels['Packets RX'].config(text=str(self.stats['packets_received']))
        
    # ==================== UTILITY METHODS ====================
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
        if self.serial_handler:
            self.serial_handler.stop()
        self.root.destroy()
        
    def run(self):
        """เริ่มการทำงานของโปรแกรม"""
        # Auto scan ports on startup
        self.scan_ports()
        
        # Start main loop
        self.root.mainloop()

# ==============================================================================
# TEST MODE (for development without robot)
# ==============================================================================

class TestDataGenerator:
    """สำหรับทดสอบ GUI โดยไม่ต้องมีหุ่นยนต์จริง"""
    
    def __init__(self, gui):
        self.gui = gui
        self.running = False
        self.thread = None
        
        # Simulated robot state
        self.sim_x = 0
        self.sim_y = 0
        self.sim_heading = 0
        self.sim_speed = 2
        
    def start_simulation(self):
        """เริ่มการจำลองข้อมูล"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._simulate_data, daemon=True)
            self.thread.start()
            self.gui.log("🧪 Test simulation started")
            
    def stop_simulation(self):
        """หยุดการจำลอง"""
        self.running = False
        self.gui.log("⏹️ Test simulation stopped")
        
    def _simulate_data(self):
        """สร้างข้อมูลจำลอง"""
        while self.running:
            try:
                # Simulate robot movement
                self.sim_heading += (np.random.random() - 0.5) * 10
                self.sim_x += self.sim_speed * math.cos(math.radians(self.sim_heading))
                self.sim_y += self.sim_speed * math.sin(math.radians(self.sim_heading))
                
                # Keep robot in bounds
                self.sim_x = max(-60, min(60, self.sim_x))
                self.sim_y = max(-60, min(60, self.sim_y))
                
                # Simulate sensor readings
                sensors = []
                for angle in [0, 90, 180, 270]:
                    base_distance = 30 + np.random.random() * 100
                    if abs(self.sim_x) > 50 or abs(self.sim_y) > 50:
                        base_distance = min(base_distance, 20)
                    sensors.append(base_distance)
                
                # Update GUI data
                self.gui.robot_position['x'] = self.sim_x
                self.gui.robot_position['y'] = self.sim_y
                self.gui.robot_position['heading'] = self.sim_heading
                self.gui.sensor_data = sensors
                
                # Simulate map updates
                grid_x = int(self.sim_x / self.gui.CELL_SIZE + self.gui.MAP_SIZE // 2)
                grid_y = int(self.sim_y / self.gui.CELL_SIZE + self.gui.MAP_SIZE // 2)
                
                if 0 <= grid_x < self.gui.MAP_SIZE and 0 <= grid_y < self.gui.MAP_SIZE:
                    self.gui.map_data[grid_y, grid_x] = 0  # Mark as free
                    
                    # Add some random obstacles
                    if np.random.random() < 0.1:
                        for dx in [-1, 0, 1]:
                            for dy in [-1, 0, 1]:
                                ox = grid_x + dx + int(np.random.random() * 10 - 5)
                                oy = grid_y + dy + int(np.random.random() * 10 - 5)
                                if 0 <= ox < self.gui.MAP_SIZE and 0 <= oy < self.gui.MAP_SIZE:
                                    self.gui.map_data[oy, ox] = 1
                
                # Send simulated status message
                status_msg = (f"[STATUS] Robot: ({self.sim_x:.1f},{self.sim_y:.1f}) "
                            f"{self.sim_heading:.1f}° | State: MAPPING | Map: 45.2%")
                
                self.gui.data_queue.put(status_msg)
                
                time.sleep(0.5)  # 2Hz update rate
                
            except Exception as e:
                print(f"Simulation error: {e}")

# ==============================================================================
# MAIN APPLICATION
# ==============================================================================

def main():
    """ฟังก์ชันหลักของโปรแกรม"""
    import sys
    
    try:
        # Create main application
        app = MissionControlGUI()
        
        # Check for test mode
        if '--test' in sys.argv or '--demo' in sys.argv:
            test_gen = TestDataGenerator(app)
            
            # Add test buttons
            test_frame = tk.Frame(app.root, bg='#2C2C2C')
            test_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)
            
            tk.Button(test_frame, text="🧪 Start Test Data", 
                     command=test_gen.start_simulation,
                     bg='#795548', fg='white').pack(side=tk.LEFT, padx=5)
            
            tk.Button(test_frame, text="⏹️ Stop Test Data", 
                     command=test_gen.stop_simulation,
                     bg='#795548', fg='white').pack(side=tk.LEFT, padx=5)
            
            app.log("🧪 Test mode enabled - use test buttons to simulate robot data")
        
        # Instructions
        app.log("📖 Instructions:")
        app.log("1. Select COM port and connect")
        app.log("2. Start mapping to explore area")
        app.log("3. Click on map to add waypoints")
        app.log("4. Start mission to execute waypoints")
        app.log("5. Use Save/Load to preserve missions")
        
        # Run application
        app.run()
        
    except Exception as e:
        print(f"Application error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()#!/usr/bin/env python3
"""
Mission Control GUI - Emergency Robot Mapping System
สำหรับควบคุมและแสดงผลระบบ mapping แบบเรียลไทม์
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import queue
import time
import json
import math
from datetime import datetime
import numpy as np

# Import custom modules
from map_visualizer import MapVisualizer
from serial_handler import SerialHandler

class MissionControlGUI:
    def __init__(self):
        # ================================
        # Configuration
        # ================================
        self.MAP_SIZE = 30  # Grid size
        self.CELL_SIZE = 5.0  # cm per cell
        self.DISPLAY_SIZE = 600  # Display window size
        
        # Data storage
        self.robot_position = {'x': 0, 'y': 0, 'heading': 0}
        self.sensor_data = [0, 0, 0, 0]
        self.map_data = np.full((self.MAP_SIZE, self.MAP_SIZE), -1, dtype=np.int8)
        self.waypoints = []
        self.mission_active = False
        self.mapping_active = False
        
        # Communication
        self.serial_handler = None
        self.data_queue = queue.Queue()
        
        # Statistics
        self.stats = {
            'packets_received': 0,
            'last_update': time.time(),
            'mission_start_time': 0,
            'waypoints_completed': 0
        }
        
        # Create main window
        self.setup_gui()
        
        # Start update loop
        self.update_display()
        
    def setup_gui(self):
        """สร้าง GUI หลัก"""
        self.root = tk.Tk()
        self.root.title("🤖 Emergency Robot Mission Control")
        self.root.geometry("1400x900")
        self.root.configure(bg='#2C2C2C')