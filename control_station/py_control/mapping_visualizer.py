import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import threading
import queue
import time
from collections import deque

class RobotMapVisualizer:
    def __init__(self, serial_port='COM6', baud_rate=115200):
        # Map configuration
        self.MAP_SIZE = 100
        self.CELL_SIZE = 10  # cm
        self.map_data = np.full((self.MAP_SIZE, self.MAP_SIZE), -1, dtype=np.int8)  # -1=unknown, 0=free, 100=occupied
        
        # Robot state
        self.robot_pos = {'x': self.MAP_SIZE * self.CELL_SIZE / 2, 
                         'y': self.MAP_SIZE * self.CELL_SIZE / 2, 
                         'heading': 0}
        self.sensor_distances = [0, 0, 0, 0]  # Front, Right, Back, Left
        
        # Data queue for thread-safe communication
        self.data_queue = queue.Queue()
        self.path_history = deque(maxlen=1000)
        
        # Serial connection
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            print(f"Connected to {serial_port}")
        except Exception as e:
            print(f"Failed to connect to serial port: {e}")
            self.ser = None
        
        # Matplotlib setup
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(16, 8))
        self.setup_plots()
        
        # Start serial reading thread
        if self.ser:
            self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.serial_thread.start()
    
    def setup_plots(self):
        """Setup matplotlib plots"""
        # Main map plot
        self.ax1.set_title('Robot SLAM Map', fontsize=14)
        self.ax1.set_xlabel('X (cm)')
        self.ax1.set_ylabel('Y (cm)')
        self.ax1.set_aspect('equal')
        self.ax1.grid(True, alpha=0.3)
        
        # Sensor data plot
        self.ax2.set_title('Ultrasonic Sensor Readings', fontsize=14)
        self.ax2.set_ylabel('Distance (cm)')
        self.ax2.set_xlabel('Time')
        self.ax2.grid(True, alpha=0.3)
        
        # Initialize sensor data tracking
        self.sensor_history = {'time': deque(maxlen=100),
                              'front': deque(maxlen=100),
                              'right': deque(maxlen=100), 
                              'back': deque(maxlen=100),
                              'left': deque(maxlen=100)}
    
    def read_serial_data(self):
        """Read data from ESP32 control station"""
        while True:
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"📨 Received: {line}")  # Debug output
                        self.parse_serial_data(line)
                except Exception as e:
                    print(f"Serial read error: {e}")
            time.sleep(0.01)
    
    def parse_serial_data(self, line):
        """Parse incoming serial data from control station"""
        if '[STATUS]' in line:
            # Parse status line: [STATUS] Robot: (x,y) heading° | Sensors: Fx Rx Bx Lx
            try:
                parts = line.split('|')
                if len(parts) >= 2:
                    # Parse robot position and heading
                    robot_part = parts[0].split('Robot:')[1].strip()
                    pos_heading = robot_part.split('°')[0]
                    pos_part, heading_part = pos_heading.rsplit(' ', 1)
                    
                    # Extract coordinates
                    coords = pos_part.strip('()').split(',')
                    x, y = float(coords[0]), float(coords[1])
                    heading = float(heading_part)
                    
                    # Parse sensor data
                    sensor_part = parts[1].split('Sensors:')[1].strip()
                    sensors = []
                    for s in sensor_part.split():
                        if s.startswith(('F', 'R', 'B', 'L')):
                            sensors.append(float(s[1:]))
                    
                    # Update data
                    data = {
                        'type': 'status',
                        'robot_pos': {'x': x, 'y': y, 'heading': heading},
                        'sensors': sensors,
                        'timestamp': time.time()
                    }
                    self.data_queue.put(data)
            except Exception as e:
                print(f"Error parsing status: {e}")
        
        elif 'Received:' in line:
            # Parse received data line
            try:
                if 'Robot at' in line:
                    # Extract position data
                    parts = line.split('Robot at')[1].strip()
                    pos_heading = parts.split('heading')[0].strip('() ')
                    x, y = map(float, pos_heading.split(','))
                    heading = float(parts.split('heading')[1].split('°')[0].strip())
                    
                    data = {
                        'type': 'position',
                        'robot_pos': {'x': x, 'y': y, 'heading': heading},
                        'timestamp': time.time()
                    }
                    self.data_queue.put(data)
            except Exception as e:
                print(f"Error parsing received data: {e}")
        
        elif 'Last Send Status' in line:
            if 'Success' in line:
                print("✅ Command sent successfully!")
            else:
                print("❌ Command failed!")
    
    def send_command(self, command):
        """Send movement command to robot"""
        if self.ser:
            self.ser.write(command.encode())
            print(f"Sent command: {command}")
    
    def update_robot_position(self, pos_data):
        """Update robot position and path history"""
        self.robot_pos = pos_data
        
        # Convert to grid coordinates for path history
        grid_x = int(pos_data['x'] / self.CELL_SIZE)
        grid_y = int(pos_data['y'] / self.CELL_SIZE)
        self.path_history.append((grid_x, grid_y))
    
    def update_sensor_data(self, sensor_data):
        """Update sensor readings and history"""
        self.sensor_distances = sensor_data
        current_time = time.time()
        
        self.sensor_history['time'].append(current_time)
        self.sensor_history['front'].append(sensor_data[0] if len(sensor_data) > 0 else 0)
        self.sensor_history['right'].append(sensor_data[1] if len(sensor_data) > 1 else 0)
        self.sensor_history['back'].append(sensor_data[2] if len(sensor_data) > 2 else 0)
        self.sensor_history['left'].append(sensor_data[3] if len(sensor_data) > 3 else 0)
    
    def simulate_map_update(self):
        """Simulate map updates based on robot position and sensors"""
        # Convert robot position to grid coordinates
        grid_x = int(self.robot_pos['x'] / self.CELL_SIZE)
        grid_y = int(self.robot_pos['y'] / self.CELL_SIZE)
        
        if 0 <= grid_x < self.MAP_SIZE and 0 <= grid_y < self.MAP_SIZE:
            # Mark robot position as free
            self.map_data[grid_y, grid_x] = 0
            
            # Simulate obstacle detection based on sensor readings
            sensor_angles = [0, 90, 180, 270]  # Front, Right, Back, Left
            
            for i, (distance, angle) in enumerate(zip(self.sensor_distances, sensor_angles)):
                if distance > 0 and distance < 400:  # Valid reading
                    # Calculate obstacle position
                    total_angle = np.radians(self.robot_pos['heading'] + angle)
                    obs_x = self.robot_pos['x'] + distance * np.cos(total_angle)
                    obs_y = self.robot_pos['y'] + distance * np.sin(total_angle)
                    
                    obs_grid_x = int(obs_x / self.CELL_SIZE)
                    obs_grid_y = int(obs_y / self.CELL_SIZE)
                    
                    # Mark obstacle if within map bounds
                    if 0 <= obs_grid_x < self.MAP_SIZE and 0 <= obs_grid_y < self.MAP_SIZE:
                        self.map_data[obs_grid_y, obs_grid_x] = 100
                    
                    # Mark free space along the ray
                    steps = int(distance / (self.CELL_SIZE / 2))
                    for step in range(1, steps):
                        ray_x = self.robot_pos['x'] + step * (self.CELL_SIZE / 2) * np.cos(total_angle)
                        ray_y = self.robot_pos['y'] + step * (self.CELL_SIZE / 2) * np.sin(total_angle)
                        
                        ray_grid_x = int(ray_x / self.CELL_SIZE)
                        ray_grid_y = int(ray_y / self.CELL_SIZE)
                        
                        if (0 <= ray_grid_x < self.MAP_SIZE and 0 <= ray_grid_y < self.MAP_SIZE and 
                            self.map_data[ray_grid_y, ray_grid_x] != 100):
                            self.map_data[ray_grid_y, ray_grid_x] = 0
    
    def animate(self, frame):
        """Animation function for matplotlib"""
        # Process queued data
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                
                if data['type'] == 'status':
                    self.update_robot_position(data['robot_pos'])
                    if 'sensors' in data:
                        self.update_sensor_data(data['sensors'])
                elif data['type'] == 'position':
                    self.update_robot_position(data['robot_pos'])
                
            except queue.Empty:
                break
        
        # Update simulated map
        self.simulate_map_update()
        
        # Clear plots
        self.ax1.clear()
        self.ax2.clear()
        
        # Plot map
        self.ax1.set_title(f'Robot SLAM Map - Position: ({self.robot_pos["x"]:.1f}, {self.robot_pos["y"]:.1f})')
        self.ax1.set_xlabel('X (cm)')
        self.ax1.set_ylabel('Y (cm)')
        
        # Show map around robot (dynamic zoom)
        robot_grid_x = int(self.robot_pos['x'] / self.CELL_SIZE)
        robot_grid_y = int(self.robot_pos['y'] / self.CELL_SIZE)
        
        view_size = 40  # Show 40x40 cells around robot
        x_min = max(0, robot_grid_x - view_size)
        x_max = min(self.MAP_SIZE, robot_grid_x + view_size)
        y_min = max(0, robot_grid_y - view_size)
        y_max = min(self.MAP_SIZE, robot_grid_y + view_size)
        
        # Create map display
        map_display = self.map_data[y_min:y_max, x_min:x_max]
        
        # Convert map values for display
        display_map = np.zeros_like(map_display, dtype=float)
        display_map[map_display == -1] = 0.5  # Unknown = gray
        display_map[map_display == 0] = 1.0   # Free = white
        display_map[map_display == 100] = 0.0 # Occupied = black
        
        extent = [x_min * self.CELL_SIZE, x_max * self.CELL_SIZE, 
                 y_min * self.CELL_SIZE, y_max * self.CELL_SIZE]
        
        self.ax1.imshow(display_map, cmap='gray', extent=extent, origin='lower', alpha=0.8)
        
        # Draw robot
        robot_circle = Circle((self.robot_pos['x'], self.robot_pos['y']), 
                             radius=15, color='red', alpha=0.7)
        self.ax1.add_patch(robot_circle)
        
        # Draw robot heading
        heading_rad = np.radians(self.robot_pos['heading'])
        arrow_length = 25
        dx = arrow_length * np.cos(heading_rad)
        dy = arrow_length * np.sin(heading_rad)
        
        self.ax1.arrow(self.robot_pos['x'], self.robot_pos['y'], dx, dy,
                      head_width=8, head_length=8, fc='red', ec='red')
        
        # Draw path history
        if len(self.path_history) > 1:
            path_x = [pos[0] * self.CELL_SIZE for pos in self.path_history]
            path_y = [pos[1] * self.CELL_SIZE for pos in self.path_history]
            self.ax1.plot(path_x, path_y, 'b-', alpha=0.5, linewidth=2)
        
        # Draw sensor ranges
        sensor_angles = [0, 90, 180, 270]
        sensor_colors = ['red', 'green', 'blue', 'orange']
        sensor_names = ['Front', 'Right', 'Back', 'Left']
        
        legend_handles = []
        for i, (distance, angle, color, name) in enumerate(zip(self.sensor_distances, sensor_angles, sensor_colors, sensor_names)):
            if distance > 0:
                total_angle = np.radians(self.robot_pos['heading'] + angle)
                end_x = self.robot_pos['x'] + distance * np.cos(total_angle)
                end_y = self.robot_pos['y'] + distance * np.sin(total_angle)
                
                line = self.ax1.plot([self.robot_pos['x'], end_x], [self.robot_pos['y'], end_y], 
                             color=color, alpha=0.6, linewidth=2, label=f'{name}: {distance:.1f}cm')[0]
                legend_handles.append(line)
        
        if legend_handles:
            self.ax1.legend(handles=legend_handles, loc='upper right')
        
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_aspect('equal')
        
        # Plot sensor data over time
        self.ax2.set_title('Ultrasonic Sensor Readings Over Time')
        self.ax2.set_ylabel('Distance (cm)')
        self.ax2.set_xlabel('Time (relative)')
        
        if len(self.sensor_history['time']) > 0:
            base_time = self.sensor_history['time'][0] if self.sensor_history['time'] else 0
            rel_time = [t - base_time for t in self.sensor_history['time']]
            
            self.ax2.plot(rel_time, self.sensor_history['front'], 'r-', label='Front', linewidth=2)
            self.ax2.plot(rel_time, self.sensor_history['right'], 'g-', label='Right', linewidth=2)
            self.ax2.plot(rel_time, self.sensor_history['back'], 'b-', label='Back', linewidth=2)
            self.ax2.plot(rel_time, self.sensor_history['left'], 'orange', label='Left', linewidth=2)
            
            self.ax2.legend()
            self.ax2.set_ylim(0, 400)
        
        self.ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
    
    def on_key_press(self, event):
        """Handle keyboard input for robot control"""
        if event.key == 'w':
            self.send_command('w')
        elif event.key == 's':
            self.send_command('s')
        elif event.key == 'a':
            self.send_command('a')
        elif event.key == 'd':
            self.send_command('d')
        elif event.key == 'x':
            self.send_command('x')
        elif event.key == 'm':
            self.send_command('m')
        elif event.key == 't':
            self.send_command('t')
        elif event.key == 'r':
            # Reset map
            self.map_data.fill(-1)
            self.path_history.clear()
            print("Map reset")
    
    def run(self):
        """Start the visualization"""
        print("=== Robot SLAM Visualizer ===")
        print("Controls:")
        print("  W - Forward")
        print("  S - Backward") 
        print("  A - Turn Left")
        print("  D - Turn Right")
        print("  X - Stop")
        print("  M - Print Map")
        print("  T - Map Statistics")
        print("  R - Reset Map")
        print("=============================")
        
        # Connect keyboard events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Start animation with cache_frame_data=False to avoid warning
        ani = animation.FuncAnimation(self.fig, self.animate, interval=100, 
                                    blit=False, cache_frame_data=False)
        plt.show()
    
    def close(self):
        """Clean up resources"""
        if self.ser:
            self.ser.close()

if __name__ == "__main__":
    import sys
    
    # Default port สำหรับ Windows
    default_port = 'COM6'
    
    # รับ argument จาก command line
    if len(sys.argv) > 1:
        port = sys.argv[1]
        print(f"Using specified port: {port}")
    else:
        port = default_port
        print(f"Using default port: {port}")
        print("💡 Tip: You can specify port like: python mapping_visualizer.py COM3")
    
    # Create and run visualizer
    visualizer = RobotMapVisualizer(serial_port=port)
    
    try:
        visualizer.run()
    except KeyboardInterrupt:
        print("\nShutting down visualizer...")
    finally:
        visualizer.close()