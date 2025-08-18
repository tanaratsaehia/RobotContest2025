<<<<<<< Updated upstream
#!/usr/bin/env python3
<<<<<<< Updated upstream
# Enhanced controller.py - ปรับปรุงให้ใช้งานง่ายขึ้น
=======
"""
OPTIMIZED Robot Controller - Fast Data Updates
รองรับข้อมูลเซ็นเซอร์ที่เร็วขึ้น (ทุก 100ms)
"""
>>>>>>> Stashed changes

import serial
import sys
import threading
import time
<<<<<<< Updated upstream
import os
=======
import tkinter as tk
from tkinter import ttk
from collections import deque
import numpy as np
>>>>>>> Stashed changes

# Configuration
SERIAL_PORT = 'COM6'  # แก้ไขตาม port ของคุณ
BAUD_RATE = 115200

class RobotController:
    def __init__(self, port, baud_rate):
        try:
            self.ser = serial.Serial(port, baud_rate, timeout=0.1, dsrdtr=False)
            self.running = True
            print(f"[Python] Connected to {port} at {baud_rate} baud")
        except Exception as e:
            print(f"[Python] Error connecting to {port}: {e}")
            sys.exit(1)
    
    def read_serial(self):
        """อ่านข้อมูลจาก ESP32 อย่างต่อเนื่อง"""
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # แสดงข้อมูลจาก robot แบบ color coding
                        if "Robot Status" in line or "Position:" in line:
                            print(f"\033[92m{line}\033[0m")  # Green for status
                        elif "ERROR" in line or "EMERGENCY" in line:
                            print(f"\033[91m{line}\033[0m")  # Red for errors
                        elif "Command Received" in line:
                            print(f"\033[94m{line}\033[0m")  # Blue for commands
                        else:
                            print(line)
            except Exception as e:
                print(f"[Python] Serial read error: {e}")
                break
            
            time.sleep(0.01)  # เพื่อไม่ให้ใช้ CPU เยอะ
    
    def send_command(self, cmd):
        """ส่งคำสั่งไปยัง ESP32"""
        try:
            self.ser.write(cmd.encode())
            print(f"[Python] Sent: '{cmd}'")
        except Exception as e:
            print(f"[Python] Send error: {e}")
    
    def close(self):
        """ปิดการเชื่อมต่อ"""
        self.running = False
        self.ser.close()

def clear_screen():
    """ล้างหน้าจอ"""
    os.system('clear' if os.name == 'posix' else 'cls')

def print_header():
    """แสดงหัวข้อโปรแกรม"""
    clear_screen()
    print("=" * 60)
    print("🤖 ROBOT AUTONOMOUS MAPPING CONTROLLER 🗺️")
    print("=" * 60)

def print_commands():
    """แสดงรายการคำสั่งที่ใช้ได้"""
    print("\n📋 AVAILABLE COMMANDS:")
    print("-" * 40)
    print("🎮 Manual Control:")
    print("  w/s/a/d  - Move robot (forward/back/left/right)")
    print("  x        - Stop robot")
    print("")
    print("🗺️  Autonomous Mapping:")
    print("  m        - 🚀 START MAPPING")
    print("  r        - 🏠 Return to start")
    print("  p        - ⏸️  Pause mapping")
    print("  c        - ▶️  Continue mapping")
    print("")
    print("⚙️  Settings & Info:")
    print("  1-9      - Set speed (10%-90%)")
    print("  t        - 🔍 Test sensors")
    print("  M        - 📊 Show mapping status")
    print("  R        - 🔄 Reset position")
    print("  h        - 📖 Show this help")
    print("  q        - 🚪 Quit program")
    print("-" * 40)

def main():
    # เชื่อมต่อกับ robot
    controller = RobotController(SERIAL_PORT, BAUD_RATE)
    
    # เริ่ม thread สำหรับอ่าน serial
    serial_thread = threading.Thread(target=controller.read_serial, daemon=True)
    serial_thread.start()
    
    print_header()
    print_commands()
    print("\n✅ Robot connected! Ready for commands...")
    print("💡 Tip: Press 'm' to start autonomous mapping!")
    print("\nEnter command: ", end="", flush=True)
    
    try:
        while True:
            # รอคำสั่งจากผู้ใช้
            try:
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    cmd = input().strip().lower()
                else:
                    time.sleep(0.1)
                    continue
            except:
                # สำหรับระบบที่ไม่ support select
                cmd = input().strip().lower()
            
            if not cmd:
                print("Enter command: ", end="", flush=True)
                continue
            
            # ประมวลผลคำสั่ง
            if cmd == 'q':
                print("👋 Goodbye!")
                break
            elif cmd == 'h':
                print_header()
                print_commands()
            elif cmd == 'm':
                print("\n🚀 Starting autonomous mapping...")
                print("🤖 Robot will explore and return to start automatically")
                controller.send_command('m')
            elif cmd == 'r':
                print("\n🏠 Commanding robot to return to start...")
                controller.send_command('r')
            elif cmd == 'p':
                print("\n⏸️  Pausing mapping...")
                controller.send_command('p')
            elif cmd == 'c':
                print("\n▶️  Continuing mapping...")
                controller.send_command('c')
            elif cmd == 't':
                print("\n🔍 Testing all sensors...")
                controller.send_command('t')
            elif cmd == 'x':
                print("\n🛑 Emergency stop!")
                controller.send_command('x')
            elif cmd in 'wasd':
                direction_names = {'w': 'forward', 'a': 'left', 's': 'backward', 'd': 'right'}
                print(f"🎮 Moving {direction_names[cmd]}...")
                controller.send_command(cmd)
            elif cmd in '123456789':
                speed = int(cmd) * 10
                print(f"⚙️  Setting motor speed to {speed}%...")
                controller.send_command(cmd)
            elif cmd == 'M':
                print("\n📊 Requesting mapping status...")
                controller.send_command('M')
            elif cmd == 'R':
                print("\n🔄 Resetting robot position...")
                controller.send_command('R')
            else:
                print(f"❌ Unknown command: '{cmd}' (press 'h' for help)")
            
            print("\nEnter command: ", end="", flush=True)
            
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    
    finally:
        print("🔌 Closing connection...")
        controller.close()

if __name__ == "__main__":
    # Import select เฉพาะระบบที่ support
    try:
        import select
    except ImportError:
        select = None
    
=======
import serial
import threading
import time
import sys
import os
import re

# ตั้งค่า Serial Port - เปลี่ยนให้ตรงกับพอร์ตของคุณ
SERIAL_PORT = 'COM6'  # Windows: COM3, COM4, etc. | Linux/Mac: /dev/ttyUSB0, /dev/ttyACM0
BAUD_RATE = 115200
DATA_HISTORY_SIZE = 100  # Keep last 100 data points for analysis

class OptimizedRobotController:
    def __init__(self):
        self.ser = None
<<<<<<< Updated upstream
=======
        self.connected = False
        self.robot_pos = [25.0, 25.0]  # Start at center of 50x50 map
        self.robot_heading = 0.0
        self.sensor_data = [0.0, 0.0, 0.0, 0.0]
        
        # Performance tracking
        self.data_count = 0
        self.data_rate = 0.0
        self.last_data_time = 0
        self.data_history = deque(maxlen=DATA_HISTORY_SIZE)
        
        # Enhanced patterns for fast parsing
        self.robot_pattern = re.compile(
            r"\[ROBOT\]\s*POS:([\d.-]+),([\d.-]+),([\d.-]+)\|S:([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+)"
        )
        self.pos_pattern = re.compile(
            r"POS:([\d.-]+),([\d.-]+),([\d.-]+)\|S:([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+)"
        )
        self.sensor_pattern = re.compile(r"Sensor\s*(\d+)\s*.*?:\s*([\d.-]+)\s*cm", re.I)
        
        self.setup_gui()
>>>>>>> Stashed changes
        self.running = True
        self.last_command_time = 0
        self.command_cooldown = 0.3  # 300ms between commands
        
<<<<<<< Updated upstream
        # Data storage
        self.sensor_data = {'Front': '---', 'Right': '---', 'Back': '---', 'Left': '---'}
        self.robot_position = {'x': 25.0, 'y': 25.0, 'angle': 0.0}
        self.obstacles = set()
        self.waypoints = []
        self.mapping_active = False
        self.navigation_active = False
        
        # Display control
        self.show_sensors = False
        self.show_map = False
        self.last_display_update = 0
        
        self.setup_serial()
        
    def setup_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, dsrdtr=False)
            print(f"✅ Connected to {SERIAL_PORT}")
            print("📡 Starting data reader...")
            threading.Thread(target=self.read_serial, daemon=True).start()
            threading.Thread(target=self.update_display, daemon=True).start()
            time.sleep(1)  # Give connection time to stabilize
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            print(f"💡 Make sure {SERIAL_PORT} is correct and ESP32 is connected")
            sys.exit(1)
    
    def read_serial(self):
        """อ่านข้อมูลจาก Serial และแสดงผล"""
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    raw = self.ser.readline()
                    if raw:
                        line = raw.decode('utf-8', errors='ignore').strip()
                        if line:
                            self.parse_data(line)
                            # แสดงข้อความสำคัญ
                            if any(keyword in line.lower() for keyword in 
                                  ['mapping', 'navigation', 'waypoint', 'command', 'ready', 'error', 'starting', 'stopping']):
                                print(f"🤖 {line}")
                except Exception as e:
                    if self.running:
                        print(f"📡 Serial read error: {e}")
            time.sleep(0.01)
    
    def parse_data(self, line):
        """แยกข้อมูลจากหุ่นยนต์"""
        # Parse sensor data: "Sensors - Front: 45.2cm | Right: 12.8cm | Back: 89.1cm | Left: 67.5cm"
        if line.startswith("Sensors -"):
            parts = line.split(" | ")
            for part in parts:
                if ":" in part:
                    sensor_info = part.split(": ")
                    if len(sensor_info) == 2:
                        sensor_name = sensor_info[0].replace("Sensors - ", "").strip()
                        sensor_value = sensor_info[1].replace("cm", "").strip()
                        if sensor_name in self.sensor_data:
                            self.sensor_data[sensor_name] = sensor_value
        
        # Parse position data: "POS:25.0,25.0,45.0"
        pos_match = re.search(r"(?:POS|NAV_POS):([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?)", line)
        if pos_match:
            self.robot_position['x'] = float(pos_match.group(1))
            self.robot_position['y'] = float(pos_match.group(2))
            self.robot_position['angle'] = float(pos_match.group(3))
        
        # Parse obstacle data: "OBS:15,20"
        obs_match = re.search(r"OBS:(\d+),(\d+)", line)
        if obs_match:
            x, y = int(obs_match.group(1)), int(obs_match.group(2))
            self.obstacles.add((x, y))
        
        # Parse system status
        if "STARTING MAPPING MODE" in line:
            self.mapping_active = True
        elif "STOPPING MAPPING MODE" in line:
            self.mapping_active = False
        elif "Starting autonomous navigation" in line:
            self.navigation_active = True
        elif "Autonomous navigation stopped" in line:
            self.navigation_active = False
    
    def clear_screen(self):
        """ล้างหน้าจอ"""
        os.system('cls' if os.name == 'nt' else 'clear')
    
    def display_sensor_status(self):
        """แสดงสถานะเซ็นเซอร์แบบ real-time"""
        print("╔═══════════════ SENSOR STATUS ═══════════════╗")
        print("║                                             ║")
        
        # แสดงเซ็นเซอร์ในรูปแบบหุ่นยนต์
        front = self.sensor_data['Front']
        back = self.sensor_data['Back']
        left = self.sensor_data['Left']
        right = self.sensor_data['Right']
        
        print(f"║              🔴 Front: {front:>6}cm              ║")
        print("║                      ┌───┐                    ║")
        print(f"║  🔴 Left: {left:>6}cm  │ 🤖 │  Right: {right:>6}cm 🔴  ║")
        print("║                      └───┘                    ║")
        print(f"║               🔴 Back: {back:>6}cm              ║")
        print("║                                             ║")
        
        # แสดงสถานะสี
        def get_status_color(value):
            if value == '---':
                return "⚫ No Data"
            try:
                val = float(value)
                if val < 15:
                    return "🔴 DANGER"
                elif val < 30:
                    return "🟡 CAUTION"
                else:
                    return "🟢 SAFE"
            except:
                return "⚫ Error"
        
        print(f"║ Status: {get_status_color(front):>12} | {get_status_color(right):>12} ║")
        print(f"║         {get_status_color(left):>12} | {get_status_color(back):>12} ║")
        print("╚═════════════════════════════════════════════╝")
    
    def display_map_status(self):
        """แสดงแผนที่และสถานะหุ่นยนต์"""
        print("╔═══════════════ MAP & STATUS ════════════════╗")
        
        # แสดงตำแหน่งหุ่นยนต์
        print(f"║ Robot Position: ({self.robot_position['x']:6.1f}, {self.robot_position['y']:6.1f})    ║")
        print(f"║ Robot Angle:    {self.robot_position['angle']:6.1f}°                    ║")
        print(f"║ Mapping:        {'🟢 ACTIVE' if self.mapping_active else '🔴 INACTIVE':>12}           ║")
        print(f"║ Navigation:     {'🟢 ACTIVE' if self.navigation_active else '🔴 INACTIVE':>12}           ║")
        print(f"║ Obstacles:      {len(self.obstacles):>3} detected              ║")
        print("║                                             ║")
        
        # แสดงแผนที่ขนาดเล็ก (15x10)
        print("║           🗺️  Mini Map (Grid)               ║")
        
        # คำนวณขอบเขตที่จะแสดง
        robot_x = int(self.robot_position['x'] / 10)  # แปลงเป็น grid
        robot_y = int(self.robot_position['y'] / 10)
        
        for y in range(9, -1, -1):  # 10 rows, แสดงจากบนลงล่าง
            line = "║ "
            for x in range(15):  # 15 columns
                if x == robot_x and y == robot_y:
                    line += "🤖"
                elif (x, y) in self.obstacles:
                    line += "🟥"
                else:
                    line += "⬜"
            line += " ║"
            print(line)
        
        print("║ 🤖=Robot 🟥=Obstacle ⬜=Free Space          ║")
        print("╚═════════════════════════════════════════════╝")
    
    def display_combined_status(self):
        """แสดงสถานะรวม"""
        current_time = time.time()
        if current_time - self.last_display_update < 0.5:  # อัพเดตทุก 500ms
            return
        
        self.last_display_update = current_time
        
        if self.show_sensors or self.show_map:
            self.clear_screen()
            print("🤖 ROBOT CONTROL SYSTEM - LIVE STATUS")
            print("=" * 50)
            
            if self.show_sensors:
                self.display_sensor_status()
                print()
            
            if self.show_map:
                self.display_map_status()
                print()
            
            print("Commands: h=help, s=toggle sensors, o=toggle map, q=quit")
            print("Control: w/s/a/d, Mapping: m/M, Navigation: n/N, Paths: 1/2")
            print("=" * 50)
    
    def update_display(self):
        """อัพเดตการแสดงผลอัตโนมัติ"""
        while self.running:
            if self.show_sensors or self.show_map:
                self.display_combined_status()
            time.sleep(0.5)
    
    def send_command(self, cmd):
        """ส่งคำสั่งไปยังหุ่นยนต์"""
        current_time = time.time()
        
        # Check command cooldown
        if current_time - self.last_command_time < self.command_cooldown:
            remaining = self.command_cooldown - (current_time - self.last_command_time)
            print(f"⏳ Wait {remaining:.1f}s before next command")
            return False
        
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd.encode())
                print(f"📤 Sent: {cmd}")
                self.last_command_time = current_time
                return True
            except Exception as e:
                print(f"❌ Error sending command: {e}")
                return False
        else:
            print("❌ Serial connection not available")
            return False
    
    def show_help(self):
        """แสดงคำสั่งที่ใช้ได้"""
        help_text = """
🎮 ROBOT CONTROL COMMANDS:
=======
    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Optimized Robot Controller v2.0")
        self.root.geometry("500x700")
        self.root.configure(bg='#2b2b2b')  # Dark theme for better visibility
        
        # Connection frame
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding="10")
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        conn_input_frame = ttk.Frame(conn_frame)
        conn_input_frame.pack(fill="x")
        
        ttk.Label(conn_input_frame, text="Port:").pack(side="left")
        self.port_var = tk.StringVar(value="COM3")
        port_entry = ttk.Entry(conn_input_frame, textvariable=self.port_var, width=8)
        port_entry.pack(side="left", padx=5)
        
        self.connect_btn = ttk.Button(conn_input_frame, text="Connect", 
                                    command=self.toggle_connection)
        self.connect_btn.pack(side="left", padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="Disconnected", 
                                    foreground="red", font=("Arial", 10, "bold"))
        self.status_label.pack(pady=5)
        
        # Performance metrics
        perf_frame = ttk.LabelFrame(self.root, text="Performance", padding="10")
        perf_frame.pack(fill="x", padx=10, pady=5)
        
        self.rate_label = ttk.Label(perf_frame, text="Data Rate: -- Hz", 
                                  font=("Arial", 10, "bold"), foreground="blue")
        self.rate_label.pack(anchor="w")
        
        self.count_label = ttk.Label(perf_frame, text="Packets: 0")
        self.count_label.pack(anchor="w")
        
        # Controls - Enhanced
        control_frame = ttk.LabelFrame(self.root, text="Manual Control", padding="10")
        control_frame.pack(fill="x", padx=10, pady=5)
        
        # Direction buttons in grid
        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack()
        
        # Movement buttons with better styling
        btn_style = {"width": 4, "font": ("Arial", 12, "bold")}
        
        ttk.Button(btn_frame, text="↑", command=lambda: self.send_cmd("w"), 
                  **btn_style).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(btn_frame, text="←", command=lambda: self.send_cmd("a"), 
                  **btn_style).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(btn_frame, text="STOP", command=lambda: self.send_cmd("x"), 
                  width=6, font=("Arial", 10, "bold")).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(btn_frame, text="→", command=lambda: self.send_cmd("d"), 
                  **btn_style).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(btn_frame, text="↓", command=lambda: self.send_cmd("s"), 
                  **btn_style).grid(row=2, column=1, padx=2, pady=2)
        
        # Quick action buttons
        action_frame = ttk.Frame(control_frame)
        action_frame.pack(pady=10)
        
        ttk.Button(action_frame, text="Test", command=lambda: self.send_cmd("test")).pack(side="left", padx=5)
        ttk.Button(action_frame, text="Status", command=lambda: self.send_cmd("status")).pack(side="left", padx=5)
        ttk.Button(action_frame, text="Rate Info", command=lambda: self.send_cmd("rate")).pack(side="left", padx=5)
        
        # Robot status with better formatting
        status_frame = ttk.LabelFrame(self.root, text="Robot Status", padding="10")
        status_frame.pack(fill="x", padx=10, pady=5)
        
        pos_frame = ttk.Frame(status_frame)
        pos_frame.pack(fill="x")
        
        self.pos_label = ttk.Label(pos_frame, text="Position: (25.0, 25.0)", 
                                 font=("Courier", 11, "bold"), foreground="green")
        self.pos_label.pack(anchor="w")
        
        self.heading_label = ttk.Label(pos_frame, text="Heading: 0.0°", 
                                     font=("Courier", 11, "bold"), foreground="green")
        self.heading_label.pack(anchor="w")
        
        # Map visualization frame
        map_frame = ttk.Frame(status_frame)
        map_frame.pack(fill="x", pady=5)
        
        ttk.Label(map_frame, text="Map: 50×50 cells (5m×5m)", 
                font=("Arial", 9)).pack(anchor="w")
        
        # Sensors with visual indicators
        sensor_frame = ttk.LabelFrame(self.root, text="Sensors (Real-time)", padding="10")
        sensor_frame.pack(fill="x", padx=10, pady=5)
        
        self.sensor_labels = []
        sensor_names = ["Front", "Right", "Back", "Left"]
        sensor_icons = ["🔼", "▶️", "🔽", "◀️"]
        
        for i, (name, icon) in enumerate(zip(sensor_names, sensor_icons)):
            sensor_row = ttk.Frame(sensor_frame)
            sensor_row.pack(fill="x", pady=2)
            
            label = ttk.Label(sensor_row, text=f"{icon} {name}: -- cm", 
                            font=("Courier", 11), width=20)
            label.pack(side="left")
            
            # Progress bar for distance visualization
            progress = ttk.Progressbar(sensor_row, length=100, maximum=200)
            progress.pack(side="left", padx=10, fill="x", expand=True)
            
            self.sensor_labels.append((label, progress))
        
        # Console with improved styling
        console_frame = ttk.LabelFrame(self.root, text="Console Log", padding="10")
        console_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        console_container = ttk.Frame(console_frame)
        console_container.pack(fill="both", expand=True)
        
        self.console = tk.Text(console_container, height=10, font=("Consolas", 9),
                             bg="#1e1e1e", fg="#ffffff", insertbackground="white")
        scroll = ttk.Scrollbar(console_container, command=self.console.yview)
        self.console.config(yscrollcommand=scroll.set)
        
        self.console.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        
        # Keyboard bindings
        self.root.bind('<KeyPress-w>', lambda e: self.send_cmd("w"))
        self.root.bind('<KeyPress-s>', lambda e: self.send_cmd("s"))
        self.root.bind('<KeyPress-a>', lambda e: self.send_cmd("a"))
        self.root.bind('<KeyPress-d>', lambda e: self.send_cmd("d"))
        self.root.bind('<KeyPress-x>', lambda e: self.send_cmd("x"))
        self.root.focus_set()  # Enable keyboard input
        
    def log(self, msg, tag="INFO"):
        """Enhanced logging with performance tracking"""
        timestamp = time.strftime("%H:%M:%S.%f")[:-3]  # Include milliseconds
        
        # Performance-aware logging
        if tag == "DATA" and hasattr(self, 'last_log_time'):
            if time.time() - self.last_log_time < 0.5:  # Limit data logs
                return
        
        full_msg = f"[{timestamp}] [{tag}] {msg}\n"
        
        self.console.insert(tk.END, full_msg)
        self.console.see(tk.END)
        
        # Keep console size manageable
        lines = int(self.console.index('end-1c').split('.')[0])
        if lines > 200:
            self.console.delete('1.0', '50.0')  # Remove first 50 lines
        
        # Color coding
        color_map = {
            "ERROR": "red",
            "SENSOR": "#00AAFF",
            "POSITION": "#00FF00",
            "COMMAND": "#FF8800",
            "PERFORMANCE": "#FF00FF",
            "DATA": "#FFFF00"
        }
        
        color = color_map.get(tag, "#CCCCCC")
        
        # Apply color
        last_line = self.console.index("end-2l")
        self.console.tag_add(tag, last_line, "end-1c")
        self.console.tag_config(tag, foreground=color)
        
        # Console output for debugging
        print(f"[{tag}] {msg}")
        
        if tag == "DATA":
            self.last_log_time = time.time()
        
    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()
            
    def connect(self):
        try:
            port = self.port_var.get()
            self.ser = serial.Serial(port, BAUD_RATE, timeout=0.1)  # Shorter timeout
            time.sleep(1)  # Reduced startup delay
            
            self.connected = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="Connected", foreground="green")
            
            # Reset performance counters
            self.data_count = 0
            self.last_data_time = time.time()
            self.data_history.clear()
            
            # Start reading thread
            self.data_thread = threading.Thread(target=self.read_data, daemon=True)
            self.data_thread.start()
            
            # Start performance monitoring thread
            self.perf_thread = threading.Thread(target=self.monitor_performance, daemon=True)
            self.perf_thread.start()
            
            self.log(f"Connected to {port} at {BAUD_RATE} baud", "INFO")
            self.log("High-speed data reception ready", "PERFORMANCE")
            
        except Exception as e:
            self.log(f"Connection failed: {e}", "ERROR")
            
    def disconnect(self):
        try:
            self.running = False
            if self.ser:
                self.ser.close()
            self.connected = False
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="Disconnected", foreground="red")
            self.log("Disconnected", "INFO")
        except Exception as e:
            self.log(f"Disconnect error: {e}", "ERROR")
            
    def send_cmd(self, command):
        if self.connected and self.ser:
            try:
                self.ser.write((command + '\n').encode())
                self.log(f"→ {command}", "COMMAND")
            except Exception as e:
                self.log(f"Send error: {e}", "ERROR")
        else:
            self.log("Not connected!", "ERROR")
            
    def read_data(self):
        """Optimized data reading with minimal processing"""
        buffer = ""
        
        while self.running and self.connected and self.ser:
            try:
                if self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += chunk
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip():
                            self.process_data(line.strip())
                            
                time.sleep(0.001)  # Minimal delay for high-speed processing
                
            except Exception as e:
                self.log(f"Read error: {e}", "ERROR")
                break
                
    def process_data(self, line):
        """Optimized data processing with performance tracking"""
        current_time = time.time()
        
        # Parse robot position data
        robot_match = self.robot_pattern.search(line)
        if robot_match:
            try:
                x, y, heading = map(float, robot_match.groups()[:3])
                sensors = list(map(float, robot_match.groups()[3:7]))
                
                self.update_robot_data(x, y, heading, sensors, current_time)
                return
                
            except (ValueError, IndexError) as e:
                self.log(f"Parse error (ROBOT): {e}", "ERROR")
        
        # Parse standard position format
        pos_match = self.pos_pattern.search(line)
        if pos_match:
            try:
                x, y, heading = map(float, pos_match.groups()[:3])
                sensors = list(map(float, pos_match.groups()[3:7]))
                
                self.update_robot_data(x, y, heading, sensors, current_time)
                return
                
            except (ValueError, IndexError) as e:
                self.log(f"Parse error (POS): {e}", "ERROR")
        
        # Log other important messages
        if any(kw in line for kw in ['[ERROR]', '[SUCCESS]', '[WARNING]', '[STATUS]']):
            self.log(line, "INFO")
            
    def update_robot_data(self, x, y, heading, sensors, timestamp):
        """Update robot data with performance tracking"""
        # Update robot state
        self.robot_pos = [x, y]
        self.robot_heading = heading
        self.sensor_data = sensors
        
        # Performance tracking
        self.data_count += 1
        self.data_history.append(timestamp)
        
        # Calculate data rate
        if len(self.data_history) >= 2:
            time_span = self.data_history[-1] - self.data_history[0]
            if time_span > 0:
                self.data_rate = (len(self.data_history) - 1) / time_span
        
        # Update GUI
        self.root.after(0, self.update_gui)
        
        # Selective logging to avoid spam
        if self.data_count % 10 == 0:  # Log every 10th update
            self.log(f"Pos: ({x:.1f}, {y:.1f}) @ {heading:.1f}° | Rate: {self.data_rate:.1f}Hz", "DATA")
            
    def monitor_performance(self):
        """Background performance monitoring"""
        while self.running and self.connected:
            try:
                time.sleep(1)  # Update every second
                self.root.after(0, self.update_performance_display)
            except:
                break
                
    def update_performance_display(self):
        """Update performance metrics in GUI"""
        self.rate_label.config(text=f"Data Rate: {self.data_rate:.1f} Hz")
        self.count_label.config(text=f"Packets: {self.data_count}")
        
        # Color code data rate
        if self.data_rate >= 8.0:
            color = "green"
        elif self.data_rate >= 5.0:
            color = "orange"
        else:
            color = "red"
            
        self.rate_label.config(foreground=color)
        
    def update_gui(self):
        """Update GUI with current data"""
        try:
            # Update position
            self.pos_label.config(text=f"Position: ({self.robot_pos[0]:.1f}, {self.robot_pos[1]:.1f})")
            self.heading_label.config(text=f"Heading: {self.robot_heading:.1f}°")
            
            # Update sensors with visual feedback
            sensor_names = ["Front", "Right", "Back", "Left"]
            sensor_icons = ["🔼", "▶️", "🔽", "◀️"]
            
            for i, (name, icon) in enumerate(zip(sensor_names, sensor_icons)):
                distance = self.sensor_data[i]
                label, progress = self.sensor_labels[i]
                
                if 0 < distance < 999:
                    # Update text with color coding
                    if distance < 20:
                        color, status = "red", "⚠️"
                    elif distance < 50:
                        color, status = "orange", "⚡"
                    else:
                        color, status = "green", "✅"
                    
                    label.config(text=f"{icon} {name}: {distance:.1f}cm {status}", foreground=color)
                    
                    # Update progress bar
                    progress_value = min(distance, 200)  # Cap at 200cm
                    progress['value'] = progress_value
                    
                else:
                    label.config(text=f"{icon} {name}: -- cm ❌", foreground="gray")
                    progress['value'] = 0
                    
        except Exception as e:
            self.log(f"GUI update error: {e}", "ERROR")
            
    def run(self):
        """Start the application"""
        self.log("Optimized Robot Controller v2.0 Started", "INFO")
        self.log("Features: Fast data updates, performance monitoring", "INFO")
        self.log("Keyboard controls: WASD for movement, X for stop", "INFO")
        
        try:
            self.root.mainloop()
        finally:
            self.running = False
>>>>>>> Stashed changes

📱 Manual Control:
  w - Move Forward      s - Move Backward  
  a - Turn Left         d - Turn Right
  x - Stop Motors

🗺️ Autonomous Mapping:
  m - Start Mapping Mode
  M - Stop Mapping Mode

🎯 Path Planning:
  n - Start Navigation    N - Stop Navigation
  c - Clear Waypoints     l - List Waypoints

📋 Pre-defined Paths:
  1 - Load Competition Path 1 (Rectangle)
  2 - Load Competition Path 2 (Obstacle Course)

🖥️ Display Controls:
  s - Toggle Sensor Display
  o - Toggle Map Display  
  r - Refresh Display

ℹ️ Other:
  h - Show this help
  q - Quit program

💡 Tips:
- Robot auto-adjusts speed based on obstacles
- Minimum speed: 25% (to ensure motor works)
- Use 's' and 'o' to monitor robot in real-time
- Red sensors = danger zone, adjust path accordingly
        """
        print(help_text)
    
    def run(self):
        """รันโปรแกรมหลัก"""
        print("🚀 Simple Robot Controller with Live Display")
        print("📋 Type 'h' for help, 'q' to quit")
        print("🎯 Type 's' for sensors, 'o' for map display")
        print("⌨️  Waiting for commands...")
        
        while self.running:
            try:
                # Get user input
                cmd = input().strip().lower()
                
                if not cmd:
                    continue
                    
                # Process commands
                if cmd == 'q':
                    print("👋 Shutting down...")
                    break
                elif cmd == 'h':
                    self.show_help()
                elif cmd == 's':
                    self.show_sensors = not self.show_sensors
                    print(f"📊 Sensor display: {'ON' if self.show_sensors else 'OFF'}")
                    if not self.show_sensors and not self.show_map:
                        self.clear_screen()
                elif cmd == 'o':
                    self.show_map = not self.show_map
                    print(f"🗺️ Map display: {'ON' if self.show_map else 'OFF'}")
                    if not self.show_sensors and not self.show_map:
                        self.clear_screen()
                elif cmd == 'r':
                    if self.show_sensors or self.show_map:
                        self.display_combined_status()
                    else:
                        print("💡 Enable sensor or map display first (s/o)")
                elif cmd in ['w', 's', 'a', 'd', 'x']:
                    # Manual control
                    command_map = {
                        'w': 'forward',
                        's': 'backward', 
                        'a': 'turn_left',
                        'd': 'turn_right',
                        'x': 'stop'
                    }
                    self.send_command(command_map[cmd])
                elif cmd == 'm':
                    print("🗺️ Starting mapping mode...")
                    self.send_command('start_mapping')
                elif cmd == 'M':
                    print("🛑 Stopping mapping mode...")
                    self.send_command('stop_mapping')
                elif cmd == 'n':
                    print("🎯 Starting navigation...")
                    self.send_command('wp_start')
                elif cmd == 'N':
                    print("🛑 Stopping navigation...")
                    self.send_command('wp_stop')
                elif cmd == 'c':
                    print("🗑️ Clearing waypoints...")
                    self.send_command('wp_clear')
                elif cmd == 'l':
                    print("📋 Listing waypoints...")
                    self.send_command('wp_list')
                elif cmd == '1':
                    print("📋 Loading competition path 1...")
                    self.send_command('load_path_1')
                elif cmd == '2':
                    print("📋 Loading competition path 2...")
                    self.send_command('load_path_2')
                elif cmd.startswith('wp_add:'):
                    # Custom waypoint: wp_add:100,50
                    self.send_command(cmd)
                else:
                    print(f"❓ Unknown command: {cmd}")
                    print("💡 Type 'h' for help")
                    
            except KeyboardInterrupt:
                print("\n👋 Interrupted by user")
                break
            except EOFError:
                print("\n👋 EOF received")
                break
            except Exception as e:
                print(f"❌ Error: {e}")
        
        # Cleanup
        self.running = False
        if self.ser:
            self.ser.close()
        print("✅ Controller stopped")

def main():
    """ฟังก์ชันหลัก"""
    print("🤖 Enhanced Robot Controller with Live Display")
    print("=" * 60)
    
    # เช็คพอร์ต
    print(f"🔌 Attempting to connect to {SERIAL_PORT}")
    print("💡 If connection fails, check:")
    print("   1. ESP32 is connected and powered")
    print("   2. Correct port in SERIAL_PORT variable") 
    print("   3. No other programs using the port")
    print()
    
    try:
        controller = OptimizedRobotController()
        controller.run()
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        sys.exit(1)

if __name__ == "__main__":
>>>>>>> Stashed changes
    main()