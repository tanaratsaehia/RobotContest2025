<<<<<<< Updated upstream
<<<<<<< Updated upstream
import serial
import sys
import threading
import platform
import time

<<<<<<< Updated upstream
# Replace with your ESP32 serial port
SERIAL_PORT = '/dev/ttyUSB1'  # e.g., 'COM3' on Windows
=======
import serial, sys, threading, re, time

SERIAL_PORT = 'COM6'  # <-- Change to your actual COM port
>>>>>>> Stashed changes
BAUD_RATE = 115200
=======
# การตั้งค่า Serial Communication
# พอร์ตนี้ขึ้นอยู่กับระบบปฏิบัติการและการเชื่อมต่อของคุณ
SERIAL_PORT = "COM6"   # Windows: COM3, COM4, etc. | Linux: /dev/ttyUSB0 | Mac: /dev/tty.SLAB_USBtoUART
BAUD_RATE = 115200     # ความเร็วการสื่อสาร (ต้องตรงกับใน ESP32)
>>>>>>> Stashed changes

<<<<<<< Updated upstream
# ตัวแปรสำหรับเก็บสถานะ
current_speed = 50     # ความเร็วปัจจุบัน (เริ่มต้นที่ 50%)
speed_step = 10        # ขนาดการเพิ่ม/ลดความเร็วแต่ละครั้ง
min_speed = 10         # ความเร็วต่ำสุด
max_speed = 100        # ความเร็วสูงสุด
last_command = "STOP"  # คำสั่งสุดท้ายที่ส่ง

# เปิด Serial connection พร้อมการจัดการข้อผิดพลาด
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, dsrdtr=False)
    print(f"[System] เชื่อมต่อ ESP32 สำเร็จที่ {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"[Error] ไม่สามารถเชื่อมต่อ {SERIAL_PORT}: {e}")
    sys.exit(1)

# ตรวจสอบระบบปฏิบัติการเพื่อใช้ฟังก์ชันอ่าน keyboard ที่เหมาะสม
is_windows = platform.system() == "Windows"

if is_windows:
    import msvcrt
else:
    import tty
    import termios

def getch():
    """
    ฟังก์ชันอ่านการกดปุ่มแบบ real-time โดยไม่ต้องกด Enter
    รองรับทั้ง Windows, Linux และ Mac
    """
    if is_windows:
        # Windows: ใช้ msvcrt module
        return msvcrt.getch().decode("utf-8", errors="ignore")
    else:
        # Unix-based systems: ใช้ termios เพื่อปรับโหมด terminal
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)  # เปลี่ยนเป็น raw mode
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # คืนค่า setting เดิม
=======
# Sensor <num>: <float> cm
SENSOR_RE = re.compile(r"Sensor\s*(\d+)\s*:\s*([+-]?\d+(?:\.\d+)?)\s*cm", re.I)
sensor_vals = {1: None, 2: None, 3: None, 4: None}

# Cross-platform getch
if sys.platform == 'win32':
    import msvcrt
    def getch():
        return msvcrt.getch().decode('utf-8')
else:
    import tty, termios
    def getch():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
>>>>>>> Stashed changes
        return ch

def read_serial():
    """
    Thread สำหรับอ่านข้อมูลจาก ESP32 แบบต่อเนื่อง
    ทำงานแบบ background เพื่อไม่ให้บล็อกการกดปุ่มของผู้ใช้
    """
    while True:
<<<<<<< Updated upstream
        try:
            if ser.in_waiting > 0:  # ตรวจสอบว่ามีข้อมูลรออยู่หรือไม่
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    print(f"\n[ESP32] {line}")
                    display_status()  # แสดงสถานะปัจจุบันหลังจากได้รับข้อมูล
        except serial.SerialException:
            print("\n[Error] การเชื่อมต่อ Serial ขาดหาย")
=======
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode('utf-8', errors='ignore').strip()
        m = SENSOR_RE.search(line)
        if not m:
            continue
        idx = int(m.group(1))
        val = float(m.group(2))
        if 1 <= idx <= 4:
            sensor_vals[idx] = val

def render_loop():
    # Allocate exactly 4 lines once
    sys.stdout.write("\n" * 4)
    sys.stdout.flush()
    while True:
        sys.stdout.write("\033[4F")  # Move up 4 lines
        for i in range(1, 5):
            v = sensor_vals[i]
            line = f"S{i}: {v:.1f} cm" if v is not None else f"S{i}: —"
            sys.stdout.write("\r\033[K" + line + "\n")
        sys.stdout.flush()
        time.sleep(0.1)

threading.Thread(target=read_serial, daemon=True).start()
threading.Thread(target=render_loop, daemon=True).start()

try:
    while True:
        key = getch()
        if key == 'q':
>>>>>>> Stashed changes
            break
        except KeyboardInterrupt:
            break

def send_command_to_esp32(command):
    """
    ส่งคำสั่งไป ESP32 พร้อมจัดการข้อผิดพลาด
    """
    global last_command
    try:
        # ส่งคำสั่งในรูปแบบที่ ESP32 เข้าใจได้
        command_with_speed = f"{command}:{current_speed}"
        ser.write(command_with_speed.encode())
        last_command = command
        print(f"[Sent] {command_with_speed}")
    except serial.SerialException as e:
        print(f"[Error] ส่งคำสั่งไม่สำเร็จ: {e}")

def increase_speed():
    """เพิ่มความเร็วโดยไม่เกินค่าสูงสุด"""
    global current_speed
    if current_speed < max_speed:
        current_speed += speed_step
        print(f"[Speed] เพิ่มความเร็วเป็น {current_speed}%")
        # ส่งคำสั่งสุดท้ายด้วยความเร็วใหม่
        if last_command != "STOP":
            send_command_to_esp32(last_command)
    else:
        print(f"[Speed] ความเร็วสูงสุดแล้ว ({max_speed}%)")

def decrease_speed():
    """ลดความเร็วโดยไม่ต่ำกว่าค่าต่ำสุด"""
    global current_speed
    if current_speed > min_speed:
        current_speed -= speed_step
        print(f"[Speed] ลดความเร็วเป็น {current_speed}%")
        # ส่งคำสั่งสุดท้ายด้วยความเร็วใหม่
        if last_command != "STOP":
            send_command_to_esp32(last_command)
    else:
        print(f"[Speed] ความเร็วต่ำสุดแล้ว ({min_speed}%)")

def display_status():
    """แสดงสถานะปัจจุบันของระบบ"""
    print(f"\n{'='*50}")
    print(f"สถานะ: {last_command.ljust(10)} | ความเร็ว: {current_speed}%")
    print(f"{'='*50}")

def display_help():
    """แสดงคำแนะนำการใช้งาน"""
    help_text = """
    ╔════════════════════════════════════════════════════════════╗
    ║                     คู่มือการใช้งาน                        ║
    ╠════════════════════════════════════════════════════════════╣
    ║  การเคลื่อนที่:                                             ║
    ║    W - เคลื่อนที่ไปข้างหน้า      S - ถอยหลัง               ║
    ║    A - เลี้ยวซ้าย                D - เลี้ยวขวา              ║
    ║    X - หยุด (Emergency Stop)                               ║
    ║                                                            ║
    ║  การควบคุมความเร็ว:                                         ║
    ║    + หรือ = - เพิ่มความเร็ว       - หรือ _ - ลดความเร็ว      ║
    ║                                                            ║
    ║  คำสั่งอื่นๆ:                                               ║
    ║    H - แสดงคำแนะนำนี้            Q - ออกจากโปรแกรม         ║
    ║    I - แสดงสถานะปัจจุบัน                                    ║
    ╚════════════════════════════════════════════════════════════╝
    """
    print(help_text)

def main():
    """ฟังก์ชันหลักของโปรแกรม"""
    
    # เริ่มต้น thread สำหรับอ่าน serial
    serial_thread = threading.Thread(target=read_serial, daemon=True)
    serial_thread.start()
    
    # แสดงข้อมูลเริ่มต้น
    print("\n" + "="*60)
    print("🤖 ESP32 Robot Controller with Variable Speed Control 🤖")
    print("="*60)
    display_help()
    display_status()
    
    try:
        while True:
            key = getch().lower()  # อ่านการกดปุ่มและแปลงเป็นตัวพิมพ์เล็ก
            
            # จัดการคำสั่งเคลื่อนที่พื้นฐาน
            if key == 'w':
                print("[Action] ไปข้างหน้า")
                send_command_to_esp32("FORWARD")
                
            elif key == 's':
                print("[Action] ถอยหลัง")
                send_command_to_esp32("BACKWARD")
                
            elif key == 'a':
                print("[Action] เลี้ยวซ้าย")
                send_command_to_esp32("LEFT")
                
            elif key == 'd':
                print("[Action] เลี้ยวขวา")
                send_command_to_esp32("RIGHT")
                
            elif key == 'x' or key == ' ':  # เว้นวรรคหรือ x สำหรับหยุดฉุกเฉิน
                print("[Action] หยุดฉุกเฉิน!")
                send_command_to_esp32("STOP")
                
            # จัดการคำสั่งปรับความเร็ว
            elif key == '+' or key == '=':
                increase_speed()
                
            elif key == '-' or key == '_':
                decrease_speed()
                
            # คำสั่งแสดงข้อมูล
            elif key == 'h':
                display_help()
                
            elif key == 'i':
                display_status()
                
            # ออกจากโปรแกรม
            elif key == 'q':
                print("\n[System] กำลังออกจากโปรแกรม...")
                send_command_to_esp32("STOP")  # หยุดหุ่นยนต์ก่อนออก
                break
                
            # กรณีกดปุ่มที่ไม่รู้จัก
            else:
                if ord(key) >= 32:  # ตัวอักษรที่พิมพ์ได้
                    print(f"[Warning] ไม่รู้จักคำสั่ง '{key}' - กด 'H' เพื่อดูคำแนะนำ")
            
            # แสดงสถานะหลังจากทำคำสั่ง (ยกเว้นคำสั่งแสดงข้อมูล)
            if key not in ['h', 'i', 'q']:
                display_status()
                
    except KeyboardInterrupt:
        print("\n[System] ได้รับสัญญาณ Ctrl+C - กำลังออกจากโปรแกรม...")
    
    finally:
        # ปิดการเชื่อมต่ออย่างปลอดภัย
        try:
            send_command_to_esp32("STOP")  # หยุดหุ่นยนต์ก่อนปิด
            time.sleep(0.5)  # รอให้คำสั่งส่งเสร็จ
            ser.close()
            print("[System] ปิดการเชื่อมต่อ Serial แล้ว")
        except:
            pass

if __name__ == "__main__":
    main()
=======
#!/usr/bin/env python3
"""
Complete Autonomous Robot Mapping Controller
Robot Contest 2025 [Robot Team]
"""

import serial
import sys
import threading
import re
import time
import tkinter as tk
from tkinter import ttk, messagebox
import json
from datetime import datetime

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    import numpy as np
    from matplotlib.animation import FuncAnimation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib/numpy not available. Map visualization disabled.")

# Configuration for Windows
SERIAL_PORTS = ['COM1', 'COM2', 'COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8', 'COM9', 'COM10']
BAUD_RATE = 115200

# Mapping parameters
MAP_SIZE = 100
GRID_SIZE = 10  # cm per grid cell

class AutoMappingController:
    def __init__(self):
        self.ser = None
        self.connected = False
        self.mapping_active = False
        
        # Map data
        if MATPLOTLIB_AVAILABLE:
            self.occupancy_map = np.full((MAP_SIZE, MAP_SIZE), -1, dtype=np.int8)
        else:
            self.occupancy_map = [[-1 for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]
        
        self.robot_pos = [MAP_SIZE//2, MAP_SIZE//2]
        self.robot_heading = 0.0
        self.sensor_data = [0.0, 0.0, 0.0, 0.0]
        self.path_history = []
        
        # Data parsing
        self.position_pattern = re.compile(r"POS:([\d.-]+),([\d.-]+),([\d.-]+)\|S:([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+)")
        self.sensor_pattern = re.compile(r"Sensor\s*(\d+)\s*:\s*([\d.-]+)\s*cm", re.I)
        
        # GUI
        self.setup_gui()
        
        if MATPLOTLIB_AVAILABLE:
            self.setup_plot()
        
        # Start data reading thread
        self.running = True
        self.data_thread = None
        
    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Autonomous Robot Mapping Controller")
        self.root.geometry("500x700")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Title
        title_label = tk.Label(self.root, text="Robot Mapping Controller", 
                              font=("Arial", 16, "bold"), fg="blue")
        title_label.pack(pady=10)
        
        # Connection frame
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding="10")
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(conn_frame, text="COM Port:").pack(anchor="w")
        self.port_var = tk.StringVar(value="COM3")
        port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, values=SERIAL_PORTS, width=15)
        port_combo.pack(pady=2)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(pady=5)
        
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.pack()
        
        # Control frame
        control_frame = ttk.LabelFrame(self.root, text="Robot Control", padding="10")
        control_frame.pack(fill="x", padx=10, pady=5)
        
        # Mapping controls
        mapping_frame = ttk.Frame(control_frame)
        mapping_frame.pack(fill="x", pady=5)
        
        self.start_mapping_btn = ttk.Button(mapping_frame, text="Start Auto Mapping", 
                                          command=self.start_autonomous_mapping)
        self.start_mapping_btn.pack(side="left", padx=2)
        
        self.stop_mapping_btn = ttk.Button(mapping_frame, text="Stop Mapping", 
                                         command=self.stop_autonomous_mapping)
        self.stop_mapping_btn.pack(side="left", padx=2)
        
        # Manual controls
        manual_frame = ttk.LabelFrame(control_frame, text="Manual Control", padding="5")
        manual_frame.pack(fill="x", pady=5)
        
        # Direction buttons
        dir_frame = ttk.Frame(manual_frame)
        dir_frame.pack()
        
        ttk.Button(dir_frame, text="↑", command=lambda: self.send_command("w"), width=3).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(dir_frame, text="←", command=lambda: self.send_command("a"), width=3).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(dir_frame, text="■", command=lambda: self.send_command("x"), width=3).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(dir_frame, text="→", command=lambda: self.send_command("d"), width=3).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(dir_frame, text="↓", command=lambda: self.send_command("s"), width=3).grid(row=2, column=1, padx=2, pady=2)
        
        # Status frame
        status_frame = ttk.LabelFrame(self.root, text="Robot Status", padding="10")
        status_frame.pack(fill="x", padx=10, pady=5)
        
        self.position_label = ttk.Label(status_frame, text="Position: (50, 50)")
        self.position_label.pack(anchor="w")
        
        self.heading_label = ttk.Label(status_frame, text="Heading: 0°")
        self.heading_label.pack(anchor="w")
        
        self.mapping_status_label = ttk.Label(status_frame, text="Mapping: Inactive")
        self.mapping_status_label.pack(anchor="w")
        
        # Sensor data frame
        sensor_frame = ttk.LabelFrame(self.root, text="Ultrasonic Sensors", padding="10")
        sensor_frame.pack(fill="x", padx=10, pady=5)
        
        self.sensor_labels = []
        sensor_names = ["Front", "Right", "Back", "Left"]
        for i, name in enumerate(sensor_names):
            label = ttk.Label(sensor_frame, text=f"{name}: -- cm")
            label.pack(anchor="w")
            self.sensor_labels.append(label)
        
        # Map controls
        map_frame = ttk.LabelFrame(self.root, text="Map Controls", padding="10")
        map_frame.pack(fill="x", padx=10, pady=5)
        
        map_btn_frame = ttk.Frame(map_frame)
        map_btn_frame.pack(fill="x")
        
        if MATPLOTLIB_AVAILABLE:
            ttk.Button(map_btn_frame, text="Show Map", command=self.show_map).pack(side="left", padx=2)
        
        ttk.Button(map_btn_frame, text="Clear Map", command=self.clear_map).pack(side="left", padx=2)
        ttk.Button(map_btn_frame, text="Save Map", command=self.save_map).pack(side="left", padx=2)
        
        # Console frame
        console_frame = ttk.LabelFrame(self.root, text="Console", padding="10")
        console_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Console text area
        console_scroll = ttk.Scrollbar(console_frame)
        console_scroll.pack(side="right", fill="y")
        
        self.console_text = tk.Text(console_frame, height=8, yscrollcommand=console_scroll.set, 
                                   font=("Consolas", 9))
        self.console_text.pack(fill="both", expand=True)
        console_scroll.config(command=self.console_text.yview)
        
        # Control instructions
        instructions = """
        Controls:
        W/↑ - Forward    S/↓ - Backward
        A/← - Turn Left  D/→ - Turn Right
        X/■ - Stop       M - Toggle Auto Mapping
        """
        
        instr_frame = ttk.LabelFrame(self.root, text="Instructions", padding="5")
        instr_frame.pack(fill="x", padx=10, pady=5)
        
        instr_label = ttk.Label(instr_frame, text=instructions, font=("Arial", 8))
        instr_label.pack()
        
        # Keyboard binding
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.focus_set()
        
    def setup_plot(self):
        if not MATPLOTLIB_AVAILABLE:
            return
            
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlim(0, MAP_SIZE)
        self.ax.set_ylim(0, MAP_SIZE)
        self.ax.set_aspect('equal')
        self.ax.set_title('Robot Autonomous Mapping')
        self.ax.set_xlabel('X (grid cells, 10cm each)')
        self.ax.set_ylabel('Y (grid cells, 10cm each)')
        self.ax.grid(True, alpha=0.3)
        
        # Initialize plot elements
        self.map_image = self.ax.imshow(self.occupancy_map.T, origin='lower', cmap='RdYlBu_r', 
                                       vmin=-1, vmax=1, alpha=0.8)
        self.robot_marker, = self.ax.plot([], [], 'ro', markersize=8, label='Robot')
        self.robot_direction, = self.ax.plot([], [], 'r-', linewidth=2)
        self.path_line, = self.ax.plot([], [], 'b-', alpha=0.5, linewidth=1, label='Path')
        self.sensor_lines = [self.ax.plot([], [], 'g--', alpha=0.7)[0] for _ in range(4)]
        
        self.ax.legend()
        plt.tight_layout()
        
    def log_message(self, message):
        """Add message to console"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        full_message = f"[{timestamp}] {message}\n"
        
        self.console_text.insert(tk.END, full_message)
        self.console_text.see(tk.END)
        
        # Keep only last 1000 lines
        lines = self.console_text.get("1.0", tk.END).split('\n')
        if len(lines) > 1000:
            self.console_text.delete("1.0", f"{len(lines)-1000}.0")
        
        print(full_message.strip())  # Also print to stdout
        
    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()
            
    def connect(self):
        try:
            port = self.port_var.get()
            self.log_message(f"Attempting to connect to {port}...")
            
            self.ser = serial.Serial(port, BAUD_RATE, timeout=1, dsrdtr=False)
            time.sleep(2)  # Wait for ESP32 to reset
            
            self.connected = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="Status: Connected", foreground="green")
            
            # Start data reading thread
            if self.data_thread is None or not self.data_thread.is_alive():
                self.data_thread = threading.Thread(target=self.read_serial_data, daemon=True)
                self.data_thread.start()
                
            self.log_message(f"✓ Connected to {port}")
            
        except Exception as e:
            self.log_message(f"✗ Connection failed: {str(e)}")
            messagebox.showerror("Connection Error", f"Failed to connect to {self.port_var.get()}:\n{str(e)}")
            
    def disconnect(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = None
            self.connected = False
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="Status: Disconnected", foreground="red")
            self.log_message("Disconnected from robot")
        except Exception as e:
            self.log_message(f"Error during disconnect: {e}")
        
    def send_command(self, command):
        if self.connected and self.ser and self.ser.is_open:
            try:
                self.ser.write((command + '\n').encode())
                self.log_message(f"→ Sent: {command}")
            except Exception as e:
                self.log_message(f"✗ Send error: {e}")
        else:
            self.log_message("✗ Not connected to robot")
                
    def start_autonomous_mapping(self):
        if self.connected:
            self.send_command("start_mapping")
            self.mapping_active = True
            self.mapping_status_label.config(text="Mapping: Active", foreground="green")
            self.log_message("🤖 Started autonomous mapping")
        else:
            self.log_message("✗ Connect to robot first")
            
    def stop_autonomous_mapping(self):
        if self.connected:
            self.send_command("stop_mapping")
            self.mapping_active = False
            self.mapping_status_label.config(text="Mapping: Inactive", foreground="red")
            self.log_message("⏹ Stopped autonomous mapping")
        else:
            self.log_message("✗ Connect to robot first")
            
    def read_serial_data(self):
        self.log_message("📡 Started data reading thread")
        
        while self.running and self.connected and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.process_serial_data(line)
                time.sleep(0.01)
            except Exception as e:
                self.log_message(f"✗ Serial read error: {e}")
                break
                
        self.log_message("📡 Data reading thread stopped")
                
    def process_serial_data(self, line):
        # Skip debug messages but log them
        if any(keyword in line for keyword in ['[SETUP]', '[IMU]', '[ESP32]', '[AUTO]', '[STATUS]', '[READY]']):
            self.log_message(f"🤖 {line}")
            return
            
        if line.startswith('[') or 'Button state' in line or 'gyro:' in line:
            return
            
        # Parse position and sensor data
        pos_match = self.position_pattern.search(line)
        if pos_match:
            try:
                x, y, heading = float(pos_match.group(1)), float(pos_match.group(2)), float(pos_match.group(3))
                sensors = [float(pos_match.group(i)) for i in range(4, 8)]
                
                self.robot_pos = [x, y]
                self.robot_heading = heading
                self.sensor_data = sensors
                
                # Add to path history
                self.path_history.append((x, y))
                if len(self.path_history) > 1000:
                    self.path_history = self.path_history[-1000:]
                
                # Update map based on sensor data
                if MATPLOTLIB_AVAILABLE:
                    self.update_map_from_sensors()
                
                # Update GUI labels
                self.root.after(0, self.update_gui_labels)
                
            except ValueError as e:
                self.log_message(f"✗ Position parse error: {e}")
                
        # Parse individual sensor readings
        sensor_match = self.sensor_pattern.search(line)
        if sensor_match:
            try:
                sensor_id = int(sensor_match.group(1)) - 1
                distance = float(sensor_match.group(2))
                if 0 <= sensor_id < 4:
                    self.sensor_data[sensor_id] = distance
                    # Update GUI immediately for sensor data
                    self.root.after(0, self.update_gui_labels)
                    # Log sensor data occasionally
                    if distance < 30:  # Only log close objects
                        sensor_names = ["Front", "Right", "Back", "Left"]
                        self.log_message(f"📏 {sensor_names[sensor_id]}: {distance:.1f}cm")
            except ValueError:
                pass
                
    def update_map_from_sensors(self):
        """Update occupancy map based on current sensor readings"""
        if not MATPLOTLIB_AVAILABLE:
            return
            
        if not all(isinstance(x, (int, float)) for x in self.robot_pos):
            return
            
        grid_x, grid_y = int(self.robot_pos[0]), int(self.robot_pos[1])
        
        # Mark current position as free
        if 0 <= grid_x < MAP_SIZE and 0 <= grid_y < MAP_SIZE:
            self.occupancy_map[grid_x, grid_y] = 0
            
        # Process each sensor (Front=0°, Right=90°, Back=180°, Left=270°)
        sensor_angles = [0, 90, 180, 270]
        
        for i, (angle_offset, distance) in enumerate(zip(sensor_angles, self.sensor_data)):
            if distance > 300 or distance < 2:
                continue
                
            absolute_angle = (self.robot_heading + angle_offset) % 360
            rad = np.radians(absolute_angle)
            
            # Trace ray from robot to obstacle
            max_range = min(distance / GRID_SIZE, 30)
            
            for step in np.arange(0.5, max_range, 0.5):
                ray_x = grid_x + int(step * np.cos(rad))
                ray_y = grid_y + int(step * np.sin(rad))
                
                if 0 <= ray_x < MAP_SIZE and 0 <= ray_y < MAP_SIZE:
                    if step >= max_range - 1:
                        self.occupancy_map[ray_x, ray_y] = 1  # Occupied
                    else:
                        if self.occupancy_map[ray_x, ray_y] == -1:
                            self.occupancy_map[ray_x, ray_y] = 0  # Free
                            
    def update_gui_labels(self):
        """Update GUI labels with current robot status"""
        try:
            self.position_label.config(text=f"Position: ({self.robot_pos[0]:.1f}, {self.robot_pos[1]:.1f})")
            self.heading_label.config(text=f"Heading: {self.robot_heading:.1f}°")
            
            sensor_names = ["Front", "Right", "Back", "Left"]
            for i, (name, distance) in enumerate(zip(sensor_names, self.sensor_data)):
                if distance < 999:
                    self.sensor_labels[i].config(text=f"{name}: {distance:.1f} cm")
                else:
                    self.sensor_labels[i].config(text=f"{name}: -- cm")
        except:
            pass
                
    def show_map(self):
        """Display the real-time map"""
        if not MATPLOTLIB_AVAILABLE:
            messagebox.showwarning("Map Unavailable", "Matplotlib not installed. Cannot show map.")
            return
            
        self.update_plot()
        plt.show(block=False)
        
        # Start animation for real-time updates
        if hasattr(self, 'animation'):
            self.animation.event_source.stop()
        self.animation = FuncAnimation(self.fig, self.animate, interval=500, blit=False)
        self.log_message("🗺 Map window opened")
        
    def update_plot(self):
        """Update the map visualization"""
        if not MATPLOTLIB_AVAILABLE:
            return
            
        # Update map image
        self.map_image.set_array(self.occupancy_map.T)
        
        # Update robot position
        if self.robot_pos:
            self.robot_marker.set_data([self.robot_pos[0]], [self.robot_pos[1]])
            
            # Draw robot direction arrow
            arrow_length = 3
            dx = arrow_length * np.cos(np.radians(self.robot_heading))
            dy = arrow_length * np.sin(np.radians(self.robot_heading))
            self.robot_direction.set_data([self.robot_pos[0], self.robot_pos[0] + dx],
                                        [self.robot_pos[1], self.robot_pos[1] + dy])
        
        # Update path
        if len(self.path_history) > 1:
            path_x, path_y = zip(*self.path_history)
            self.path_line.set_data(path_x, path_y)
            
        # Update sensor rays
        if self.robot_pos and MATPLOTLIB_AVAILABLE:
            sensor_angles = [0, 90, 180, 270]
            colors = ['red', 'green', 'blue', 'orange']
            
            for i, (angle_offset, distance, line) in enumerate(zip(sensor_angles, self.sensor_data, self.sensor_lines)):
                if distance < 300 and distance > 2:
                    absolute_angle = (self.robot_heading + angle_offset) % 360
                    rad = np.radians(absolute_angle)
                    
                    end_x = self.robot_pos[0] + (distance / GRID_SIZE) * np.cos(rad)
                    end_y = self.robot_pos[1] + (distance / GRID_SIZE) * np.sin(rad)
                    
                    line.set_data([self.robot_pos[0], end_x], [self.robot_pos[1], end_y])
                    line.set_color(colors[i])
                else:
                    line.set_data([], [])
                    
    def animate(self, frame):
        """Animation function for real-time map updates"""
        try:
            if MATPLOTLIB_AVAILABLE:
                self.update_plot()
                return [self.map_image, self.robot_marker, self.robot_direction, self.path_line] + self.sensor_lines
            return []
        except:
            return []
        
    def clear_map(self):
        """Clear the current map"""
        if MATPLOTLIB_AVAILABLE:
            self.occupancy_map.fill(-1)
        else:
            self.occupancy_map = [[-1 for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]
        
        self.path_history.clear()
        self.log_message("🗺 Map cleared")
        
    def save_map(self):
        """Save the current map and robot data"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"robot_map_{timestamp}.json"
        
        # Convert numpy array to list if needed
        if MATPLOTLIB_AVAILABLE:
            map_data_list = self.occupancy_map.tolist()
        else:
            map_data_list = self.occupancy_map
            
        map_data = {
            'timestamp': timestamp,
            'map_size': MAP_SIZE,
            'grid_size': GRID_SIZE,
            'occupancy_map': map_data_list,
            'robot_position': self.robot_pos,
            'robot_heading': self.robot_heading,
            'path_history': self.path_history,
            'sensor_data': self.sensor_data,
            'mapping_active': self.mapping_active
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(map_data, f, indent=2)
            self.log_message(f"💾 Map saved: {filename}")
            messagebox.showinfo("Save Map", f"Map saved successfully as {filename}")
        except Exception as e:
            self.log_message(f"✗ Save error: {e}")
            messagebox.showerror("Save Error", f"Failed to save map:\n{str(e)}")
            
    def on_key_press(self, event):
        """Handle keyboard input for manual control"""
        if not self.connected:
            return
            
        key = event.keysym.lower()
        command_map = {
            'w': 'w', 'up': 'w',
            's': 's', 'down': 's', 
            'a': 'a', 'left': 'a',
            'd': 'd', 'right': 'd',
            'space': 'x'
        }
        
        if key in command_map:
            self.send_command(command_map[key])
        elif key == 'm':
            if self.mapping_active:
                self.stop_autonomous_mapping()
            else:
                self.start_autonomous_mapping()
                
    def on_closing(self):
        """Handle window closing"""
        self.log_message("🔄 Shutting down...")
        self.running = False
        
        if self.connected:
            self.disconnect()
            
        if MATPLOTLIB_AVAILABLE and hasattr(self, 'animation'):
            self.animation.event_source.stop()
            plt.close('all')
            
        self.root.destroy()
        
    def run(self):
        """Start the GUI application"""
        print("=" * 50)
        print("  Autonomous Robot Mapping Controller")
        print("  Robot Contest 2025 [Robot Team]")
        print("=" * 50)
        print("Features:")
        print("✓ ESP32 ESP-NOW Communication")
        print("✓ Real-time Sensor Monitoring")
        print("✓ Autonomous Mapping")
        print("✓ Manual Robot Control")
        if MATPLOTLIB_AVAILABLE:
            print("✓ Live Map Visualization")
        else:
            print("⚠ Map Visualization Disabled (install matplotlib)")
        print("=" * 50)
        
        self.log_message("🚀 Robot Mapping Controller Started")
        self.log_message("📋 Instructions:")
        self.log_message("   1. Select COM port and click Connect")
        self.log_message("   2. Use manual controls or start auto mapping")
        self.log_message("   3. Press 'Show Map' to view real-time mapping")
        
        self.root.mainloop()

def check_dependencies():
    """Check if required packages are installed"""
    missing = []
    
    try:
        import serial
    except ImportError:
        missing.append("pyserial")
        
    if not MATPLOTLIB_AVAILABLE:
        missing.append("matplotlib")
        missing.append("numpy")
    
    if missing:
        print("Missing packages:")
        for pkg in missing:
            print(f"  - {pkg}")
        print("\nInstall with: pip install " + " ".join(missing))
        
        response = input("\nContinue without visualization? (y/n): ")
        if response.lower() != 'y':
            return False
    
    return True

if __name__ == "__main__":
    if not check_dependencies():
        sys.exit(1)
        
    try:
        controller = AutoMappingController()
        controller.run()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        input("Press Enter to exit...")
>>>>>>> Stashed changes
