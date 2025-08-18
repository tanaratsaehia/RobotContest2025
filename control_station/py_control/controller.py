#!/usr/bin/env python3
"""
Real-time Robot Controller with Servo Control - Fixed Stop Command
Enhanced version with reliable X key stop command and servo control
"""

import serial
import sys
import threading
import re
import time
import tkinter as tk
from tkinter import ttk

# Configuration
BAUD_RATE = 115200

class RealTimeRobotController:
    def __init__(self):
        self.ser = None
        self.connected = False
        self.robot_pos = [50.0, 50.0]
        self.robot_heading = 0.0
        self.sensor_data = [0.0, 0.0, 0.0, 0.0]
        self.current_command = "stop"
        self.current_speed = "normal"
        self.servo_angle = 90  # Default servo position
        
        # Enhanced patterns for real-time system with servo support
        # SENSORS:10.5,25.3,15.2,30.1|CMD:forward|SPEED:120|SERVO:90
        self.realtime_pattern = re.compile(r"SENSORS:([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+)\|CMD:(\w+)\|SPEED:(\d+)(?:\|SERVO:(\d+))?")
        
        # Legacy patterns for backward compatibility
        self.robot_pattern = re.compile(r"\[ROBOT\]\s*POS:([\d.-]+),([\d.-]+),([\d.-]+)\|S:([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+)")
        self.pos_pattern = re.compile(r"POS:([\d.-]+),([\d.-]+),([\d.-]+)\|S:([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+)")
        self.sensor_pattern = re.compile(r"Sensor\s*(\d+)\s*.*?:\s*([\d.-]+)\s*cm", re.I)
        self.servo_pattern = re.compile(r"SERVO:\s*(\d+)", re.I)
        
        self.setup_gui()
        self.running = True
        
    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("🤖 Real-time Robot Controller with Servo")
        self.root.geometry("520x900")  # เพิ่มความสูงสำหรับ servo controls
        self.root.configure(bg='#f0f0f0')
        
        # Connection Frame
        conn_frame = ttk.LabelFrame(self.root, text="🔌 Connection", padding="10")
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        port_frame = ttk.Frame(conn_frame)
        port_frame.pack(fill="x")
        
        ttk.Label(port_frame, text="Port:").pack(side="left")
        self.port_var = tk.StringVar(value="COM6")
        ttk.Entry(port_frame, textvariable=self.port_var, width=10).pack(side="left", padx=(5,10))
        
        self.connect_btn = ttk.Button(port_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side="left")
        
        self.status_label = ttk.Label(conn_frame, text="❌ Disconnected", foreground="red", font=("Arial", 10, "bold"))
        self.status_label.pack(pady=(5,0))
        
        # Real-time Movement Controls
        control_frame = ttk.LabelFrame(self.root, text="🎮 Movement Control", padding="10")
        control_frame.pack(fill="x", padx=10, pady=5)
        
        # Movement buttons - เปลี่ยน Stop กลับเป็น X
        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(pady=5)
        
        # Top row
        ttk.Button(btn_frame, text="⬆️ Forward (W)", command=lambda: self.send_cmd("w"), width=15).grid(row=0, column=1, padx=2, pady=2)
        
        # Middle row - เปลี่ยนจาก SPACE กลับเป็น X  
        ttk.Button(btn_frame, text="↩️ Left (A)", command=lambda: self.send_cmd("a"), width=15).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(btn_frame, text="⏹️ Stop (X)", command=lambda: self.send_cmd("x"), width=15).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(btn_frame, text="↪️ Right (D)", command=lambda: self.send_cmd("d"), width=15).grid(row=1, column=2, padx=2, pady=2)
        
        # Bottom row
        ttk.Button(btn_frame, text="⬇️ Back (S)", command=lambda: self.send_cmd("s"), width=15).grid(row=2, column=1, padx=2, pady=2)
        
        # Strafe buttons (if mecanum wheels)
        strafe_frame = ttk.Frame(control_frame)
        strafe_frame.pack(pady=5)
        
        ttk.Button(strafe_frame, text="⬅️ Strafe L (Q)", command=lambda: self.send_cmd("q"), width=15).pack(side="left", padx=2)
        ttk.Button(strafe_frame, text="➡️ Strafe R (E)", command=lambda: self.send_cmd("e"), width=15).pack(side="left", padx=2)
        
        # Servo Control Frame - ปรับคำสั่ง servo เนื่องจาก X ถูกใช้เป็น Stop แล้ว
        servo_frame = ttk.LabelFrame(self.root, text="🦾 Servo Control (Drop Mechanism)", padding="10")
        servo_frame.pack(fill="x", padx=10, pady=5)
        
        # Current servo position display
        self.servo_status_label = ttk.Label(servo_frame, text="Current Position: 90°", font=("Arial", 11, "bold"), foreground="navy")
        self.servo_status_label.pack(pady=(0,10))
        
        # Direction control buttons
        direction_frame = ttk.Frame(servo_frame)
        direction_frame.pack(pady=5)
        
        ttk.Button(direction_frame, text="⬅️ Turn Left (Z)", command=lambda: self.send_servo_cmd("left"), width=15).pack(side="left", padx=5)
        ttk.Button(direction_frame, text="🎯 Center (C)", command=lambda: self.send_servo_cmd("center"), width=15).pack(side="left", padx=5)
        ttk.Button(direction_frame, text="➡️ Turn Right (V)", command=lambda: self.send_servo_cmd("right"), width=15).pack(side="left", padx=5)
        
        # Drop control buttons - เปลี่ยนจาก X เป็น B สำหรับ Drop
        drop_frame = ttk.Frame(servo_frame)
        drop_frame.pack(pady=10)
        
        ttk.Button(drop_frame, text="📦 Drop Object (B)", command=lambda: self.send_servo_cmd("drop"), width=20).pack(side="left", padx=5)
        ttk.Button(drop_frame, text="🔄 Reset Position (R)", command=lambda: self.send_servo_cmd("reset"), width=20).pack(side="left", padx=5)
        
        # Fine control with slider
        slider_frame = ttk.Frame(servo_frame)
        slider_frame.pack(fill="x", pady=10)
        
        ttk.Label(slider_frame, text="Fine Control:").pack(anchor="w")
        
        self.servo_var = tk.IntVar(value=90)
        self.servo_scale = tk.Scale(slider_frame, from_=0, to=180, orient="horizontal", 
                                   variable=self.servo_var, command=self.on_servo_scale_change,
                                   length=300, resolution=10)
        self.servo_scale.pack(fill="x", pady=5)
        
        # Speed Control
        speed_frame = ttk.LabelFrame(self.root, text="⚡ Speed Control", padding="10")
        speed_frame.pack(fill="x", padx=10, pady=5)
        
        speed_btn_frame = ttk.Frame(speed_frame)
        speed_btn_frame.pack()
        
        ttk.Button(speed_btn_frame, text="🐌 Slow (1)", command=lambda: self.send_cmd("1"), width=12).pack(side="left", padx=2)
        ttk.Button(speed_btn_frame, text="🚶 Normal (2)", command=lambda: self.send_cmd("2"), width=12).pack(side="left", padx=2)
        ttk.Button(speed_btn_frame, text="🏃 Fast (3)", command=lambda: self.send_cmd("3"), width=12).pack(side="left", padx=2)
        
        self.speed_label = ttk.Label(speed_frame, text="Current Speed: Normal", font=("Arial", 10, "bold"))
        self.speed_label.pack(pady=(5,0))
        
        # System Controls
        sys_frame = ttk.LabelFrame(self.root, text="🔧 System", padding="10")
        sys_frame.pack(fill="x", padx=10, pady=5)
        
        sys_btn_frame = ttk.Frame(sys_frame)
        sys_btn_frame.pack()
        
        ttk.Button(sys_btn_frame, text="🔄 Test", command=lambda: self.send_cmd("test")).pack(side="left", padx=2)
        ttk.Button(sys_btn_frame, text="📊 Info", command=lambda: self.send_cmd("info")).pack(side="left", padx=2)
        ttk.Button(sys_btn_frame, text="❓ Help", command=lambda: self.send_cmd("help")).pack(side="left", padx=2)
        
        # Robot Status
        status_frame = ttk.LabelFrame(self.root, text="🤖 Robot Status", padding="10")
        status_frame.pack(fill="x", padx=10, pady=5)
        
        # Command and Speed Status
        cmd_frame = ttk.Frame(status_frame)
        cmd_frame.pack(fill="x")
        
        self.cmd_label = ttk.Label(cmd_frame, text="Command: stop", font=("Arial", 11, "bold"), foreground="blue")
        self.cmd_label.pack(anchor="w")
        
        self.robot_speed_label = ttk.Label(cmd_frame, text="Robot Speed: 140", font=("Arial", 11, "bold"), foreground="green")
        self.robot_speed_label.pack(anchor="w")
        
        # Sensor Display
        sensor_frame = ttk.LabelFrame(self.root, text="📡 Sensors (cm)", padding="10")
        sensor_frame.pack(fill="x", padx=10, pady=5)
        
        self.sensor_labels = []
        sensor_names = ["🔼 Front", "➡️ Right", "🔽 Back", "⬅️ Left"]
        
        # Create sensor display in 2x2 grid
        sensor_grid = ttk.Frame(sensor_frame)
        sensor_grid.pack()
        
        for i, name in enumerate(sensor_names):
            row = i // 2
            col = i % 2
            label = ttk.Label(sensor_grid, text=f"{name}: -- cm", font=("Arial", 11, "bold"))
            label.grid(row=row, column=col, padx=10, pady=5, sticky="w")
            self.sensor_labels.append(label)
        
        # Console
        console_frame = ttk.LabelFrame(self.root, text="📝 Console", padding="10")
        console_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Console with scrollbar
        console_container = ttk.Frame(console_frame)
        console_container.pack(fill="both", expand=True)
        
        self.console = tk.Text(console_container, height=10, font=("Consolas", 9), wrap="word")
        scroll = ttk.Scrollbar(console_container, command=self.console.yview)
        self.console.config(yscrollcommand=scroll.set)
        
        self.console.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        
        # Updated keyboard bindings - เปลี่ยนกลับไปใช้ X สำหรับ Stop
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.focus_set()
        
    def on_key_press(self, event):
        """Handle keyboard input for real-time control - Updated with X for stop"""
        key = event.char.lower()
        
        # Movement controls - X กลับมาเป็น Stop แล้ว
        movement_mappings = {
            'w': 'w', 's': 's', 'a': 'a', 'd': 'd',
            'q': 'q', 'e': 'e', 'x': 'x',  # X สำหรับ Stop
            '1': '1', '2': '2', '3': '3'
        }
        
        # Servo controls - เปลี่ยน Drop จาก X เป็น B
        servo_mappings = {
            'z': 'left',    # Turn servo left
            'c': 'center',  # Center servo
            'v': 'right',   # Turn servo right
            'b': 'drop',    # Drop object (เปลี่ยนจาก X เป็น B)
            'r': 'reset'    # Reset servo position
        }
        
        if key in movement_mappings:
            self.send_cmd(movement_mappings[key])
        elif key in servo_mappings:
            self.send_servo_cmd(servo_mappings[key])
        
    def on_servo_scale_change(self, value):
        """Handle servo slider changes"""
        angle = int(value)
        self.send_servo_angle(angle)
        
    def send_servo_cmd(self, servo_command):
        """Send servo command with predefined positions"""
        servo_commands = {
            'left': 45,      # Turn left position
            'center': 90,    # Center position  
            'right': 135,    # Turn right position
            'drop': 180,     # Full drop position
            'reset': 90      # Reset to center
        }
        
        if servo_command in servo_commands:
            angle = servo_commands[servo_command]
            self.send_servo_angle(angle)
            
            # Update display names for better user feedback
            display_names = {
                'left': '⬅️ Turn Left',
                'center': '🎯 Center',
                'right': '➡️ Turn Right', 
                'drop': '📦 Drop Object',
                'reset': '🔄 Reset Position'
            }
            
            self.log(f"Servo Command: {display_names.get(servo_command, servo_command)} ({angle}°)", "SERVO")
        
    def send_servo_angle(self, angle):
        """Send specific servo angle command"""
        if self.connected and self.ser:
            try:
                # Send servo command in format: servo:angle
                command = f"servo:{angle}"
                self.ser.write((command + '\n').encode())
                
                # Update local servo position
                self.servo_angle = angle
                self.servo_var.set(angle)
                self.servo_status_label.config(text=f"Current Position: {angle}°")
                
                self.log(f"Servo moved to {angle}°", "SERVO")
                
            except Exception as e:
                self.log(f"Servo command error: {e}", "ERROR")
        else:
            self.log("Cannot send servo command - not connected!", "ERROR")
        
    def log(self, msg, tag="INFO"):
        """Log message to console with colors"""
        timestamp = time.strftime("%H:%M:%S")
        full_msg = f"[{timestamp}] [{tag}] {msg}\n"
        
        self.console.insert(tk.END, full_msg)
        self.console.see(tk.END)
        
        # Enhanced color coding including servo
        colors = {
            "ERROR": "red",
            "SENSOR": "blue", 
            "POSITION": "green",
            "COMMAND": "purple",
            "SPEED": "orange",
            "STATUS": "navy",
            "SUCCESS": "darkgreen",
            "SERVO": "darkred"  # New color for servo commands
        }
        
        color = colors.get(tag, "black")
        
        # Apply color to last line
        try:
            last_line = self.console.index("end-2l")
            self.console.tag_add(tag, last_line, "end-1c")
            self.console.tag_config(tag, foreground=color)
        except:
            pass
            
        print(f"[{tag}] {msg}")
        
    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()
            
    def connect(self):
        try:
            port = self.port_var.get()
            self.ser = serial.Serial(port, BAUD_RATE, timeout=1)
            time.sleep(2)
            
            self.connected = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="✅ Connected", foreground="green")
            
            # Start reading thread
            self.data_thread = threading.Thread(target=self.read_data, daemon=True)
            self.data_thread.start()
            
            self.log(f"Connected to {port}", "SUCCESS")
            self.log("Movement: WASD+X for stop, Speed: 123, Servo: ZCVBR", "INFO")
            
        except Exception as e:
            self.log(f"Connection failed: {e}", "ERROR")
            
    def disconnect(self):
        try:
            if self.ser:
                self.ser.close()
            self.connected = False
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="❌ Disconnected", foreground="red")
            self.log("Disconnected", "INFO")
        except Exception as e:
            self.log(f"Disconnect error: {e}", "ERROR")
            
    def send_cmd(self, command):
        if self.connected and self.ser:
            try:
                self.ser.write((command + '\n').encode())
                
                # Map commands for display - อัปเดตเพื่อแสดง X เป็น Stop
                cmd_names = {
                    'w': 'Forward', 's': 'Backward', 'a': 'Turn Left', 'd': 'Turn Right',
                    'q': 'Strafe Left', 'e': 'Strafe Right', 'x': 'Stop',  # X เป็น Stop แล้ว
                    '1': 'Speed Slow', '2': 'Speed Normal', '3': 'Speed Fast'
                }
                
                display_name = cmd_names.get(command, command)
                self.log(f"Sent: {display_name}", "COMMAND")
                
            except Exception as e:
                self.log(f"Send error: {e}", "ERROR")
        else:
            self.log("Not connected!", "ERROR")
            
    def read_data(self):
        """Read data from serial port"""
        self.log("Started real-time data reading with servo support", "INFO")
        
        while self.running and self.connected and self.ser:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.process_data(line)
                time.sleep(0.01)
            except Exception as e:
                self.log(f"Read error: {e}", "ERROR")
                break
                
    def process_data(self, line):
        """Process incoming data with real-time format support including servo"""
        
        # Try enhanced real-time format: SENSORS:10.5,25.3,15.2,30.1|CMD:forward|SPEED:120|SERVO:90
        realtime_match = self.realtime_pattern.search(line)
        if realtime_match:
            try:
                sensors = [
                    float(realtime_match.group(1)),
                    float(realtime_match.group(2)), 
                    float(realtime_match.group(3)),
                    float(realtime_match.group(4))
                ]
                command = realtime_match.group(5)
                speed = int(realtime_match.group(6))
                servo_angle = int(realtime_match.group(7)) if realtime_match.group(7) else self.servo_angle
                
                # Update data
                self.sensor_data = sensors
                self.current_command = command
                self.servo_angle = servo_angle
                
                # Update GUI
                self.root.after(0, lambda: self.update_realtime_gui(command, speed, servo_angle))
                
                return
                
            except ValueError as e:
                self.log(f"Parse error (realtime): {e}", "ERROR")
        
        # Handle servo status messages
        servo_match = self.servo_pattern.search(line)
        if servo_match:
            try:
                servo_angle = int(servo_match.group(1))
                self.servo_angle = servo_angle
                self.root.after(0, lambda: self.update_servo_gui(servo_angle))
                self.log(f"Servo position updated: {servo_angle}°", "SERVO")
                return
            except ValueError as e:
                self.log(f"Servo parse error: {e}", "ERROR")
        
        # Handle individual sensor readings: Sensor 1: 25.3 cm
        sensor_match = self.sensor_pattern.search(line)
        if sensor_match:
            try:
                sensor_id = int(sensor_match.group(1)) - 1
                distance = float(sensor_match.group(2))
                
                if 0 <= sensor_id < 4:
                    old_val = self.sensor_data[sensor_id]
                    self.sensor_data[sensor_id] = distance
                    
                    self.root.after(0, self.update_sensors_gui)
                    
                    # Log significant changes
                    if abs(distance - old_val) > 10:
                        sensor_names = ["Front", "Right", "Back", "Left"]
                        self.log(f"{sensor_names[sensor_id]}: {distance:.1f} cm", "SENSOR")
                        
            except (ValueError, IndexError) as e:
                self.log(f"Sensor parse error: {e}", "ERROR")
        
        # Log important system messages including servo messages
        important_keywords = ['[✓]', '[✗]', '[ERROR]', '[READY]', '[STATUS]', '[SPEED]', '[CMD]', '[SERVO]', 'Speed set to', 'Servo']
        if any(keyword in line for keyword in important_keywords):
            if 'Speed set to' in line:
                self.log(line, "SPEED")
            elif '[SERVO]' in line or 'Servo' in line:
                self.log(line, "SERVO")
            elif '[✓]' in line or '[READY]' in line:
                self.log(line, "SUCCESS")  
            elif '[✗]' in line or '[ERROR]' in line:
                self.log(line, "ERROR")
            else:
                self.log(line, "STATUS")
                
    def update_realtime_gui(self, command, speed, servo_angle=None):
        """Update GUI with real-time data including servo position"""
        try:
            # Update command display
            self.cmd_label.config(text=f"Command: {command}")
            
            # Update speed display  
            self.robot_speed_label.config(text=f"Robot Speed: {speed}")
            
            # Update speed label based on speed value
            if speed <= 110:
                speed_text = "🐌 Slow"
                self.current_speed = "slow"
            elif speed <= 150:
                speed_text = "🚶 Normal"  
                self.current_speed = "normal"
            else:
                speed_text = "🏃 Fast"
                self.current_speed = "fast"
                
            self.speed_label.config(text=f"Current Speed: {speed_text}")
            
            # Update servo if provided
            if servo_angle is not None:
                self.update_servo_gui(servo_angle)
            
            # Update sensors
            self.update_sensors_gui()
            
        except Exception as e:
            self.log(f"GUI update error: {e}", "ERROR")
    
    def update_servo_gui(self, angle):
        """Update servo display"""
        try:
            self.servo_angle = angle
            self.servo_var.set(angle)
            self.servo_status_label.config(text=f"Current Position: {angle}°")
        except Exception as e:
            self.log(f"Servo GUI update error: {e}", "ERROR")
            
    def update_sensors_gui(self):
        """Update sensor display"""
        try:
            sensor_names = ["🔼 Front", "➡️ Right", "🔽 Back", "⬅️ Left"]
            
            for i, (name, distance) in enumerate(zip(sensor_names, self.sensor_data)):
                if distance < 999 and distance > 0:
                    # Color and icon based on distance
                    if distance < 15:
                        color = "red"
                        icon = "🔴"
                        status = "DANGER"
                    elif distance < 30:
                        color = "orange" 
                        icon = "🟡"
                        status = "CAUTION"
                    elif distance < 60:
                        color = "blue"
                        icon = "🟢"
                        status = "SAFE"
                    else:
                        color = "green"
                        icon = "🟢"
                        status = "CLEAR"
                    
                    self.sensor_labels[i].config(
                        text=f"{name}: {distance:.1f} cm {icon}",
                        foreground=color
                    )
                else:
                    self.sensor_labels[i].config(
                        text=f"{name}: -- cm ⚫",
                        foreground="gray"
                    )
                    
        except Exception as e:
            self.log(f"Sensor GUI update error: {e}", "ERROR")
            
    def run(self):
        """Start the application"""
        self.log("🚀 Real-time Robot Controller with Servo Started", "SUCCESS")
        self.log("🎮 Controls: WASD=Move, X=Stop, 123=Speed, ZCVBR=Servo", "INFO")
        self.log("📦 Servo: Z=Left, C=Center, V=Right, B=Drop, R=Reset", "INFO")
        
        try:
            self.root.mainloop()
        finally:
            self.running = False
            if self.connected:
                self.disconnect()

if __name__ == "__main__":
    try:
        controller = RealTimeRobotController()
        controller.run()
    except Exception as e:
        print(f"Error: {e}")
        input("Press Enter to exit...")