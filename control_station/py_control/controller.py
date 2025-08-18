#!/usr/bin/env python3
"""
Simple Robot Controller - Fixed for [ROBOT] prefix
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

class SimpleRobotController:
    def __init__(self):
        self.ser = None
        self.connected = False
        self.robot_pos = [50.0, 50.0]
        self.robot_heading = 0.0
        self.sensor_data = [0.0, 0.0, 0.0, 0.0]
        
        # Enhanced patterns to handle [ROBOT] prefix
        self.robot_pattern = re.compile(r"\[ROBOT\]\s*POS:([\d.-]+),([\d.-]+),([\d.-]+)\|S:([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+)")
        self.pos_pattern = re.compile(r"POS:([\d.-]+),([\d.-]+),([\d.-]+)\|S:([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+)")
        self.sensor_pattern = re.compile(r"Sensor\s*(\d+)\s*.*?:\s*([\d.-]+)\s*cm", re.I)
        
        self.setup_gui()
        self.running = True
        
    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Simple Robot Controller")
        self.root.geometry("400x600")
        
        # Connection
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding="10")
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        self.port_var = tk.StringVar(value="COM3")
        ttk.Entry(conn_frame, textvariable=self.port_var, width=10).pack()
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(pady=5)
        
        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.pack()
        
        # Controls
        control_frame = ttk.LabelFrame(self.root, text="Manual Control", padding="10")
        control_frame.pack(fill="x", padx=10, pady=5)
        
        # Direction buttons
        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack()
        
        ttk.Button(btn_frame, text="↑", command=lambda: self.send_cmd("forward"), width=3).grid(row=0, column=1)
        ttk.Button(btn_frame, text="←", command=lambda: self.send_cmd("turn_left"), width=3).grid(row=1, column=0)
        ttk.Button(btn_frame, text="■", command=lambda: self.send_cmd("stop"), width=3).grid(row=1, column=1)
        ttk.Button(btn_frame, text="→", command=lambda: self.send_cmd("turn_right"), width=3).grid(row=1, column=2)
        ttk.Button(btn_frame, text="↓", command=lambda: self.send_cmd("backward"), width=3).grid(row=2, column=1)
        
        # Test button
        ttk.Button(control_frame, text="Test Robot", command=lambda: self.send_cmd("test")).pack(pady=5)
        
        # Status display
        status_frame = ttk.LabelFrame(self.root, text="Robot Status", padding="10")
        status_frame.pack(fill="x", padx=10, pady=5)
        
        self.pos_label = ttk.Label(status_frame, text="Position: (50.0, 50.0)", font=("Arial", 11, "bold"))
        self.pos_label.pack(anchor="w")
        
        self.heading_label = ttk.Label(status_frame, text="Heading: 0.0°", font=("Arial", 11, "bold"))
        self.heading_label.pack(anchor="w")
        
        # Sensors
        sensor_frame = ttk.LabelFrame(self.root, text="Sensors", padding="10")
        sensor_frame.pack(fill="x", padx=10, pady=5)
        
        self.sensor_labels = []
        sensor_names = ["Front", "Right", "Back", "Left"]
        for i, name in enumerate(sensor_names):
            label = ttk.Label(sensor_frame, text=f"{name}: -- cm", font=("Arial", 10))
            label.pack(anchor="w")
            self.sensor_labels.append(label)
        
        # Console
        console_frame = ttk.LabelFrame(self.root, text="Console", padding="10")
        console_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.console = tk.Text(console_frame, height=12, font=("Consolas", 9))
        scroll = ttk.Scrollbar(console_frame, command=self.console.yview)
        self.console.config(yscrollcommand=scroll.set)
        
        self.console.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        
    def log(self, msg, tag="INFO"):
        """Log message to console"""
        timestamp = time.strftime("%H:%M:%S")
        full_msg = f"[{timestamp}] [{tag}] {msg}\n"
        
        self.console.insert(tk.END, full_msg)
        self.console.see(tk.END)
        
        # Color coding
        if tag == "ERROR":
            color = "red"
        elif tag == "SENSOR":
            color = "blue"
        elif tag == "POSITION":
            color = "green"
        elif tag == "COMMAND":
            color = "purple"
        else:
            color = "black"
            
        # Apply color to last line
        last_line = self.console.index("end-2l")
        self.console.tag_add(tag, last_line, "end-1c")
        self.console.tag_config(tag, foreground=color)
        
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
            self.status_label.config(text="Connected", foreground="green")
            
            # Start reading thread
            self.data_thread = threading.Thread(target=self.read_data, daemon=True)
            self.data_thread.start()
            
            self.log(f"Connected to {port}", "INFO")
            
        except Exception as e:
            self.log(f"Connection failed: {e}", "ERROR")
            
    def disconnect(self):
        try:
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
                self.log(f"Sent: {command}", "COMMAND")
            except Exception as e:
                self.log(f"Send error: {e}", "ERROR")
        else:
            self.log("Not connected!", "ERROR")
            
    def read_data(self):
        """Read data from serial port"""
        self.log("Started data reading", "INFO")
        
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
        """Process incoming data with enhanced parsing"""
        
        # Log raw data
        if line.strip():
            self.log(f"Raw: {line}", "DEBUG")
        
        # Try to parse [ROBOT] POS format first
        robot_match = self.robot_pattern.search(line)
        if robot_match:
            try:
                x = float(robot_match.group(1))
                y = float(robot_match.group(2))
                heading = float(robot_match.group(3))
                sensors = [
                    float(robot_match.group(4)),
                    float(robot_match.group(5)),
                    float(robot_match.group(6)),
                    float(robot_match.group(7))
                ]
                
                # Update data
                self.robot_pos = [x, y]
                self.robot_heading = heading
                self.sensor_data = sensors
                
                # Update GUI
                self.root.after(0, self.update_gui)
                
                self.log(f"Position: ({x:.1f}, {y:.1f}) @ {heading:.1f}°", "POSITION")
                self.log(f"Sensors: {[f'{s:.1f}' for s in sensors]}", "SENSOR")
                
                return
                
            except ValueError as e:
                self.log(f"Parse error (ROBOT): {e}", "ERROR")
        
        # Try regular POS format
        pos_match = self.pos_pattern.search(line)
        if pos_match:
            try:
                x = float(pos_match.group(1))
                y = float(pos_match.group(2))
                heading = float(pos_match.group(3))
                sensors = [
                    float(pos_match.group(4)),
                    float(pos_match.group(5)),
                    float(pos_match.group(6)),
                    float(pos_match.group(7))
                ]
                
                self.robot_pos = [x, y]
                self.robot_heading = heading
                self.sensor_data = sensors
                
                self.root.after(0, self.update_gui)
                self.log(f"Position: ({x:.1f}, {y:.1f}) @ {heading:.1f}°", "POSITION")
                
                return
                
            except ValueError as e:
                self.log(f"Parse error (POS): {e}", "ERROR")
        
        # Try individual sensor format
        sensor_match = self.sensor_pattern.search(line)
        if sensor_match:
            try:
                sensor_id = int(sensor_match.group(1)) - 1
                distance = float(sensor_match.group(2))
                
                if 0 <= sensor_id < 4:
                    old_val = self.sensor_data[sensor_id]
                    self.sensor_data[sensor_id] = distance
                    
                    self.root.after(0, self.update_gui)
                    
                    sensor_names = ["Front", "Right", "Back", "Left"]
                    if abs(distance - old_val) > 5:
                        self.log(f"{sensor_names[sensor_id]}: {distance:.1f} cm", "SENSOR")
                        
            except (ValueError, IndexError) as e:
                self.log(f"Sensor parse error: {e}", "ERROR")
        
        # Log other messages
        if any(kw in line for kw in ['[SETUP]', '[MOTOR]', '[COMMAND]', '[STATUS]']):
            self.log(line, "INFO")
            
    def update_gui(self):
        """Update GUI with current data"""
        try:
            # Update position
            self.pos_label.config(text=f"Position: ({self.robot_pos[0]:.1f}, {self.robot_pos[1]:.1f})")
            self.heading_label.config(text=f"Heading: {self.robot_heading:.1f}°")
            
            # Update sensors
            sensor_names = ["Front", "Right", "Back", "Left"]
            for i, (name, distance) in enumerate(zip(sensor_names, self.sensor_data)):
                if distance < 999 and distance > 0:
                    # Color based on distance
                    if distance < 15:
                        color = "red"
                        icon = "🔴"
                    elif distance < 50:
                        color = "orange"
                        icon = "🟡"
                    else:
                        color = "green"
                        icon = "🟢"
                    
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
            self.log(f"GUI update error: {e}", "ERROR")
            
    def run(self):
        """Start the application"""
        self.log("Robot Controller Started", "INFO")
        self.log("Click Connect and select your COM port", "INFO")
        self.root.mainloop()

if __name__ == "__main__":
    try:
        controller = SimpleRobotController()
        controller.run()
    except Exception as e:
        print(f"Error: {e}")
        input("Press Enter to exit...")