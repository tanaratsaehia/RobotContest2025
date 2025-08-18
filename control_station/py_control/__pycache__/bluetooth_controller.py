#!/usr/bin/env python3
"""
Python Bluetooth Robot Controller
Simple GUI controller for ESP32 robot via Bluetooth Classic
"""

import bluetooth
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

class BluetoothRobotController:
    def __init__(self):
        self.sock = None
        self.connected = False
        self.current_command = "stop"
        self.current_speed = "normal"
        self.servo_angle = 90
        
        self.setup_gui()
        self.running = True
        
    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("🤖 Bluetooth Robot Controller")
        self.root.geometry("500x700")
        self.root.configure(bg='#f0f0f0')
        
        # Connection Frame
        conn_frame = ttk.LabelFrame(self.root, text="📡 Bluetooth Connection", padding="10")
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        # Device selection
        device_frame = ttk.Frame(conn_frame)
        device_frame.pack(fill="x", pady=5)
        
        ttk.Label(device_frame, text="Device:").pack(side="left")
        self.device_var = tk.StringVar(value="Robot_Controller")
        device_combo = ttk.Combobox(device_frame, textvariable=self.device_var, width=20)
        device_combo['values'] = ("Robot_Controller", "Robot_Test", "Custom...")
        device_combo.pack(side="left", padx=(5,10))
        
        self.scan_btn = ttk.Button(device_frame, text="Scan", command=self.scan_devices)
        self.scan_btn.pack(side="left", padx=2)
        
        self.connect_btn = ttk.Button(device_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side="left", padx=2)
        
        self.status_label = ttk.Label(conn_frame, text="❌ Disconnected", foreground="red", font=("Arial", 10, "bold"))
        self.status_label.pack(pady=(5,0))
        
        # Movement Control
        control_frame = ttk.LabelFrame(self.root, text="🎮 Movement Control", padding="10")
        control_frame.pack(fill="x", padx=10, pady=5)
        
        # Movement buttons
        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(pady=5)
        
        # Top row
        ttk.Button(btn_frame, text="⬆️ Forward (W)", command=lambda: self.send_command("F"), width=15).grid(row=0, column=1, padx=2, pady=2)
        
        # Middle row
        ttk.Button(btn_frame, text="⬅️ Left (A)", command=lambda: self.send_command("L"), width=15).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(btn_frame, text="⏹️ Stop (X)", command=lambda: self.send_command("STOP"), width=15).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(btn_frame, text="➡️ Right (D)", command=lambda: self.send_command("R"), width=15).grid(row=1, column=2, padx=2, pady=2)
        
        # Bottom row
        ttk.Button(btn_frame, text="⬇️ Backward (S)", command=lambda: self.send_command("B"), width=15).grid(row=2, column=1, padx=2, pady=2)
        
        # Speed Control
        speed_frame = ttk.LabelFrame(self.root, text="⚡ Speed Control", padding="10")
        speed_frame.pack(fill="x", padx=10, pady=5)
        
        speed_btn_frame = ttk.Frame(speed_frame)
        speed_btn_frame.pack()
        
        ttk.Button(speed_btn_frame, text="🐌 Slow (1)", command=lambda: self.send_command("1"), width=12).pack(side="left", padx=2)
        ttk.Button(speed_btn_frame, text="🚶 Normal (2)", command=lambda: self.send_command("2"), width=12).pack(side="left", padx=2)
        ttk.Button(speed_btn_frame, text="🏃 Fast (3)", command=lambda: self.send_command("3"), width=12).pack(side="left", padx=2)
        
        self.speed_label = ttk.Label(speed_frame, text="Current Speed: Normal", font=("Arial", 10, "bold"))
        self.speed_label.pack(pady=(5,0))
        
        # Servo Control
        servo_frame = ttk.LabelFrame(self.root, text="🦾 Drop Mechanism", padding="10")
        servo_frame.pack(fill="x", padx=10, pady=5)
        
        # Servo buttons
        servo_btn_frame = ttk.Frame(servo_frame)
        servo_btn_frame.pack(pady=5)
        
        ttk.Button(servo_btn_frame, text="📦 Drop Object", command=lambda: self.send_command("DROP"), width=15).pack(side="left", padx=5)
        ttk.Button(servo_btn_frame, text="🔄 Reset Position", command=lambda: self.send_command("RESET"), width=15).pack(side="left", padx=5)
        
        # Servo angle control
        angle_frame = ttk.Frame(servo_frame)
        angle_frame.pack(fill="x", pady=10)
        
        ttk.Label(angle_frame, text="Servo Angle:").pack(anchor="w")
        
        self.servo_var = tk.IntVar(value=90)
        self.servo_scale = tk.Scale(angle_frame, from_=0, to=180, orient="horizontal", 
                                   variable=self.servo_var, command=self.on_servo_change,
                                   length=300, resolution=10)
        self.servo_scale.pack(fill="x", pady=5)
        
        # System Commands
        system_frame = ttk.LabelFrame(self.root, text="🔧 System Commands", padding="10")
        system_frame.pack(fill="x", padx=10, pady=5)
        
        sys_btn_frame = ttk.Frame(system_frame)
        sys_btn_frame.pack()
        
        ttk.Button(sys_btn_frame, text="📊 Status", command=lambda: self.send_command("STATUS")).pack(side="left", padx=2)
        ttk.Button(sys_btn_frame, text="🔄 Test", command=lambda: self.send_command("TEST")).pack(side="left", padx=2)
        ttk.Button(sys_btn_frame, text="❓ Help", command=lambda: self.send_command("HELP")).pack(side="left", padx=2)
        
        # Response Display
        response_frame = ttk.LabelFrame(self.root, text="📝 Robot Response", padding="10")
        response_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Response text with scrollbar
        response_container = ttk.Frame(response_frame)
        response_container.pack(fill="both", expand=True)
        
        self.response_text = tk.Text(response_container, height=8, font=("Consolas", 9), wrap="word")
        scroll = ttk.Scrollbar(response_container, command=self.response_text.yview)
        self.response_text.config(yscrollcommand=scroll.set)
        
        self.response_text.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        
        # Keyboard bindings
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.focus_set()
        
    def on_key_press(self, event):
        """Handle keyboard input for real-time control"""
        key = event.char.lower()
        
        key_mappings = {
            'w': 'F', 's': 'B', 'a': 'L', 'd': 'R', 'x': 'STOP',
            '1': '1', '2': '2', '3': '3'
        }
        
        if key in key_mappings:
            self.send_command(key_mappings[key])
    
    def on_servo_change(self, value):
        """Handle servo slider changes"""
        angle = int(value)
        self.send_command(f"SERVO:{angle}")
        
    def log_response(self, message, tag="INFO"):
        """Log response message with timestamp"""
        timestamp = time.strftime("%H:%M:%S")
        full_msg = f"[{timestamp}] {message}\n"
        
        self.response_text.insert(tk.END, full_msg)
        self.response_text.see(tk.END)
        
        # Color coding
        colors = {
            "SENT": "blue",
            "RECEIVED": "green", 
            "ERROR": "red",
            "STATUS": "purple"
        }
        
        if tag in colors:
            try:
                last_line = self.response_text.index("end-2l")
                self.response_text.tag_add(tag, last_line, "end-1c")
                self.response_text.tag_config(tag, foreground=colors[tag])
            except:
                pass
        
        print(f"[{tag}] {message}")
    
    def scan_devices(self):
        """Scan for nearby Bluetooth devices"""
        self.log_response("Scanning for Bluetooth devices...", "STATUS")
        self.scan_btn.config(text="Scanning...", state="disabled")
        
        def scan_thread():
            try:
                devices = bluetooth.discover_devices(duration=8, lookup_names=True, flush_cache=True)
                
                device_names = []
                for addr, name in devices:
                    device_names.append(f"{name} ({addr})")
                    if "robot" in name.lower() or "esp32" in name.lower():
                        self.log_response(f"Found robot device: {name} - {addr}", "STATUS")
                
                self.root.after(0, lambda: self.update_device_list(device_names))
                
            except Exception as e:
                self.root.after(0, lambda: self.log_response(f"Scan error: {e}", "ERROR"))
            
            self.root.after(0, lambda: self.scan_btn.config(text="Scan", state="normal"))
        
        threading.Thread(target=scan_thread, daemon=True).start()
    
    def update_device_list(self, devices):
        """Update device combobox with scan results"""
        if devices:
            self.log_response(f"Found {len(devices)} devices", "STATUS")
            # Update combobox values - you might want to implement this
        else:
            self.log_response("No devices found", "STATUS")
    
    def toggle_connection(self):
        """Toggle Bluetooth connection"""
        if self.connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        """Connect to Bluetooth device"""
        device_name = self.device_var.get()
        
        self.log_response(f"Connecting to {device_name}...", "STATUS")
        self.connect_btn.config(text="Connecting...", state="disabled")
        
        def connect_thread():
            try:
                # Find device by name
                devices = bluetooth.discover_devices(lookup_names=True)
                target_addr = None
                
                for addr, name in devices:
                    if device_name in name:
                        target_addr = addr
                        break
                
                if not target_addr:
                    raise Exception(f"Device '{device_name}' not found")
                
                # Connect to device
                self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                self.sock.connect((target_addr, 1))  # Channel 1 is standard for SPP
                
                self.connected = True
                
                # Start receive thread
                self.receive_thread = threading.Thread(target=self.receive_data, daemon=True)
                self.receive_thread.start()
                
                self.root.after(0, self.on_connected)
                
            except Exception as e:
                self.root.after(0, lambda: self.on_connect_error(str(e)))
        
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def on_connected(self):
        """Called when connection is successful"""
        self.log_response("✅ Connected successfully!", "STATUS")
        self.status_label.config(text="✅ Connected", foreground="green")
        self.connect_btn.config(text="Disconnect", state="normal")
        
        # Send initial status request
        self.send_command("STATUS")
    
    def on_connect_error(self, error):
        """Called when connection fails"""
        self.log_response(f"❌ Connection failed: {error}", "ERROR")
        self.connect_btn.config(text="Connect", state="normal")
        
    def disconnect(self):
        """Disconnect from Bluetooth device"""
        try:
            if self.sock:
                self.sock.close()
            self.connected = False
            
            self.log_response("Disconnected", "STATUS")
            self.status_label.config(text="❌ Disconnected", foreground="red")
            self.connect_btn.config(text="Connect")
            
        except Exception as e:
            self.log_response(f"Disconnect error: {e}", "ERROR")
    
    def send_command(self, command):
        """Send command to robot"""
        if not self.connected or not self.sock:
            self.log_response("❌ Not connected!", "ERROR")
            return
        
        try:
            message = command + "\n"
            self.sock.send(message.encode())
            self.log_response(f"→ {command}", "SENT")
            
            # Update local state
            if command in ["F", "FORWARD"]:
                self.current_command = "forward"
            elif command in ["B", "BACKWARD"]:
                self.current_command = "backward"
            elif command in ["L", "LEFT"]:
                self.current_command = "left"
            elif command in ["R", "RIGHT"]:
                self.current_command = "right"
            elif command in ["STOP", "X"]:
                self.current_command = "stop"
            elif command in ["1", "SLOW"]:
                self.current_speed = "slow"
                self.speed_label.config(text="Current Speed: Slow")
            elif command in ["2", "NORMAL"]:
                self.current_speed = "normal"
                self.speed_label.config(text="Current Speed: Normal")
            elif command in ["3", "FAST"]:
                self.current_speed = "fast"
                self.speed_label.config(text="Current Speed: Fast")
            elif command.startswith("SERVO:"):
                angle = int(command.split(":")[1])
                self.servo_angle = angle
                
        except Exception as e:
            self.log_response(f"Send error: {e}", "ERROR")
            self.disconnect()
    
    def receive_data(self):
        """Receive data from robot"""
        while self.running and self.connected:
            try:
                data = self.sock.recv(1024).decode().strip()
                if data:
                    self.root.after(0, lambda msg=data: self.log_response(f"← {msg}", "RECEIVED"))
                    
            except Exception as e:
                if self.connected:  # Only log if we expect to be connected
                    self.root.after(0, lambda: self.log_response(f"Receive error: {e}", "ERROR"))
                break
    
    def run(self):
        """Start the application"""
        self.log_response("🚀 Bluetooth Robot Controller Started", "STATUS")
        self.log_response("🎮 Controls: WASD=Move, X=Stop, 123=Speed", "STATUS")
        self.log_response("📡 Click 'Scan' to find devices, then 'Connect'", "STATUS")
        
        try:
            self.root.mainloop()
        finally:
            self.running = False
            if self.connected:
                self.disconnect()

if __name__ == "__main__":
    try:
        # Check if bluetooth module is available
        import bluetooth
        controller = BluetoothRobotController()
        controller.run()
    except ImportError:
        print("Error: pybluez module not found!")
        print("Please install it with: pip install pybluez")
        input("Press Enter to exit...")
    except Exception as e:
        print(f"Error: {e}")
        input("Press Enter to exit...")