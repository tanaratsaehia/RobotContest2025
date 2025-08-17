import tkinter as tk
from tkinter import ttk, messagebox
import serial
import threading
import time
from datetime import datetime
import serial.tools.list_ports

class GUIRobotController:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("🤖 Robot Controller GUI")
        self.root.geometry("600x500")
        self.root.configure(bg='#2b2b2b')
        
        self.ser = None
        self.running = False
        self.status_text = tk.StringVar(value="Disconnected")
        
        self.setup_ui()
        self.scan_ports()
        
    def setup_ui(self):
        """สร้าง UI elements"""
        # Title
        title_label = tk.Label(self.root, text="🤖 ESP32 Robot Controller", 
                              font=('Arial', 20, 'bold'), 
                              fg='white', bg='#2b2b2b')
        title_label.pack(pady=10)
        
        # Connection Frame
        conn_frame = tk.LabelFrame(self.root, text="Connection", 
                                  font=('Arial', 12, 'bold'), 
                                  fg='white', bg='#2b2b2b')
        conn_frame.pack(fill='x', padx=20, pady=10)
        
        # Port selection
        port_frame = tk.Frame(conn_frame, bg='#2b2b2b')
        port_frame.pack(fill='x', padx=5, pady=5)
        
        tk.Label(port_frame, text="COM Port:", fg='white', bg='#2b2b2b').pack(side='left')
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.port_var, width=15)
        self.port_combo.pack(side='left', padx=5)
        
        # Buttons
        tk.Button(port_frame, text="🔍 Scan", command=self.scan_ports, 
                 bg='#4CAF50', fg='white', font=('Arial', 10, 'bold')).pack(side='left', padx=5)
        
        self.connect_btn = tk.Button(port_frame, text="📡 Connect", command=self.toggle_connection,
                                   bg='#2196F3', fg='white', font=('Arial', 10, 'bold'))
        self.connect_btn.pack(side='left', padx=5)
        
        # Status
        self.status_label = tk.Label(conn_frame, textvariable=self.status_text, 
                                   fg='red', bg='#2b2b2b', font=('Arial', 10, 'bold'))
        self.status_label.pack(pady=5)
        
        # Control Frame
        control_frame = tk.LabelFrame(self.root, text="Robot Control", 
                                    font=('Arial', 12, 'bold'), 
                                    fg='white', bg='#2b2b2b')
        control_frame.pack(fill='both', expand=True, padx=20, pady=10)
        
        # Movement buttons
        self.create_movement_buttons(control_frame)
        
        # Map buttons
        self.create_map_buttons(control_frame)
        
        # Log area
        self.create_log_area()
        
    def create_movement_buttons(self, parent):
        """สร้างปุ่มควบคุมการเคลื่อนไหว"""
        move_frame = tk.Frame(parent, bg='#2b2b2b')
        move_frame.pack(pady=20)
        
        tk.Label(move_frame, text="Movement", font=('Arial', 14, 'bold'), 
                fg='white', bg='#2b2b2b').pack()
        
        # Button layout frame
        button_layout = tk.Frame(move_frame, bg='#2b2b2b')
        button_layout.pack(pady=10)
        
        # Forward button (top row)
        forward_frame = tk.Frame(button_layout, bg='#2b2b2b')
        forward_frame.pack()
        
        tk.Button(forward_frame, text="🔼\nFORWARD", command=lambda: self.send_command('w'),
                 bg='#4CAF50', fg='white', font=('Arial', 12, 'bold'), 
                 width=10, height=2).pack()
        
        # Middle row: Left, Stop, Right
        middle_frame = tk.Frame(button_layout, bg='#2b2b2b')
        middle_frame.pack(pady=5)
        
        tk.Button(middle_frame, text="⬅️\nLEFT", command=lambda: self.send_command('a'),
                 bg='#FF9800', fg='white', font=('Arial', 12, 'bold'), 
                 width=10, height=2).pack(side='left', padx=5)
        
        tk.Button(middle_frame, text="⏹️\nSTOP", command=lambda: self.send_command('x'),
                 bg='#F44336', fg='white', font=('Arial', 12, 'bold'), 
                 width=10, height=2).pack(side='left', padx=5)
        
        tk.Button(middle_frame, text="➡️\nRIGHT", command=lambda: self.send_command('d'),
                 bg='#FF9800', fg='white', font=('Arial', 12, 'bold'), 
                 width=10, height=2).pack(side='left', padx=5)
        
        # Backward button (bottom row)
        backward_frame = tk.Frame(button_layout, bg='#2b2b2b')
        backward_frame.pack(pady=5)
        
        tk.Button(backward_frame, text="🔽\nBACKWARD", command=lambda: self.send_command('s'),
                 bg='#4CAF50', fg='white', font=('Arial', 12, 'bold'), 
                 width=10, height=2).pack()
        
    def create_map_buttons(self, parent):
        """สร้างปุ่มเกี่ยวกับ mapping"""
        map_frame = tk.Frame(parent, bg='#2b2b2b')
        map_frame.pack(pady=10)
        
        tk.Label(map_frame, text="Mapping", font=('Arial', 14, 'bold'), 
                fg='white', bg='#2b2b2b').pack()
        
        btn_frame = tk.Frame(map_frame, bg='#2b2b2b')
        btn_frame.pack()
        
        tk.Button(btn_frame, text="🗺️ Show Map", command=lambda: self.send_command('m'),
                 bg='#9C27B0', fg='white', font=('Arial', 10, 'bold'), 
                 width=15).pack(side='left', padx=5, pady=5)
        
        tk.Button(btn_frame, text="📊 Map Stats", command=lambda: self.send_command('t'),
                 bg='#9C27B0', fg='white', font=('Arial', 10, 'bold'), 
                 width=15).pack(side='left', padx=5, pady=5)
        
        tk.Button(btn_frame, text="🔄 Reset Map", command=self.reset_map,
                 bg='#795548', fg='white', font=('Arial', 10, 'bold'), 
                 width=15).pack(side='left', padx=5, pady=5)
        
        # Second row for visualizer button
        viz_frame = tk.Frame(map_frame, bg='#2b2b2b')
        viz_frame.pack(pady=5)
        
        tk.Button(viz_frame, text="📈 Open Visualizer", command=self.open_visualizer,
                 bg='#607D8B', fg='white', font=('Arial', 10, 'bold'), 
                 width=45).pack()
    
    def create_log_area(self):
        """สร้างพื้นที่แสดง log"""
        log_frame = tk.LabelFrame(self.root, text="Status Log", 
                                 font=('Arial', 12, 'bold'), 
                                 fg='white', bg='#2b2b2b')
        log_frame.pack(fill='both', expand=True, padx=20, pady=(0, 20))
        
        # Text widget with scrollbar
        text_frame = tk.Frame(log_frame, bg='#2b2b2b')
        text_frame.pack(fill='both', expand=True, padx=5, pady=5)
        
        self.log_text = tk.Text(text_frame, height=8, bg='#1e1e1e', fg='#00ff00', 
                               font=('Consolas', 10), wrap=tk.WORD)
        scrollbar = tk.Scrollbar(text_frame, orient='vertical', command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        self.log_text.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')
        
        # Clear log button
        tk.Button(log_frame, text="🗑️ Clear Log", command=self.clear_log,
                 bg='#607D8B', fg='white', font=('Arial', 10)).pack(pady=5)
        
        self.log("🤖 Robot Controller GUI started")
        self.log("📡 Please select COM port and connect")
    
    def scan_ports(self):
        """สแกนหา COM ports"""
        ports = serial.tools.list_ports.comports()
        port_list = []
        
        for port in ports:
            port_list.append(port.device)
            if any(keyword.lower() in port.description.lower() 
                  for keyword in ['CP210', 'CH340', 'ESP32', 'Arduino']):
                self.log(f"🎯 Found ESP32-like device: {port.device} - {port.description}")
        
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.set(port_list[0])
            self.log(f"🔍 Found {len(port_list)} COM ports")
        else:
            self.log("❌ No COM ports found")
    
    def toggle_connection(self):
        """เชื่อมต่อ/ตัดการเชื่อมต่อ"""
        if not self.ser or not self.ser.is_open:
            self.connect()
        else:
            self.disconnect()
    
    def connect(self):
        """เชื่อมต่อกับ ESP32"""
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Please select a COM port")
            return
        
        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            self.running = True
            
            # Start reading thread
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()
            
            self.status_text.set(f"Connected to {port}")
            self.status_label.config(fg='green')
            self.connect_btn.config(text="🔌 Disconnect", bg='#F44336')
            self.log(f"✅ Connected to {port}")
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect to {port}:\n{e}")
            self.log(f"❌ Connection failed: {e}")
    
    def disconnect(self):
        """ตัดการเชื่อมต่อ"""
        self.running = False
        if self.ser:
            self.ser.close()
            
        self.status_text.set("Disconnected")
        self.status_label.config(fg='red')
        self.connect_btn.config(text="📡 Connect", bg='#2196F3')
        self.log("🔌 Disconnected")
    
    def read_serial(self):
        """อ่านข้อมูลจาก serial"""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.log(f"📨 {line}")
                        
                        # Parse specific messages
                        if "Success" in line:
                            self.log("✅ Command successful!")
                        elif "Fail" in line:
                            self.log("❌ Command failed!")
                            
            except Exception as e:
                if self.running:
                    self.log(f"❌ Read error: {e}")
                break
            time.sleep(0.01)
    
    def send_command(self, command):
        """ส่งคำสั่งไปยัง ESP32"""
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Warning", "Not connected to ESP32")
            return
        
        try:
            self.ser.write(command.encode())
            
            cmd_names = {
                'w': '🔼 FORWARD',
                'a': '⬅️ TURN LEFT',
                's': '🔽 BACKWARD', 
                'd': '➡️ TURN RIGHT',
                'x': '⏹️ STOP',
                'm': '🗺️ SHOW MAP',
                't': '📊 MAP STATS'
            }
            
            cmd_name = cmd_names.get(command, command.upper())
            self.log(f"📤 Sent: {cmd_name}")
            
        except Exception as e:
            self.log(f"❌ Send error: {e}")
            messagebox.showerror("Send Error", f"Failed to send command: {e}")
    
    def reset_map(self):
        """Reset map"""
        result = messagebox.askyesno("Reset Map", "Are you sure you want to reset the map?")
        if result:
            self.log("🔄 Map reset requested")
            # ส่งคำสั่ง reset map ถ้ามี
    
    def open_visualizer(self):
        """เปิด mapping visualizer"""
        import subprocess
        try:
            # เปิด mapping visualizer ใน process แยก
            port = self.port_var.get()
            if port:
                subprocess.Popen(['python', 'mapping_visualizer.py', port])
                self.log("📈 Opening mapping visualizer...")
            else:
                messagebox.showwarning("Warning", "Please connect to a port first")
        except Exception as e:
            self.log(f"❌ Failed to open visualizer: {e}")
    
    def log(self, message):
        """เพิ่มข้อความใน log"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.root.update_idletasks()
    
    def clear_log(self):
        """ล้าง log"""
        self.log_text.delete(1.0, tk.END)
        self.log("🗑️ Log cleared")
    
    def run(self):
        """เริ่มการทำงานของ GUI"""
        try:
            self.root.mainloop()
        finally:
            self.disconnect()

if __name__ == "__main__":
    app = GUIRobotController()
    app.run()