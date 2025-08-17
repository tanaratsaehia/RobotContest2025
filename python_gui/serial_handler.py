#!/usr/bin/env python3
"""
Serial Communication Handler
สำหรับจัดการการสื่อสารผ่าน Serial port
"""

import serial
import threading
import queue
import time
from typing import Callable, Optional

class SerialHandler:
    """Class สำหรับจัดการการสื่อสาร Serial"""
    
    def __init__(self, port: str, baud_rate: int = 115200, data_queue: Optional[queue.Queue] = None):
        self.port = port
        self.baud_rate = baud_rate
        self.data_queue = data_queue or queue.Queue()
        
        # Serial connection
        self.serial_conn = None
        self.running = False
        self.thread = None
        
        # Callbacks
        self.on_data_received = None
        self.on_connection_lost = None
        
        # Statistics
        self.bytes_sent = 0
        self.bytes_received = 0
        self.messages_sent = 0
        self.messages_received = 0
        self.connection_time = 0
        self.last_activity = 0
        
#!/usr/bin/env python3
"""
Serial Communication Handler
สำหรับจัดการการสื่อสารผ่าน Serial port
"""

import serial
import threading
import queue
import time
from typing import Callable, Optional

class SerialHandler:
    """Class สำหรับจัดการการสื่อสาร Serial"""
    
    def __init__(self, port: str, baud_rate: int = 115200, data_queue: Optional[queue.Queue] = None):
        self.port = port
        self.baud_rate = baud_rate
        self.data_queue = data_queue or queue.Queue()
        
        # Serial connection
        self.serial_conn = None
        self.running = False
        self.thread = None
        
        # Callbacks
        self.on_data_received = None
        self.on_connection_lost = None
        
        # Statistics
        self.bytes_sent = 0
        self.bytes_received = 0
        self.messages_sent = 0
        self.messages_received = 0
        self.connection_time = 0
        self.last_activity = 0
        
    def start(self):
        """เริ่มต้นการเชื่อมต่อ Serial และ Thread สำหรับอ่านข้อมูล"""
        if self.running:
            return
        
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.running = True
            self.connection_time = time.time()
            self.thread = threading.Thread(target=self._read_serial, daemon=True)
            self.thread.start()
            self.last_activity = time.time()
            print(f"✅ Serial connection started on {self.port}")
        except serial.SerialException as e:
            print(f"❌ Error starting serial connection: {e}")
            if self.on_connection_lost:
                self.on_connection_lost(e)
                
    def stop(self):
        """หยุดการเชื่อมต่อ Serial"""
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        if self.thread:
            self.thread.join(timeout=1)
        print("📴 Serial connection stopped")
        
    def is_connected(self) -> bool:
        """ตรวจสอบสถานะการเชื่อมต่อ"""
        return self.running and self.serial_conn and self.serial_conn.is_open
        
    def send(self, data: str):
        """ส่งข้อมูลผ่าน Serial"""
        if not self.is_connected():
            raise Exception("Serial not connected")
            
        try:
            # Add newline if not present
            if not data.endswith('\n'):
                data += '\n'
                
            bytes_data = data.encode('utf-8')
            self.serial_conn.write(bytes_data)
            self.bytes_sent += len(bytes_data)
            self.messages_sent += 1
            self.last_activity = time.time()
            
        except serial.SerialException as e:
            print(f"❌ Error sending data: {e}")
            if self.on_connection_lost:
                self.on_connection_lost(e)
            raise
            
    def _read_serial(self):
        """อ่านข้อมูลจาก Serial (ทำงานใน thread แยก)"""
        buffer = ""
        
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting:
                    data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    self.bytes_received += len(data.encode('utf-8'))
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            self.messages_received += 1
                            self.last_activity = time.time()
                            
                            # Put data in queue
                            self.data_queue.put(line)
                            
                            # Call callback if set
                            if self.on_data_received:
                                self.on_data_received(line)
                                
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
                
            except serial.SerialException as e:
                if self.running:
                    print(f"❌ Serial read error: {e}")
                    if self.on_connection_lost:
                        self.on_connection_lost(e)
                break
            except Exception as e:
                if self.running:
                    print(f"❌ Unexpected error in serial read: {e}")
                break
                
    def flush_input(self):
        """ล้าง input buffer"""
        if self.is_connected():
            self.serial_conn.reset_input_buffer()
            
    def flush_output(self):
        """ล้าง output buffer"""
        if self.is_connected():
            self.serial_conn.reset_output_buffer()
            
    def get_stats(self) -> dict:
        """ได้สถิติการสื่อสาร"""
        uptime = time.time() - self.connection_time if self.connection_time > 0 else 0
        
        return {
            'connected': self.is_connected(),
            'port': self.port,
            'baud_rate': self.baud_rate,
            'uptime': uptime,
            'bytes_sent': self.bytes_sent,
            'bytes_received': self.bytes_received,
            'messages_sent': self.messages_sent,
            'messages_received': self.messages_received,
            'last_activity': self.last_activity
        }
        
    def set_callbacks(self, on_data_received: Callable = None, on_connection_lost: Callable = None):
        """ตั้งค่า callback functions"""
        self.on_data_received = on_data_received
        self.on_connection_lost = on_connection_lost