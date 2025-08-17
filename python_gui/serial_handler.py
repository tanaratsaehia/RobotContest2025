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
            self.thread = threading.Thread(target=self._read_serial)
            self.thread.start()
            self.last_activity = time.time()
        except serial.SerialException as e:
            print(f"Error starting serial connection: {e}")
            if self.on_connection_lost:
                self.on_connection_lost(e)