#!/usr/bin/env python3
# Enhanced controller.py - ปรับปรุงให้ใช้งานง่ายขึ้น

import serial
import sys
import threading
import time
import os

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
    
    main()