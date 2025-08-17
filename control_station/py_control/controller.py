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