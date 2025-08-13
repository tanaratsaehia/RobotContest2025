import serial
import sys
import tty
import termios
import threading

# Replace with your ESP32 serial port
SERIAL_PORT = '/dev/ttyUSB2'  # e.g., 'COM3' on Windows
BAUD_RATE = 115200

# Open serial without toggling DTR (avoid auto-reset)
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, dsrdtr=False)

def getch():
    """Read single char from keyboard without enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def read_serial():
    """Continuously read serial from ESP32"""
    while True:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip().strip(r"\t").strip(r"\n")
            if line:
                print("\n", line)

# Start serial reading in a background thread
thread = threading.Thread(target=read_serial, daemon=True)
thread.start()

print("[Python] Use WASD keys to control robot. Press 'q' to quit.")

try:
    while True:
        key = getch()
        if key == 'q':
            break
        # if key in ['w', 'a', 's', 'd']:
        if key:
            ser.write(key.encode())
            print(f"[Python] Sent: {key}")
finally:
    ser.close()
