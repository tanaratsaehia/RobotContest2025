import serial
import sys
import threading
import re
import time
import platform

# Cross-platform key reading
if platform.system() == 'Windows':
    import msvcrt
    def getch():
        return msvcrt.getch().decode('utf-8', errors='ignore')
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
        return ch

SERIAL_PORT = 'COM8'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, dsrdtr=False)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
except serial.SerialException as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")
    print("Available ports:")
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"  {port.device}: {port.description}")
    sys.exit(1)

# Sensor <num>: <float> cm
SENSOR_RE = re.compile(r"Sensor\s*(\d+)\s*:\s*([+-]?\d+(?:\.\d+)?)\s*cm", re.I)

sensor_vals = {1: None, 2: None, 3: None, 4: None}

def read_serial():
    """Read serial data and parse sensor values"""
    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='ignore').strip()
            if not line:
                continue
                
            # Print raw data for debugging (optional)
            # print(f"Raw: {line}")
            
            m = SENSOR_RE.search(line)
            if not m:
                continue
                
            idx = int(m.group(1))
            val = float(m.group(2))
            if 1 <= idx <= 4:
                sensor_vals[idx] = val
                
        except Exception as e:
            # Handle any parsing errors gracefully
            continue

def render_loop():
    """Render sensor values in real-time"""
    # Clear screen and move to top
    print("\033[2J\033[H", end="")
    print("Serial Sensor Monitor")
    print("Press 'q' to quit, any other key to send to device")
    print("-" * 40)
    
    # Allocate exactly 4 lines for sensors
    sys.stdout.write("\n" * 4)
    sys.stdout.flush()
    
    while True:
        try:
            # Move cursor up 4 lines to start of sensor display area
            sys.stdout.write("\033[4F")
            
            for i in range(1, 5):
                v = sensor_vals[i]
                if v is not None:
                    line = f"Sensor {i}: {v:6.1f} cm"
                    # Add simple bar graph
                    if 0 <= v <= 100:
                        bars = int(v / 5)  # Scale for display
                        bar_graph = "█" * bars + "░" * (20 - bars)
                        line += f" │{bar_graph}│"
                else:
                    line = f"Sensor {i}: ——————— cm │░░░░░░░░░░░░░░░░░░░░│"
                
                # Clear line and write new content
                sys.stdout.write(f"\r\033[K{line}\n")
            
            sys.stdout.flush()
            time.sleep(0.1)
            
        except Exception as e:
            # Handle any display errors
            continue

def main():
    """Main program loop"""
    print("Starting sensor monitor...")
    
    # Start background threads
    serial_thread = threading.Thread(target=read_serial, daemon=True)
    render_thread = threading.Thread(target=render_loop, daemon=True)
    
    serial_thread.start()
    render_thread.start()
    
    print("Monitor started. Waiting for data...")
    time.sleep(1)  # Give threads time to start
    
    try:
        while True:
            key = getch()
            
            if key.lower() == 'q':
                print("\nExiting...")
                break
                
            if key:
                try:
                    ser.write(key.encode())
                    print(f"\nSent: '{key}'")
                except Exception as e:
                    print(f"\nError sending data: {e}")
                    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        
    finally:
        try:
            ser.close()
            print("Serial port closed")
        except:
            pass

if __name__ == "__main__":
    main()