import serial, sys, tty, termios, threading, re, time

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, dsrdtr=False)

# Sensor <num>: <float> cm
SENSOR_RE = re.compile(r"Sensor\s*(\d+)\s*:\s*([+-]?\d+(?:\.\d+)?)\s*cm", re.I)

sensor_vals = {1: None, 2: None, 3: None, 4: None}

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def read_serial():
    while True:
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
    # Allocate exactly 4 lines once (no header to keep math simple).
    sys.stdout.write("\n" * 4)
    sys.stdout.flush()
    while True:
        # Move to start of the 4-line block (4 lines up), column 0
        sys.stdout.write("\033[4F")
        for i in range(1, 5):
            v = sensor_vals[i]
            line = f"S{i}: {v:.1f} cm" if v is not None else f"S{i}: —"
            # Option B: clear that line and rewrite it, then newline to go to next line
            sys.stdout.write("\r\033[K" + line + "\n")
        sys.stdout.flush()
        time.sleep(0.1)

threading.Thread(target=read_serial, daemon=True).start()
threading.Thread(target=render_loop, daemon=True).start()

try:
    while True:
        key = getch()
        if key == 'q':
            break
        if key:
            ser.write(key.encode())
finally:
    ser.close()
