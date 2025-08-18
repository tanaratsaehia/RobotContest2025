#!/usr/bin/env python3
"""
Connection Test Tool for Robot Mapping System
Tests serial communication with ESP32 control station
"""

import serial
import serial.tools.list_ports
import time
import sys

def list_available_ports():
    """List all available COM ports"""
    ports = serial.tools.list_ports.comports()
    print("Available COM ports:")
    if not ports:
        print("  No COM ports found!")
        return []
    
    for i, port in enumerate(ports):
        print(f"  {i+1}. {port.device} - {port.description}")
    return [port.device for port in ports]

def test_port(port_name, baud_rate=115200):
    """Test communication with a specific port"""
    print(f"\nTesting communication with {port_name}...")
    
    try:
        # Open serial connection
        ser = serial.Serial(port_name, baud_rate, timeout=2, dsrdtr=False)
        time.sleep(2)  # Wait for ESP32 to reset
        
        print(f"✓ Successfully opened {port_name}")
        
        # Clear any existing data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Test sending a command
        test_commands = ['w', 's', 'a', 'd', 'x']
        
        print("Testing command transmission...")
        for cmd in test_commands:
            ser.write((cmd + '\n').encode())
            print(f"  Sent: {cmd}")
            time.sleep(0.5)
        
        # Listen for responses
        print("\nListening for responses (10 seconds)...")
        start_time = time.time()
        received_data = []
        
        while time.time() - start_time < 10:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        received_data.append(line)
                        print(f"  Received: {line}")
                except:
                    pass
            time.sleep(0.1)
        
        ser.close()
        
        if received_data:
            print(f"✓ Communication successful! Received {len(received_data)} messages")
            return True
        else:
            print("⚠ No data received. Check ESP32 programming and connections.")
            return False
            
    except serial.SerialException as e:
        print(f"✗ Serial error: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False

def test_esp32_detection():
    """Test if ESP32 is detected in COM ports"""
    ports = serial.tools.list_ports.comports()
    esp32_ports = []
    
    for port in ports:
        description = port.description.lower()
        if any(keyword in description for keyword in ['esp32', 'cp210', 'ch340', 'ftdi', 'usb serial']):
            esp32_ports.append(port)
    
    if esp32_ports:
        print("\nPossible ESP32 devices found:")
        for port in esp32_ports:
            print(f"  {port.device} - {port.description}")
        return [port.device for port in esp32_ports]
    else:
        print("\n⚠ No ESP32-like devices detected")
        print("Common ESP32 USB chips: CP2102, CH340, FTDI")
        return []

def interactive_test():
    """Interactive testing mode"""
    print("=== Robot Communication Test Tool ===\n")
    
    # List available ports
    available_ports = list_available_ports()
    if not available_ports:
        print("No COM ports available. Please:")
        print("1. Connect your ESP32 via USB")
        print("2. Install ESP32 USB drivers")
        print("3. Check Device Manager for COM ports")
        return
    
    # Check for ESP32-like devices
    esp32_ports = test_esp32_detection()
    
    print("\nSelect a port to test:")
    for i, port in enumerate(available_ports):
        marker = " (ESP32?)" if port in esp32_ports else ""
        print(f"  {i+1}. {port}{marker}")
    print(f"  {len(available_ports)+1}. Test all ports")
    print("  0. Exit")
    
    try:
        choice = int(input("\nEnter your choice: "))
        
        if choice == 0:
            return
        elif choice == len(available_ports) + 1:
            # Test all ports
            print("\nTesting all available ports...")
            working_ports = []
            for port in available_ports:
                if test_port(port):
                    working_ports.append(port)
            
            if working_ports:
                print(f"\n✓ Working ports: {', '.join(working_ports)}")
            else:
                print("\n✗ No working ports found")
        elif 1 <= choice <= len(available_ports):
            selected_port = available_ports[choice-1]
            test_port(selected_port)
        else:
            print("Invalid choice")
            
    except ValueError:
        print("Please enter a valid number")
    except KeyboardInterrupt:
        print("\nTest interrupted by user")

def automated_test():
    """Automated testing for all ports"""
    print("=== Automated Port Testing ===\n")
    
    available_ports = list_available_ports()
    if not available_ports:
        print("No ports available for testing")
        return
    
    working_ports = []
    for port in available_ports:
        if test_port(port):
            working_ports.append(port)
    
    print(f"\n=== Test Results ===")
    print(f"Total ports tested: {len(available_ports)}")
    print(f"Working ports: {len(working_ports)}")
    
    if working_ports:
        print("Recommended port(s) for robot controller:")
        for port in working_ports:
            print(f"  {port}")
    else:
        print("No working ports found. Please check:")
        print("1. ESP32 is connected via USB")
        print("2. ESP32 has the control_station.ino uploaded")
        print("3. USB drivers are installed")

if __name__ == "__main__":
    print("Robot Mapping System - Connection Test\n")
    
    if len(sys.argv) > 1 and sys.argv[1] == "--auto":
        automated_test()
    else:
        try:
            interactive_test()
        except KeyboardInterrupt:
            print("\n\nTest terminated by user")
        
    print("\nTest completed. Press Enter to exit...")
    input()