import serial
import matplotlib.pyplot as plt
from asciimatics.screen import Screen
from asciimatics.exceptions import ResizeScreenError, StopApplication
from serial.tools import list_ports
import numpy as np
import io
import sys

def find_arduino():
    """Search for an Arduino in connected serial devices."""
    ports = list_ports.comports()
    for port in ports:
        if "Arduino" in port.description or "USB Serial Device" in port.description:
            return port.device
    return None

def setup_serial_connection():
    """Establishes serial connection with Arduino."""
    arduino_port = find_arduino()
    if arduino_port is None:
        print("No Arduino found. Please connect your Arduino device.")
        sys.exit(1)
    try:
        ser = serial.Serial(arduino_port, 115200, timeout=1)
        print(f"Connected to Arduino on {arduino_port}")
        return ser
    except serial.SerialException:
        print("Failed to connect on ", arduino_port)
        sys.exit(1)

def read_arduino(ser):
    """Read data from Arduino serial connection."""
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        try:
            pitch, yaw, kp, ki, kd = map(float, line.split(','))
            return pitch, yaw, kp, ki, kd
        except ValueError:
            print("Received malformed data or read error.")
    return None

def draw_graph(screen, ser):
    """Function to draw graph and display values on screen."""
    data = []
    while True:
        reading = read_arduino(ser)
        if reading:
            pitch, yaw, kp, ki, kd = reading
            data.append((pitch, yaw))
            data = data[-50:]  # Keep last 50 readings

            # Display PID values as text
            screen.clear()
            screen.print_at(f'Pitch: {pitch:.2f}   Yaw: {yaw:.2f}', 0, 0)
            screen.print_at(f'Kp: {kp:.2f}   Ki: {ki:.2f}   Kd: {kd:.2f}', 0, 1)

            # Update screen
            screen.refresh()
        else:
            screen.print_at('Waiting for new data...', 0, 2)
            screen.refresh()

try:
    ser = setup_serial_connection()
    Screen.wrapper(draw_graph, arguments=(ser,))
except ResizeScreenError:
    print("Screen size is too small. Please resize your window.")
except KeyboardInterrupt:
    print("Program exited gracefully.")
finally:
    ser.close()  # Ensure serial connection is closed properly
