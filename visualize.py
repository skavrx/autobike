import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from asciimatics.screen import Screen
from asciimatics.exceptions import ResizeScreenError, StopApplication
import numpy as np
import io
import sys

def find_arduino_serial_port(baud_rate=115200):
    ports = list(serial.tools.list_ports.comports())
    for port, desc, hwid in sorted(ports):
        try:
            ser = serial.Serial(port, baud_rate, timeout=1)
            print(f"Trying {port}...")
            # Optionally, send a specific command to Arduino to trigger a known response
            # ser.write(b'your_initialization_command\n')
            # Read from the Arduino
            response = ser.readline().decode().strip()
            if response:  # Add specific checks for your Arduino's data format
                print(f"Arduino found on {port}")
                return ser
            ser.close()
        except (OSError, serial.SerialException):
            pass
    print("No Arduino found. Check connections.")
    sys.exit(1)

def read_arduino(ser):
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        try:
            pitch, yaw, kp, ki, kd = map(float, line.split(','))
            return pitch, yaw, kp, ki, kd
        except ValueError:
            print("Received malformed data or read error.")
    return None

def draw_graph(screen, ser):
    data = []
    while True:
        reading = read_arduino(ser)
        if reading:
            pitch, yaw, kp, ki, kd = reading
            data.append((pitch, yaw))
            data = data[-50:]  # Limit to last 50 readings

            # Clear the previous content
            screen.clear_buffer(screen.COLOUR_WHITE, screen.A_NORMAL, screen.COLOUR_BLACK)
            screen.refresh()

            # Display PID values as text
            screen.print_at(f'Pitch: {pitch:.2f}   Yaw: {yaw:.2f}', 0, 0)
            screen.print_at(f'Kp: {kp:.2f}   Ki: {ki:.2f}   Kd: {kd:.2f}', 0, 1)

            # Plotting the data using matplotlib
            plt.figure(figsize=(5, 2))
            plt.plot([d[0] for d in data], label='Pitch')
            plt.plot([d[1] for d in data], label='Yaw')
            plt.legend(loc='upper right')
            plt.title('Pitch and Yaw Over Time')
            plt.grid(True)

            # Convert plot to image and display in terminal
            buf = io.BytesIO()
            plt.savefig(buf, format='png')
            buf.seek(0)
            screen.print_at('Graph updating...', 0, 2)  # Placeholder for actual graph integration
            screen.refresh()
            plt.close()
        else:
            screen.print_at('No new data...', 0, 2)
            screen.refresh()

try:
    ser = find_arduino_serial_port()
    Screen.wrapper(draw_graph, arguments=(ser,))
except ResizeScreenError:
    print("Screen size is too small. Please resize your window.")
except KeyboardInterrupt:
    print("Program exited gracefully.")
finally:
    if ser:
        ser.close()  # Ensure the serial port is closed on program exit
