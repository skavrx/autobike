import serial
import matplotlib.pyplot as plt
from asciimatics.screen import Screen
from asciimatics.exceptions import ResizeScreenError, StopApplication
import numpy as np
import io
import sys

# Setup serial connection
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Ensure to set the correct serial port
except serial.SerialException:
    print("Failed to connect to Arduino. Check the device connection and port settings.")
    sys.exit(1)

def read_arduino():
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        try:
            pitch, yaw, kp, ki, kd = map(float, line.split(','))
            return pitch, yaw, kp, ki, kd
        except ValueError:
            print("Received malformed data or read error.")
    return None

def draw_graph(screen):
    data = []
    while True:
        reading = read_arduino()
        if reading:
            pitch, yaw, kp, ki, kd = reading
            data.append((pitch, yaw))
            data = data[-50:]  # Limit to last 50 readings for better performance on small displays

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
    Screen.wrapper(draw_graph)
except ResizeScreenError:
    print("Screen size is too small. Please resize your window.")
except KeyboardInterrupt:
    print("Program exited gracefully.")
finally:
    ser.close()  # Close the serial port
