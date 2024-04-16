import sys
import serial
from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        # Set up the serial port
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        
        # Create a timer to read data
        self.timer = QtCore.QTimer()
        self.timer.setInterval(500)  # Refresh rate in milliseconds
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()
        
        # Create the main widget
        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)
        
        # Create plots for pitch and yaw
        self.pitch_plot = self.graphWidget.plot(pen='r')
        self.yaw_plot = self.graphWidget.plot(pen='b')
        
        # Data storage
        self.pitch_data = []
        self.yaw_data = []
        
        # Labels and styles
        self.graphWidget.setBackground('w')
        self.graphWidget.setTitle("Real-Time Pitch and Yaw", size="20pt")
        self.graphWidget.setLabel('left', 'Angles', color='black', size=30)
        self.graphWidget.setLabel('bottom', 'Time', color='black', size=30)
        
        self.graphWidget.showGrid(x=True, y=True)
    
    def update_plot_data(self):
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                pitch, yaw, _, _, _ = map(float, line.split(','))
                self.pitch_data.append(pitch)
                self.yaw_data.append(yaw)
                
                # Update the plot
                self.pitch_plot.setData(self.pitch_data, pen='r')
                self.yaw_plot.setData(self.yaw_data, pen='b')
        except ValueError:
            print("Received malformed data or read error.")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())
