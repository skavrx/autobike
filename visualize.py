import kivy
kivy.require('2.0.0')  # replace with your current kivy version!

from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.clock import Clock
from kivy.garden.graph import Graph, MeshLinePlot
from kivy.uix.button import Button
import serial
import threading

class DataPlotter(BoxLayout):
    def __init__(self, **kwargs):
        super(DataPlotter, self).__init__(**kwargs)
        self.orientation = 'vertical'
        
        # Graphs for pitch and yaw
        self.graph = Graph(xlabel='Time', ylabel='Values', x_ticks_minor=5,
                           x_ticks_major=25, y_ticks_major=1,
                           y_grid_label=True, x_grid_label=True, padding=5,
                           x_grid=True, y_grid=True, xmin=0, xmax=100, ymin=-90, ymax=90)
        self.pitch_plot = MeshLinePlot(color=[1, 0, 0, 1])
        self.yaw_plot = MeshLinePlot(color=[0, 1, 0, 1])
        self.graph.add_plot(self.pitch_plot)
        self.graph.add_plot(self.yaw_plot)
        
        # Labels for PID constants
        self.kp_label = Label(text='Kp: 0.0')
        self.ki_label = Label(text='Ki: 0.0')
        self.kd_label = Label(text='Kd: 0.0')
        
        # Control button
        self.stop_button = Button(text='Stop', size_hint=(1, 0.1))
        self.stop_button.bind(on_press=self.stop)
        
        # Add widgets to layout
        self.add_widget(self.graph)
        self.add_widget(self.kp_label)
        self.add_widget(self.ki_label)
        self.add_widget(self.kd_label)
        self.add_widget(self.stop_button)
        
        # Serial setup
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.data_thread = threading.Thread(target=self.read_data)
        self.data_thread.daemon = True
        self.data_thread.start()
        
    def read_data(self):
        while True:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                pitch, yaw, kp, ki, kd = map(float, line.split(','))
                self.update_graph(pitch, yaw)
                self.update_labels(kp, ki, kd)
    
    def update_graph(self, pitch, yaw):
        self.pitch_plot.points = [(i, x[0]) for i, x in enumerate(self.data)]
        self.yaw_plot.points = [(i, x[1]) for i, x in enumerate(self.data)]
        
    def update_labels(self, kp, ki, kd):
        self.kp_label.text = f'Kp: {kp}'
        self.ki_label.text = f'Ki: {ki}'
        self.kd_label.text = f'Kd: {kd}'
    
    def stop(self, instance):
        self.ser.close()
        App.get_running_app().stop()

class MyApp(App):
    def build(self):
        return DataPlotter()

if __name__ == '__main__':
    MyApp().run()
