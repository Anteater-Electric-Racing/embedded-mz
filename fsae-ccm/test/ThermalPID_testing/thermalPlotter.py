import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque
import threading

class SerialPlotter:
    def __init__(self, port='COM6', baudrate=11520, max_points=100):
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points
        self.data = deque(maxlen=max_points)
        self.timestamps = deque(maxlen=max_points)
        self.time_counter = 0
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"Connected to {port} at {baudrate} baud")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            self.ser = None
    
    def read_serial(self):
        """Read data from serial port"""
        if self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        try:
                            if'temp:' in line.lower():
                                temp = float(line.lower().split('temp:')[1].split()[0])
                                self.data.append(temp)
                                self.timestamps.append(self.time_counter)
                                self.time_counter += 1
                                return temp
                        except (ValueError, IndexError):
                            print(f"Could not parse value: {line}")
            except Exception as e:
                print(f"Error reading serial: {e}")
        return None
    
    def update_plot(self, frame):
        """Update plot with new data"""
        self.read_serial()
        
        plt.clf()
        if self.data:
            plt.plot(list(self.timestamps), list(self.data), 'b-o', linewidth=2, markersize=4)
            plt.xlabel('Time (samples)')
            plt.ylabel('Temperature (°C)')
            plt.title('Motor Temperature')
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
    
    def start_plotting(self, interval=100):
        """Start real-time plotting"""
        fig = plt.figure(figsize=(10, 6))
        ani = FuncAnimation(fig, self.update_plot, interval=interval, blit=False)
        plt.show()
    
    def close(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed")


if __name__ == "__main__":
    SERIAL_PORT = 'COM6'
    BAUDRATE = 115200
    
    plotter = SerialPlotter(port=SERIAL_PORT, baudrate=BAUDRATE, max_points=100)
    
    try:
        plotter.start_plotting(interval=100)
    except KeyboardInterrupt:
        print("\nPlotting stopped by user")
    finally:
        plotter.close()
        plt.close('all')  # ← add this, closes matplotlib cleanly
