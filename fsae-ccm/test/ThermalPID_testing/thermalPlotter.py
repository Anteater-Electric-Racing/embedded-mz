import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

class SerialPlotter:
    def __init__(self, port='COM6', baudrate=115200, max_points=100):
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points

        self.temp_data  = deque(maxlen=max_points)
        self.fan_data   = deque(maxlen=max_points)
        self.pump1_data = deque(maxlen=max_points)
        self.pump2_data = deque(maxlen=max_points)
        self.timestamps = deque(maxlen=max_points)
        self.time_counter = 0

        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"Connected to {port} at {baudrate} baud")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            self.ser = None

    def read_serial(self):
        if self.ser and self.ser.is_open:
            try:
                while self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    print(f"raw: {line}")
                    lower = line.lower()
                    try:
                        if 'temp:' in lower:
                            self.temp_data.append(float(lower.split('temp:')[1].split()[0]))
                            self.timestamps.append(self.time_counter)
                            self.time_counter += 1
                        elif 'fan output:' in lower:
                            self.fan_data.append(float(lower.split('fan output:')[1].split()[0]))
                        elif 'pump1 output:' in lower:
                            self.pump1_data.append(float(lower.split('pump1 output:')[1].split()[0]))
                        elif 'pump2 output:' in lower:
                            self.pump2_data.append(float(lower.split('pump2 output:')[1].split()[0]))
                    except (ValueError, IndexError):
                        print(f"Could not parse: {line}")
            except Exception as e:
                print(f"Serial error: {e}")

    def update_plot(self, frame):
        self.read_serial()

        t = list(self.timestamps)

        if t:
            self.line_temp.set_data(t, list(self.temp_data)[:len(t)])
            self.ax1.set_xlim(max(0, t[-1] - 100), t[-1] + 5)
            self.ax1.set_ylim(0, 130)

        if self.fan_data:
            n = min(len(t), len(self.fan_data))
            self.line_fan.set_data(t[:n], list(self.fan_data)[:n])
            self.ax2.set_xlim(max(0, t[-1] - 100), t[-1] + 5)
            self.ax2.set_ylim(0, 1.1)

        if self.pump1_data:
            n = min(len(t), len(self.pump1_data))
            self.line_pump1.set_data(t[:n], list(self.pump1_data)[:n])
            self.ax3.set_xlim(max(0, t[-1] - 100), t[-1] + 5)
            self.ax3.set_ylim(0, 1.1)

        if self.pump2_data:
            n = min(len(t), len(self.pump2_data))
            self.line_pump2.set_data(t[:n], list(self.pump2_data)[:n])
            self.ax4.set_xlim(max(0, t[-1] - 100), t[-1] + 5)
            self.ax4.set_ylim(0, 1.1)

        return self.line_temp, self.line_fan, self.line_pump1, self.line_pump2

    def start_plotting(self, interval=50):
        fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle('Thermal System — Live', fontsize=14)

        self.line_temp,  = self.ax1.plot([], [], color='red',    linewidth=2)
        self.line_fan,   = self.ax2.plot([], [], color='blue',   linewidth=2)
        self.line_pump1, = self.ax3.plot([], [], color='green',  linewidth=2)
        self.line_pump2, = self.ax4.plot([], [], color='purple', linewidth=2)

        self.ax1.set_title('Motor Temperature'); self.ax1.set_ylabel('Temp (°C)')
        self.ax2.set_title('Fan Output');        self.ax2.set_ylabel('PWM (0-1)')
        self.ax3.set_title('Pump 1 Output');     self.ax3.set_ylabel('PWM (0-1)')
        self.ax4.set_title('Pump 2 Output');     self.ax4.set_ylabel('PWM (0-1)')

        self.ax1.axhline(80, color='orange', linestyle='--', linewidth=1, label='Setpoint 80°C')
        self.ax1.legend(loc='upper left', fontsize=8)

        for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
            ax.set_xlabel('Time (samples)')
            ax.grid(True, alpha=0.3)

        plt.tight_layout()

        ani = FuncAnimation(fig, self.update_plot, interval=interval, blit=True, cache_frame_data=False)
        plt.show()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed")


if __name__ == "__main__":
    SERIAL_PORT = 'COM6'
    BAUDRATE    = 115200

    plotter = SerialPlotter(port=SERIAL_PORT, baudrate=BAUDRATE, max_points=100)

    try:
        plotter.start_plotting(interval=50)
    except KeyboardInterrupt:
        print("\nPlotting stopped by user")
    finally:
        plotter.close()
        plt.close('all')