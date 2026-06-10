import tkinter as tk
from tkinter import ttk
import serial
import time

# Configure your serial port here
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for serial to initialize
except Exception as e:
    print(f"Error opening serial port: {e}")
    ser = None

def send_command(prefix, val):
    if ser:
        cmd = f"{prefix}{val}\n"
        ser.write(cmd.encode())
        print(f"Sent: {cmd.strip()}")

def update_yaw(val):
    send_command('Y', val)

def update_pitch(val):
    send_command('P', val)

def update_roll(val):
    send_command('R', val)

# Create main window
root = tk.Tk()
root.title("XIAO Gimbal Controller")
root.geometry("400x300")

style = ttk.Style()
style.configure("TLabel", font=("Helvetica", 12))

# Yaw Slider
ttk.Label(root, text="Yaw (Pin 0)").pack(pady=5)
yaw_slider = ttk.Scale(root, from_=0, to=180, orient='horizontal', command=update_yaw)
yaw_slider.set(90)
yaw_slider.pack(fill='x', padx=20)

# Pitch Slider
ttk.Label(root, text="Pitch (Pin 1)").pack(pady=5)
pitch_slider = ttk.Scale(root, from_=0, to=180, orient='horizontal', command=update_pitch)
pitch_slider.set(90)
pitch_slider.pack(fill='x', padx=20)

# Roll Slider
ttk.Label(root, text="Roll (Pin 2)").pack(pady=5)
roll_slider = ttk.Scale(root, from_=0, to=180, orient='horizontal', command=update_roll)
roll_slider.set(90)
roll_slider.pack(fill='x', padx=20)

# Center Button
def center_all():
    yaw_slider.set(90)
    pitch_slider.set(90)
    roll_slider.set(90)
    send_command('Y', 90)
    send_command('P', 90)
    send_command('R', 90)

ttk.Button(root, text="Center All", command=center_all).pack(pady=20)

root.mainloop()

if ser:
    ser.close()
