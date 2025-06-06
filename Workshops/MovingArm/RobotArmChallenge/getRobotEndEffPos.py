import sys
import os

# Add your control folder to the path if needed
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "control"))
from mydobot import MyDobot

# Replace 'COM7' with your Dobot's serial port
device = MyDobot(port='COM7')

input("Manually move the Dobot to the desired position, then press Enter...")

# Read and print the current position
try:
    pos = device.get_pose()
    print("Raw position object:", pos)
    print("Position attribute:", pos.position)
    # If pos.position is a tuple/list of (x, y, z, r):
    x, y, z, r = pos.position
    print(f"Current Dobot position: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, R={r:.2f}")
except Exception as e:
    print("Could not read Dobot position:", e)
device.close()