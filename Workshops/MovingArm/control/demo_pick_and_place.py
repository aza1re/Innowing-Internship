import sys
import os
import yaml

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'control'))

# Using the self-defined MyDobot class
from mydobot import MyDobot

def get_dobot_port():
    config_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'device_port.yaml')
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    return config['device_port']

def main():
    port = get_dobot_port()
    device = MyDobot(port="COM7")

    print("Homing the robotic arm for calibration...")
    device.home()

    while True:
        input("Move to point A and press Enter...")
        point_a = device.get_pose().position
        device.suck(True)

        input("Move to point B and press Enter...")
        point_b = device.get_pose().position
        device.suck(False)

        while True:
            confirm = input("Confirm points and start pick and place routine? (y/n/q): ")
            if confirm.lower() == 'y':
                while True:
                    device.jump_to(point_a.x, point_a.y, point_a.z, point_a.r, height=50, wait=True)
                    device.suck(True, wait_time=0.5)
                    device.jump_to(point_b.x, point_b.y, point_b.z, point_b.r, height=50, wait=True)
                    device.suck(False)
            elif confirm.lower() == 'n':
                break
            elif confirm.lower() == 'q':
                device.close()
                return
            else:
                print("Invalid input. Please enter 'y' to confirm, 'n' to re-register points, or 'q' to quit.")

if __name__ == "__main__":
    main()
