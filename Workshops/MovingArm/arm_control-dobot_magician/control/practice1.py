import sys
import os
import time
import yaml

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'control'))

from mydobot import MyDobot

def get_dobot_port():
    config_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'device_port.yaml')
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    return config['device_port']

def main():
    port = get_dobot_port()
    device = MyDobot(port=port)

    print("Homing the robotic arm for calibration...")
    device.home()

    try:
        while True:
            """
            YOUR CODE START HERE
            """
            pose = device.get_pose().position
            print(f"X: {pose.x:.2f}, Y: {pose.y:.2f}, Z: {pose.z:.2f}, R: {pose.r:.2f}")
            """
            YOUR CODE ENDS HERE
            """
            time.sleep(0.05)  # Adjust the sleep time as needed
    except KeyboardInterrupt:
        print("Stopping coordinate output.")
    finally:
        device.close()

if __name__ == "__main__":
    main()
