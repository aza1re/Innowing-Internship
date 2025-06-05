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
    port = config.get('device_port')
    if not port or not os.path.exists(port):
        raise FileNotFoundError(f"Port {port} not found. Please check the configuration.")
    return port

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
            coordinates = [
                (100, -200, 50, 0),
                (200, -100, 50, 0),
                (200, 150, 50, 0)
            ]
            for coord in coordinates:
                device.move_to(*coord)
                time.sleep(1)
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
