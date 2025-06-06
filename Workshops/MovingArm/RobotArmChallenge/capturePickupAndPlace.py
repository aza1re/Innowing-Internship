import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import csv
import time
import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml

STRICT_MODEL_CHECK = False  # Set to False to bypass the strict check

# 1. Live preview and capture one image from RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

print("Press 's' to save a photo and run detection, 'q' to quit.")

img_path = r"C:\Users\User\OneDrive\Desktop\Innowing\Workshops\MovingArm\RobotArmChallenge\photo.jpg"
captured = False

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        color_image = np.asanyarray(color_frame.get_data())
        cv2.imshow("RealSense Camera", color_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Quit without capturing.")
            pipeline.stop()
            cv2.destroyAllWindows()
            sys.exit(0)
        elif key == ord('s'):
            cv2.imwrite(img_path, color_image)
            print(f"Saved {img_path}")
            captured = True
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()

if not captured:
    print("No photo captured, exiting.")
    sys.exit(0)

# 2. Run YOLOv8 inference
model_path = r"C:\Users\User\OneDrive\Desktop\Innowing\Workshops\MovingArm\ODworkshop\runs\detect\robot_model_2\robot_model_2\weights\best.pt"
model = YOLO(model_path)

# Define your class names in the same order as your model was trained
CLASS_NAMES = ["Big Bolt", "Metal Nut", "Small Bolt"]

def count_classes(results, class_names):
    counts = {name: 0 for name in class_names}
    for result in results:
        classes = result.boxes.cls.cpu().numpy()
        for cls in classes:
            class_name = class_names[int(cls)]
            counts[class_name] += 1
    return counts

# Loop until we get exactly 2 of each type and 6 objects total (or bypass if not strict)
while True:
    results = model(img_path)
    counts = count_classes(results, CLASS_NAMES)
    total = sum(counts.values())
    print(f"Detected counts: {counts}")
    if (STRICT_MODEL_CHECK and total == 6 and all(v == 2 for v in counts.values())) or not STRICT_MODEL_CHECK:
        if STRICT_MODEL_CHECK:
            print("Correct number of each object detected.")
        else:
            print("Bypassing strict model check, proceeding with current detection.")
        break
    else:
        print("Incorrect number of objects detected. Please adjust the objects and press 's' to capture a new image.")
        # Re-capture image
        captured = False
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)
        while not captured:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            color_image = np.asanyarray(color_frame.get_data())
            cv2.imshow("RealSense Camera", color_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Quit without capturing.")
                pipeline.stop()
                cv2.destroyAllWindows()
                sys.exit(0)
            elif key == ord('s'):
                cv2.imwrite(img_path, color_image)
                print(f"Saved {img_path}")
                captured = True
                break
        pipeline.stop()
        cv2.destroyAllWindows()

# Show the first result with bounding boxes for debugging
result_img = results[0].show()

# 3. Export detections to CSV and get first bolt
output_file = r"C:\Users\User\OneDrive\Desktop\Innowing\Workshops\MovingArm\RobotArmChallenge\detections.csv"
bolt_midpoint = None

with open(output_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["class", "mid_x", "mid_y", "confidence"])
    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        for box, cls, score in zip(boxes, classes, scores):
            x1, y1, x2, y2 = box
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2
            class_name = CLASS_NAMES[int(cls)]
            writer.writerow([class_name, f"{mid_x:.2f}", f"{mid_y:.2f}", f"{score:.2f}"])
            print(f"Detected: {class_name} at midpoint ({mid_x:.2f}, {mid_y:.2f}), confidence: {score:.2f}")
            if class_name.lower() in ["big bolt", "small bolt"] and bolt_midpoint is None:
                bolt_midpoint = (mid_x, mid_y)

# --- Calibration: Load camera-to-base transformation ---
yaml_path = r"C:\Users\User\OneDrive\Desktop\Innowing\Workshops\MovingArm\config\camera_to_base_transformation.yaml"
with open(yaml_path, 'r') as f:
    calib = yaml.safe_load(f)
R_camera2base = np.array(calib['rotation'])
t_camera2base = np.array(calib['translation']).reshape((3, 1))

# --- Camera intrinsics (replace with your actual values!) ---
camera_intrinsics = {
    'fx': 608.4603881835938,  # Focal length x
    'fy': 608.2498779296875,  # Focal length y
    'cx': 329.7581787109375,  # Principal point x
    'cy': 238.36131286621094  # Principal point y
}

# --- Get depth frame for all detections ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()
color_stream = profile.get_stream(rs.stream.color)
intr = color_stream.as_video_stream_profile().get_intrinsics()

def get_average_depth(depth_frame, x, y, window=3):
    vals = []
    for dx in range(-window//2, window//2+1):
        for dy in range(-window//2, window//2+1):
            d = depth_frame.get_distance(int(x+dx), int(y+dy))
            if d > 0:
                vals.append(d)
    return np.mean(vals) if vals else 0

# --- Dobot control section ---
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "control"))
from mydobot import MyDobot
import serial

device = MyDobot(port='COM7')

# Define drop-off locations for each class
DROP_OFFS = {
    "Big Bolt": (73.80, 235.60, 40.00),     # Example coordinates, adjust for your setup
    "Metal Nut": (151.49, 187.67, 40.94),
    "Small Bolt": (214.87, 113.77, 36.67)
}

def check_and_clear_alarms(device):
    try:
        alarms = device.get_alarms()
        if alarms:
            print(f"Alarm(s) detected: {alarms}. Clearing alarms...")
            device.clear_alarms()
            time.sleep(1)
            print("Alarms cleared.")
    except Exception as e:
        print(f"Could not check/clear alarms: {e}")

unsorted = True
while unsorted:
    results = model(img_path)
    boxes = results[0].boxes.xyxy.cpu().numpy()
    classes = results[0].boxes.cls.cpu().numpy()
    scores = results[0].boxes.conf.cpu().numpy()
    found_object = False

    for idx, (box, cls, score) in enumerate(zip(boxes, classes, scores)):
        x1, y1, x2, y2 = box
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2
        class_name = CLASS_NAMES[int(cls)]

        # Get depth at this midpoint
        # depth = get_average_depth(depth_frame, mid_x, mid_y)
        distance = depth_frame.get_distance(int(mid_x), int(mid_y))
        coordinate = rs.rs2_deproject_pixel_to_point(intr, [mid_x, mid_y], distance)
        # print(f"Detected: {class_name} at ({mid_x:.2f}, {mid_y:.2f}), depth: {depth:.3f}m, confidence: {score:.2f}")

        # Convert to camera coordinates
        # fx, fy = camera_intrinsics['fx'], camera_intrinsics['fy']
        # cx, cy = camera_intrinsics['cx'], camera_intrinsics['cy']
        # x_cam = (mid_x - cx) * depth / fx
        # y_cam = (mid_y - cy) * depth / fy
        # z_cam = depth
        # camera_point = np.array([x_cam, y_cam, z_cam])

        # Transform to base coordinates
        base_point = R_camera2base @ coordinate + t_camera2base.flatten()
        base_point_mm = base_point * 1000.0
        x, y, z = [float(v) for v in base_point_mm]
        x -= 25
        y -= 70 # Adjust for camera offset if needed
        z = -20  # Adjust for table height
        print(f"Adjusted: X={x:.2f}, Y={y:.2f}, Z={z + 25:.2f}")
        

        #Current Dobot position: X=167.59, Y=-164.37, Z=-21.60, R=-10.06
        # Move above the object
        device.grip(False)
        device.move_to(x, y, z + 25, 0, wait=True)
        check_and_clear_alarms(device)

        # Move down to the object
        device.move_to(x, y, z, 0, wait=True)
        check_and_clear_alarms(device)

        # Activate gripper/suction to pick up
        device.grip(False)  # False for suction, True for gripper
        check_and_clear_alarms(device)
        device.grip(True)

        # Move up after picking
        device.move_to(x, y, z + 25, 0, wait=True)
        check_and_clear_alarms(device)

        # Move to drop-off for this class
        drop_x, drop_y, drop_z = DROP_OFFS[class_name]
        device.move_to(drop_x, drop_y, drop_z + 25, 0, wait=True)
        check_and_clear_alarms(device)

        device.move_to(drop_x, drop_y, drop_z, 0, wait=True)
        check_and_clear_alarms(device)

        # Release gripper/suction to drop
        device.grip(True)  # True to release
        time.sleep(1)
        device.grip(False)  # False to deactivate suction/gripper
        check_and_clear_alarms(device)

        # Move up after dropping
        device.move_to(drop_x, drop_y, drop_z + 25, 0, wait=True)
        check_and_clear_alarms(device)

        print(f"Dropped {class_name} at ({drop_x}, {drop_y}, {drop_z})")

        # Take a picture after sorting each item
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if color_frame:
            color_image = np.asanyarray(color_frame.get_data())
            img_after_sort_path = f"sorted_{class_name}_{int(time.time())}.jpg"
            cv2.imwrite(img_after_sort_path, color_image)
            print(f"Saved image after sorting: {img_after_sort_path}")
            img_path = img_after_sort_path  # Use new image for next detection
        pipeline.stop()

        found_object = True
        break  # Only sort one object per loop, then re-detect

    if not found_object:
        unsorted = False  # No more objects detected

device.close()

