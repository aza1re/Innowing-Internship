import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
import os
import sys
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import pickle
from pupil_apriltags import Detector

print("Importing modules...")
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "control"))
from mydobot import MyDobot


def initialize_pipeline():
    pipeline = rs.pipeline()
    config = rs.config()
    ##################################################
    # TODO: Your Code Here
    # Initialize the RealSense camera pipeline and align color and depth frames.
    ##################################################

    ##################################################
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    ##################################################
    return pipeline, profile, align


def get_camera_intrinsics(profile):
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    return intr.fx, intr.fy, intr.ppx, intr.ppy


def initialize_detector(tag_size):
    return Detector(families="tag36h11", nthreads=1, quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0), tag_size


def process_frames(pipeline, align):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()
    if not color_frame or not depth_frame:
        return None, None
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    return color_image, depth_image


def detect_tags(detector, gray_image, camera_params, tag_size):
    return detector.detect(gray_image, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)


def get_dobot_port():
    config_file = os.path.join(os.path.dirname(__file__), "..", "config", "device_port.yaml")
    with open(config_file, "r") as file:
        config = yaml.safe_load(file)
    return config["device_port"]


def save_transformation(transformation_data, filename):
    with open(filename, "w") as file:
        yaml.dump(transformation_data, file)


def draw_tag_info(color_image, tag):
    for idx in range(len(tag.corners)):
        cv2.line(color_image, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0), 2)
    cv2.circle(color_image, tuple(tag.center.astype(int)), 5, (0, 0, 255), -1)
    center_x, center_y = tag.center.astype(int)
    cv2.putText(color_image, f"ID: {tag.tag_id}", (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


def get_average_transformation(mats):
    R_mats = [mat[:3, :3] for mat in mats]
    rpys = [R.from_matrix(R_mat).as_euler("xyz", degrees=True) for R_mat in R_mats]
    avg_rpy = np.mean(rpys, axis=0)
    avg_R = R.from_euler("xyz", avg_rpy, degrees=True).as_matrix()
    t_mats = [mat[:3, 3] for mat in mats]
    avg_t = np.mean(t_mats, axis=0)
    avg_mat = np.eye(4)
    avg_mat[:3, :3] = avg_R
    avg_mat[:3, 3] = avg_t
    return avg_mat, avg_R, avg_t, avg_rpy


def load_transformation(filename):
    with open(filename, "r") as file:
        transformation = yaml.safe_load(file)
    rotation = np.array(transformation["rotation"])
    translation = np.array(transformation["translation"])
    if translation.shape != (3, 1):
        translation = translation.reshape(3, 1)
    return rotation, translation


def main():
    print("Starting calibration validation...")
    pipeline, profile, align = initialize_pipeline()
    fx, fy, cx, cy = get_camera_intrinsics(profile)
    print(f"Camera intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}")
    detector, tag_size = initialize_detector(0.0792)  # Set the tag size in meters
    port = get_dobot_port()
    print(f"Connecting to Dobot at port: {port}")
    device = MyDobot(port=port)
    print("Homing the robotic arm for calibration...")
    #device.home()

    try:
        R_camera2base, t_camera2base = load_transformation(os.path.join(os.path.dirname(__file__), "..", "config", "camera_to_base_transformation.yaml"))
        print("Transformation loaded.")
        print(f"Rotation \n{R_camera2base}")
        print(f"Translation \n{t_camera2base}\n\n")
    except Exception as e:
        print(f"Error loading transformation: {e}")
        pipeline.stop()
        device.close()
        return

    print("Press Enter to record the AprilTag pose. The arm will move to there. Press 'q' to quit.")

    tag_coordinate = None  # Define tag_coordinate outside the loop
    
    alarm_cleared = True  # Add this flag before the loop

    while True:
        ##################################################
        # TODO: Your Code Here
        # Obtain color and depth frames from the RealSense camera.
        ##################################################
        
        ##################################################
        color_image, depth_image = process_frames(pipeline, align)
        if color_image is None:
            continue
        ##################################################

        # read alarm of robotic arm
        alarm = device.get_alarms()

        if alarm != set():
            if alarm_cleared:  # Only handle alarm once per event
                print("Alarm detected, clear the alarm:", alarm)
                device.set_home(250, 0, 50)
                device.clear_alarms()
                alarm_cleared = False  # Don't keep clearing in a loop
            else:
                # Already tried to clear, wait for user or manual intervention
                print("Alarm persists. Please move the robot away from joint limits and press any key to retry.")
                key = cv2.waitKey(0) & 0xFF
                alarm_cleared = True  # Try again after user input
            continue  # Skip rest of loop until alarm is cleared
        else:
            alarm_cleared = True  # Reset flag when alarm is gone

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        tags = detect_tags(detector, gray_image, [fx, fy, cx, cy], tag_size)

        if tags:
            tag = tags[0]
            if tag.pose_t is not None:
                tag_coordinate = tag.pose_t.copy()  # Make a copy to ensure it's not modified by reference
                draw_tag_info(color_image, tag)
                print(f"Tag ID: {tag.tag_id}, Coordinate: {tag_coordinate.flatten()}")
                # Update display to show tag was found
                cv2.putText(color_image, "Tag Found!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        else:
            # Show message if no tag is found
            cv2.putText(color_image, "No Tag Found", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            tag_coordinate = None

        cv2.imshow("AprilTag Detection", color_image)

        if cv2.getWindowProperty("AprilTag Detection", cv2.WND_PROP_VISIBLE) < 1:
            break

        key = cv2.waitKey(1) & 0xFF

        ##################################################
        # TODO: Your Code Here
        # Handle key events for the robotic arm.
        # E.g., pressing 'q' to quit, and Enter key to move to the tag coordinate.
        ##################################################

        ##################################################
        if key == ord('q'):
            break
        elif key == ord('\r') or key == ord('\n'):
            device.move_to(200, 0, 50, 0, wait=True)  # Move to a safe height before moving to tag position
            if tag_coordinate is not None:
                camera_point = tag.pose_t.flatten()  # Already in meters
                print(f"Camera Point: {camera_point}")
                base_point = R_camera2base @ camera_point + t_camera2base.flatten()
                base_point_mm = base_point * 1000.0  # back to mm
                x, y, z = [float(v) for v in base_point_mm]
                print(f"Moving to tag position: X={x:.2f}, Y={y:.2f}, Z={z + 25:.2f}")

                device.move_to(x, y, z + 25, 0, wait=False)
        ##################################################

    print("Cleaning up resources...")
    pipeline.stop()
    device.close()
    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()