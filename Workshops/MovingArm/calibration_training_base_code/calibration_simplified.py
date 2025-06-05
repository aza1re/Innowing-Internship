import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
import os
import sys
import math
from scipy.spatial.transform import Rotation as R
from pupil_apriltags import Detector

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "control"))
from mydobot import MyDobot


def initialize_pipeline():
    pipeline = rs.pipeline()
    config = rs.config()
    ##################################################
    # TODO: Your Code Here
    # Initialize the RealSense camera pipeline and align color and depth frames.
    ##################################################

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    
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
    # getting each rotation angle (roll, pitch, yaw)
    rpys = [R.from_matrix(R_mat).as_euler("xyz", degrees=True) for R_mat in R_mats]
    avg_rpy = np.mean(rpys, axis=0)
    avg_R = R.from_euler("xyz", avg_rpy, degrees=True).as_matrix()
    t_mats = [mat[:3, 3] for mat in mats]
    avg_t = np.mean(t_mats, axis=0)
    avg_mat = np.eye(4)
    avg_mat[:3, :3] = avg_R
    avg_mat[:3, 3] = avg_t
    return avg_mat, avg_R, avg_t, avg_rpy


def main():
    pipeline, profile, align = initialize_pipeline()
    fx, fy, cx, cy = get_camera_intrinsics(profile)
    detector, tag_size = initialize_detector(0.0792)  # Set the tag size in meters 0.0792
    port = get_dobot_port()
    device = MyDobot(port=port)
    pose = device.get_pose()
    print("Pose returned by device.get_pose():", pose)
    print("Homing the robotic arm for calibration...")
    print("Attach the AprilTag 15.3cm above the gripper and make the x-axes aligned before calibration.")
    #device.home()

    cHt_list = []
    bHg_list = []

    print("At the camera window, press Enter to record data, press 'q' to quit and calculate the transformation.")

    while True:
        ##################################################
        # TODO: Your Code Here
        # Obtain color and depth frames from the RealSense camera.
        ##################################################
        color_image, depth_image = process_frames(pipeline, align)
        if color_image is None:
            continue
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        tags = detect_tags(detector, gray_image, [fx, fy, cx, cy], tag_size)

        # cHt, transforms a point in the target frame to the camera frame, which is the pose of the tag in camera frame.
        # bHg, transforms a point in the gripper frame to the base frame, which is the pose of the gripper in base frame.
        if tags:
            tag = tags[0]
            if tag.pose_t is not None:
                tag_coordinate = tag.pose_t.copy()  # Make a copy to ensure it's not modified by reference
                draw_tag_info(color_image, tag)
                # Update display to show tag was found
                cv2.putText(color_image, "Tag Found!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                ##################################################
                # TODO: Your Code Here
                # Get cHt transformation and bHg transformation
                ##################################################
                # camera ← tag
                cHt = np.eye(4)
                cHt[:3, :3] = tag.pose_R
                cHt[:3,  3] = tag.pose_t.flatten()


                # base ← gripper
                # MyDobot.get_pose() returns (x, y, z, r)
                pose = device.get_pose()
                xg = pose.position.x
                yg = pose.position.y
                zg = pose.position.z
                r  = pose.position.r

                # convert mmxg, yg, zg→m
                xyz_m = np.array([xg, yg, zg]) / 1000.0
                # build rotation about Z only (gripper yaw)
                Rg = np.array([0, 0, np.arctan2(yg, xg)])  # yaw angle in radians
                Rg = R.from_rotvec(Rg).as_matrix()
                bHg = np.eye(4)
                bHg[:3, :3] = Rg
                bHg[:3,  3] = xyz_m
            else:
                cv2.putText(color_image, "No Tag Found", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                tag_coordinate = None

        cv2.imshow("AprilTag Detection", color_image)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("\r") or key == ord("\n"):  # Enter key
            if tags:
                # saving all the matrix
                cHt_list.append(cHt)
                bHg_list.append(bHg)
                print("Raw tag.pose_t (meters):", tag.pose_t.flatten())
                print(f"Data recorded. Total data points: {len(cHt_list)}")

    if len(cHt_list) >= 1:
        ##################################################
        # TODO: Your Code Here
        # Get the inverse of each cHt matrix
        ##################################################
        tHc_list = [np.linalg.inv(mat) for mat in cHt_list]

        # gripper to tag transformation (x-axes aligned)
        # [-1,  0,  0, 30]
        # [0, 1,  0,   0]
        # [0,  0, -1, 153]
        # [0,  0,  0,   1]
        # tag size = 0.0792 meters

        gHt = np.array([
            [ -1,  0,  0, 0.030 ],  # 30 mm in meters
            [ 0, 1,  0, 0  ],
            [ 0,  0, -1, 0.153 ],  # 153 mm in meters
            [ 0,  0,  0, 1  ]
        ])

        ##################################################
        # TODO: Your Code Here
        # Define the gripper to tag transformation matrix (gHt)
        ##################################################

        bHc_list = [
            bHg_list[i] @ gHt @ tHc_list[i]
            for i in range(len(cHt_list))
        ]

        ##################################################
        # TODO: Your Code Here
        # Calculate bHc_list, which is the transformation from base to camera frame
        ##################################################

        # base → camera = base→gripper @ gripper→tag @ tag→camera
        cHb_list = [
            bHg @ gHt @ tHc
            for bHg, tHc in zip(bHg_list, tHc_list)
        ]

        avg_cHb_mat, avg_cHb_R, avg_cHb_t, _ = get_average_transformation(cHb_list)

        transformation_data = {"rotation": avg_cHb_R.tolist(), "translation": avg_cHb_t.tolist()}
        save_transformation(transformation_data, os.path.join(os.path.dirname(__file__), "..", "config", "camera_to_base_transformation.yaml"))
    else:
        print("Not enough data points to calculate the transformation.")

    pipeline.stop()
    device.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
