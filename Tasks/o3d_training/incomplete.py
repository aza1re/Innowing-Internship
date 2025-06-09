# For handling file paths in an OS-independent way
from pathlib import Path

# For 3D point cloud processing and visualization
import open3d as o3d

# For numerical operations and arrays
import numpy as np

# For reading image files
from imageio.v2 import imread

# For image processing (not used yet, but often useful)
import cv2

# For handling 3D rotations (not used yet)
from scipy.spatial.transform import Rotation

# For progress bars in loops
import tqdm

import pandas as pd
from scipy.spatial.transform import Rotation as R

# --- Load file paths and set up dataset size ---
depth_path = Path("data/depth")  # Directory containing depth PNG files
rgb_path = Path("data/rgb")      # Directory containing RGB JPG files
count = 132                      # Number of frames/images to process

# --- Function to load and normalize an RGB image ---
def load_rgb(i):
    # Read image and convert to float32
    unorm = np.asarray(imread(rgb_path / f"{i:>06}.jpg")).astype(np.float32)
    # Normalize to [0, 1] for Open3D
    return unorm / 255.0

# Load all RGB images into a list
rgbs = [load_rgb(i) for i in range(count)]

# --- Function to load and convert a depth image from mm to meters ---
def load_depth(i):
    # Read depth image (in mm)
    mm = np.asarray(imread(depth_path / f"{i:>06}.png")).astype(np.float32)
    # Convert depth from millimeters to meters
    m = mm / 1000.0
    return m

# Load all depth images into a list
depths = [load_depth(i) for i in range(count)]

# --- Load camera intrinsic matrix (3x3) from CSV file ---
matrix_path = Path("data/camera_matrix.csv")
# Load intrinsics as a numpy array
matrix = np.loadtxt(matrix_path, delimiter=",")

# Example: matrix = [[fx, 0, cx], 
#                    [0, fy, cy], 
#                    [0,  0,  1]]

########################################################################
# Scaling the depth image resolution (756, 1008) to the rgb image resolution (1440, 1920, 3)
orig_width, orig_height = 1920, 1440
new_width, new_height = 1008, 756
scale_x = new_width / orig_width
scale_y = new_height / orig_height

fx = matrix[0, 0] * scale_x
fy = matrix[1, 1] * scale_y
cx = matrix[0, 2] * scale_x
cy = matrix[1, 2] * scale_y
height, width = new_height, new_width
intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
########################################################################

# --- Load camera extrinsics (odometry/poses) from CSV file ---
odometries_path = Path("data/odometry.csv")

########################################################################
# Each row should be a flattened 4x4 matrix (16 values per row)
# Each row is: timestamp, frame, x, y, z, qx, qy, qz, qw

df = pd.read_csv(odometries_path)
df.columns = df.columns.str.strip()
poses = []
for _, row in df.iterrows():
    x, y, z = row['x'], row['y'], row['z']
    qx, qy, qz, qw = row['qx'], row['qy'], row['qz'], row['qw']
    # Convert quaternion to rotation matrix
    rot = R.from_quat([qx, qy, qz, qw]).as_matrix()
    # Build 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = [x, y, z]
    poses.append(T)
odometries = poses
########################################################################

# --- Create point clouds for each frame ---
pcds = []
for rgb, depth, odometry in tqdm.tqdm(
    zip(rgbs, depths, odometries),      # Iterate over RGB, depth, and pose for each frame
    desc="Processing point clouds",
    total=len(rgbs)
):
    ########################################################################
    # Resize RGB to match depth resolution if needed
    if rgb.shape[:2] != depth.shape[:2]:
        rgb_resized = cv2.resize(rgb, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_LINEAR)
    else:
        rgb_resized = rgb

    # Convert numpy arrays to Open3D images
    rgb_o3d = o3d.geometry.Image((rgb_resized * 255).astype(np.uint8))
    depth_o3d = o3d.geometry.Image((depth * 1000).astype(np.uint16))  # Open3D expects depth in mm
    
    # Create RGBD image
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_o3d, depth_o3d, convert_rgb_to_intensity=False
    )

    # Create point cloud from RGBD image and camera intrinsics
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd, intrinsic
    )

    # Transform point cloud using odometry (camera pose)
    pcd.transform(odometry)
    ########################################################################

    # Add the point cloud to the list
    pcds.append(pcd)

# --- Combine all point clouds into a single point cloud ---
########################################################################
combined_pcd = o3d.geometry.PointCloud()
for pcd in pcds:
    combined_pcd += pcd
########################################################################

# --- Save the combined point cloud to a PLY file ---
output_path = Path("data/combined_point_cloud.ply")

########################################################################
o3d.io.write_point_cloud(str(output_path), combined_pcd)
print(f"Saved combined point cloud to {output_path}")

# --- Visualize the combined point cloud ---
o3d.visualization.draw_geometries([combined_pcd])
########################################################################