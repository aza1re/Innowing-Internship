from pathlib import Path
import open3d as o3d
import numpy as np
from imageio.v2 import imread
import cv2
from scipy.spatial.transform import Rotation
import tqdm

# Load files
depth_path = Path("data/depth")  # png files
rgb_path = Path("data/rgb")  # jpg files
count = 132

def load_rgb(i):
    unorm = np.asarray(imread(rgb_path / f"{i:>06}.jpg")).astype(np.float32)
    return unorm / 255.0

rgbs = [load_rgb(i) for i in range(count)]

def load_depth(i):
    mm = np.asarray(imread(depth_path / f"{i:>06}.png")).astype(np.float32)
    m = mm / 1000.0
    return m

depths = [load_depth(i) for i in range(count)]

# Load camera intrinsic
matrix_path = Path("data/camera_matrix.csv")
matrix = np.loadtxt(matrix_path, delimiter=",")
# ...
intrinsic = () # Open3D camera intrinsic

# Load camera extrinsics
odometries_path = Path("data/odometry.csv")
odometries = () # Odometries

# Resize RGBs to match depth
# ...

# Create point clouds
pcds = []
for rgb, depth, odometry in tqdm.tqdm(
    zip(rgbs, depths, odometries),
    desc="Processing point clouds",
    total=len(rgbs)
):
    # ...
    pcd = () # Open3D point cloud

    pcds.append(pcd)

# Combine point clouds
# ...

# Save the combined point cloud
output_path = Path("data/combined_point_cloud.ply")
# ...

# Visualize the combined point cloud
# ...