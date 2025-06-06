import pyrealsense2 as rs

# Start pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

# Get stream profile and intrinsics
color_stream = profile.get_stream(rs.stream.color)
intr = color_stream.as_video_stream_profile().get_intrinsics()

print("Camera Intrinsics:")
print(f"  fx: {intr.fx}")
print(f"  fy: {intr.fy}")
print(f"  cx: {intr.ppx}")
print(f"  cy: {intr.ppy}")
print(f"  width: {intr.width}")
print(f"  height: {intr.height}")

pipeline.stop()