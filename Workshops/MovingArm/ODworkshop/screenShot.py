import pyrealsense2 as rs
import numpy as np
import cv2
import os

def main():
    # Create a pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    print("Press 's' to save a photo, 'q' to quit.")

    save_dir = r"C:\Users\User\OneDrive\Desktop\Innowing\Workshops\MovingArm\ODworkshop\datasets\datasets_Robot\test\images"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    photo_count = 0

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
                break
            elif key == ord('s'):
                filename = os.path.join(save_dir, f"photo_{photo_count:03d}.jpg")
                cv2.imwrite(filename, color_image)
                print(f"Saved {filename}")
                photo_count += 1

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()