import os
from ultralytics import YOLO

HOME = os.getcwd() # Get the current working directory
# Path to your data.yaml
data_yaml_path = r"C:\Users\User\OneDrive\Desktop\Innowing\Workshops\MovingArm\ODworkshop\datasets\datasets_Robot\data.yaml"

# Create a YOLOv8 model (nano version for speed, or use yolov8s.yaml for better accuracy)
model = YOLO("yolov8n.yaml")

# Train the model
results = model.train(
    data=data_yaml_path,
    epochs=300,           # You can increase for better results
    imgsz=640,           # Image size (should match your dataset, 640 is standard)
    batch=4,            # Batch size (adjust based on your GPU memory)
    project=r"C:\Users\User\OneDrive\Desktop\Innowing\Workshops\MovingArm\ODworkshop\runs\detect\robot_model_2", # Where to save results
    name="robot_model_2",     # Name for this training run
    save_period=-1
)