from ultralytics import YOLO
import os
import cv2
from PIL import Image
# from sensor_msgs.msg import Image
import time
import psutil, glob

# # Directory containing the test images
# images_dir = r"/home/mrsd/Desktop/RS Del Code-20230919T000925Z-001/RoboSpect Code/YOLO V8/imagesfortest"

# # List all image files in the directory
# image_files = [f for f in os.listdir(images_dir) if f.endswith(('.jpg', '.jpeg', '.png', '.bmp'))]

# i = 1

def yolo_main(rgb_img):
    
    # Load the trained YOLOv8 Nano models
    model2 = YOLO(r"/home/sthirupa/ros2_ws/src/pipe_explorer/rs_yolo/rs_yolo/YOLOv8 Code/train32/weights/best.pt")
    model1 = YOLO(r"/home/sthirupa/ros2_ws/src/pipe_explorer/rs_yolo/rs_yolo/YOLOv8 Version 2 Trained/train10/weights/best.pt")

    # Process each image in the folder
    # for image_file in image_files:
    # Path to the current image
    # image_path = os.path.join(images_dir, image_file)
    # Load the uploaded image
    # img = cv2.imread(image_path)

    results1 = model1.predict(rgb_img)
    result1 = results1[0]

    results2 = model2.predict(rgb_img)
    result2 = results2[0]

    total_cords = []
    result = None

    # Check if there are bounding boxes
    if result1.boxes or result2.boxes :
        # print(f"Bounding boxes found in {image_file}")

        if not result1.boxes:
            result = result2
        elif not result2.boxes:
            result = result1
        else:
            maxprob1 = max([round(box1.conf[0].item(), 2) for box1 in result1.boxes])
            maxprob2 = max([round(box2.conf[0].item(), 2) for box2 in result2.boxes])
            if maxprob1 >= maxprob2:
                result = result1
            else:
                result = result2
                
        for box in result.boxes:
            class_id = result.names[box.cls[0].item()]
            cords = box.xyxy[0].tolist()
            cords = [float(round(x)) for x in cords]
            conf = round(box.conf[0].item(), 2)

            total_cords += cords
            
            print("Object type:", class_id)
            print("Coordinates:", cords)
            print("Probability:", conf)
            print("---")

        # Display the image with bounding boxes
        # Image.fromarray(result.plot()[:, :, ::-1]).show()
        # time.sleep(2)
        # for proc in psutil.process_iter():
        #     if proc.name() == "display" or proc.name()[-4:] == ".PNG":
        #         proc.kill()
        
        return '1', total_cords

    else:
        return '0', [float(1), float(1), float(2), float(2)]
        # print(f"No bounding boxes found in {image_file}")

# i = i+1    
