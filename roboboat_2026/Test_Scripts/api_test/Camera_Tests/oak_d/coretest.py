"""Chat GPT generated"""

import cv2
import time
import threading
import queue
from pathlib import Path
from Perception.Perception_Core.perception_core import CameraCore  # Adjust path if needed
from GNC.Guidance_Core.mission_helper import MissionHelper

# Load config
config = MissionHelper().load_json(path="GNC/Guidance_Core/Config/barco_polo.json")

# Define paths to models
MODEL_1 = config["test_model_path"]
MODEL_2 = config["sign_model_path"]

# Label Map (Ensure it matches your detection classes)
LABELMAP_1 = config["test_label_map"]
LABELMAP_2 = config["sign_label_map"]

# Initialize the camera core
camera = CameraCore(model_path=MODEL_2, labelMap=LABELMAP_2)
camera.start()

# Create a queue to pass frames between threads
frame_queue = queue.Queue(maxsize=4)
on = threading.Event()  # Use an Event instead of a bool for thread-safe stopping
on.set()  # Set it to True initially

lock = threading.Lock()

# Function to capture frames
def capture_frames():
    while on.is_set():
        start_time = time.time_ns()
        depth = camera.get_object_depth(scale=0.2)
        frame = camera.visualize()
        end_time = time.time_ns()
        print(f"Used {((end_time - start_time) / 1e9):.2f} s to get frame")

        # Put the captured frame into the queue
        with lock:
            try:
                if frame_queue.full():
                    frame_queue.get_nowait()  # Remove the oldest frame
                frame_queue.put_nowait(frame)
            except queue.Empty:
                pass
        time.sleep(1 / 20)  # Adjust to match the FPS (20 FPS)

# Start the capture thread
capture_thread = threading.Thread(target=capture_frames, daemon=True)
capture_thread.start()

# Main loop to display frames
def display_frames(window_name):
    while on.is_set():
        if not frame_queue.empty():
            frame = frame_queue.get()
            ss = time.time_ns()
            cv2.imshow(window_name, frame)
            ee = time.time_ns()
            print(f"Used {((ee - ss) / 1e9):.2f} s to display")

        # Check for the 'q' key to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            on.clear()  # Signal the thread to stop
            break

# Display frames for model 1
display_frames("rgb_model1")

# Stop capturing before switching models
on.clear()
capture_thread.join()
camera.stop()

# Switch to model 2
print("Switching model...")
camera.switchModel(modelPath=MODEL_2, labelMap=LABELMAP_2)
time.sleep(5)  # Ensure model switch completion

# Restart capturing with the new model
on.set()
camera.start()
capture_thread = threading.Thread(target=capture_frames, daemon=True)
capture_thread.start()

# Display frames for model 2
display_frames("rgb_model2")

# Cleanup
camera.stop()
on.clear()
capture_thread.join()
cv2.destroyAllWindows()
