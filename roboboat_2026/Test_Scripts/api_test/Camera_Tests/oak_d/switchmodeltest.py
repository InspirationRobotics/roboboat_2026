"""Chat GPT generated, need to modify"""
import time
from Perception.Perception_Core.perception_core import CameraCore  # Ensure this imports your CameraCore class
from GNC.Guidance_Core.mission_helper import MissionHelper

# Load config
config = MissionHelper()
config = config.load_json(path="GNC/Guidance_Core/Config/barco_polo.json")

# Define paths to models
MODEL_1 = config["test_model_path"]
MODEL_2 = config["sign_model_path"]

# Label Map (Ensure it matches your detection classes)
LABELMAP_1 = config["test_label_map"]
LABELMAP_2 = config["sign_label_map"]

def test_switch_model():
    """Test switching between two models."""
    print("\nInitializing CameraCore with the first model...")
    camera = CameraCore(MODEL_2, LABELMAP_2)
    
    print("\nStarting camera...")
    camera.start()
    time.sleep(5)  # Let it run for a few seconds

    print("\nSwitching to the second model...")
    camera.switchModel(MODEL_1,LABELMAP_1)
    time.sleep(5)  # Allow time for the model switch to take effect

    print("\nSwitching back to the first model...")
    camera.switchModel(MODEL_2,LABELMAP_2)
    time.sleep(5)  # Ensure the camera restarts correctly

    print("\nStopping camera...")
    camera.stop()
    print("Test completed successfully!")

if __name__ == "__main__":
    test_switch_model()
