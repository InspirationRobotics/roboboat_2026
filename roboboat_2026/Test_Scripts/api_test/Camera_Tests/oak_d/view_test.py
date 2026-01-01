import cv2
import depthai as dai
import numpy as np

# Create a DepthAI pipeline
pipeline = dai.Pipeline()

# Create nodes for left, right, and color cameras
CAM_A = pipeline.create(dai.node.ColorCamera)
CAM_B= pipeline.create(dai.node.ColorCamera)
CAM_C = pipeline.create(dai.node.ColorCamera)

# Create output nodes
xout_A = pipeline.create(dai.node.XLinkOut)
xout_B = pipeline.create(dai.node.XLinkOut)
xout_C = pipeline.create(dai.node.XLinkOut)

# Set stream names
xout_A.setStreamName("CAM_A")
xout_B.setStreamName("CAM_B")
xout_C.setStreamName("CAM_C")


# Configure color camera
CAM_A.setBoardSocket(dai.CameraBoardSocket.CAM_A)
# CAM_A.setIspScale(2, 3)  # Reduce resolution if needed
CAM_C.setBoardSocket(dai.CameraBoardSocket.CAM_C)
CAM_B.setBoardSocket(dai.CameraBoardSocket.CAM_B)

CAM_A.isp.link(xout_A.input)
CAM_B.isp.link(xout_B.input)
CAM_C.isp.link(xout_C.input)

# Connect to the device and start the pipeline
with dai.Device(pipeline) as device:
    CAM_A_q = device.getOutputQueue(name="CAM_A", maxSize=4, blocking=False)
    CAM_B_q = device.getOutputQueue(name="CAM_B", maxSize=4, blocking=False)
    CAM_C_q = device.getOutputQueue(name="CAM_C", maxSize=4, blocking=False)

    print("Capturing images...")

    while (True):
        # Get frames
        camA_frame= CAM_A_q.get().getCvFrame()
        camB_frame = CAM_B_q.get().getCvFrame()
        camC_frame = CAM_C_q.get().getCvFrame()

        # Display images (optional)
        cv2.imshow("Left Camera", camA_frame)
        cv2.imshow("Right Camera", camB_frame)
        cv2.imshow("Color Camera", camC_frame)

        if cv2.waitKey(1) == ord('q'):
            break
    
    cv2.destroyAllWindows()
