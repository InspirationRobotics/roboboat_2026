
#!/usr/bin/env python3

"""
The code is the same as for Tiny Yolo V3 and V4, the only difference is the blob file
- Tiny YOLOv3: https://github.com/david8862/keras-YOLOv3-model-set
- Tiny YOLOv4: https://github.com/TNTWEN/OpenVINO-YOLOV4

This is a test script for integrating nn and depth map, currently we are able to dipslay them seperately and show depth value when hover over pixels
TODO Find out the width and height of the OAK_D LR camear, and use that information to find the appropriate size for our neural network.
TODO Integrate the bounding box with the depth map, currently they are speperated.
"""

from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time

# TODO: The confidence level when using the cross model is oddly hight, need to find out why
# Also the camera can't detect the label when it's far away, we need to train new model so it can see the cross from far away

# Get yolo v8n model blob file path
# Using the example trained nn model for testing, will need to switch to our own model once trianing is done.
nnPath = str((Path(__file__).parent / Path('../../../../Perception/Models/test_model/yolov8n_coco_640x352.blob')).resolve().absolute())
if not Path(nnPath).exists():
    import sys
    print(f"Your path: {nnPath}")
    raise FileNotFoundError(f'Required file/s not found"')

# yolo v8 abel texts
# Need to switch to our own label map
labelMap = [
    "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
    "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"
]

syncNN = True

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = True
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = True
# Better handling for occlusions:
lr_check = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
leftCam = pipeline.create(dai.node.ColorCamera)
rightCam = pipeline.create(dai.node.ColorCamera)
detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
stereo = pipeline.create(dai.node.StereoDepth)
xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutDepth =pipeline.create(dai.node.XLinkOut)
nnOut = pipeline.create(dai.node.XLinkOut)

# set output nodes
xoutDepth.setStreamName("depth")
xoutRgb.setStreamName("rgb")
nnOut.setStreamName("nn")

# Properties
leftCam.setIspScale(2, 3)
leftCam.setPreviewSize(640, 352) #TODO our preview size should be 640 x 400 because our original camera size is 1920 x 1200
leftCam.setCamera("left")
leftCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
leftCam.setInterleaved(False)
leftCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
leftCam.setFps(40)

# set right cam
rightCam.setIspScale(2, 3)
rightCam.setPreviewSize(640, 352)
rightCam.setCamera("right")
rightCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
rightCam.setInterleaved(False)
rightCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
rightCam.setFps(40)

# config for stereo post processing

config = stereo.initialConfig.get()
config.postProcessing.speckleFilter.enable = True
config.postProcessing.speckleFilter.speckleRange = 100
config.postProcessing.temporalFilter.enable = False
config.postProcessing.spatialFilter.enable =  False
config.postProcessing.spatialFilter.holeFillingRadius = 2
config.postProcessing.spatialFilter.numIterations = 1
# config.postProcessing.thresholdFilter.minRange = 400
# config.postProcessing.thresholdFilter.maxRange = 15000
# config.postProcessing.decimationFilter.decimationFactor = 1

# set stereo property
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
stereo.initialConfig.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)   #POST PROCESSING
# stereo.initialConfig.set(config)
# link to post process https://docs.luxonis.com/software/depthai/examples/depth_post_processing/#depthai.MedianFilter.MEDIAN_OFF 
stereo.setLeftRightCheck(lr_check)
stereo.setExtendedDisparity(extended_disparity)
stereo.setSubpixel(subpixel)

# Network specific settings
detectionNetwork.setConfidenceThreshold(0.5)
detectionNetwork.setNumClasses(80)
detectionNetwork.setCoordinateSize(4)
detectionNetwork.setIouThreshold(0.5)
detectionNetwork.setBlobPath(nnPath)
detectionNetwork.setNumInferenceThreads(2)
detectionNetwork.input.setBlocking(False)


# Link rgb & nn
leftCam.preview.link(detectionNetwork.input)
if syncNN:
    detectionNetwork.passthrough.link(xoutRgb.input)
else:
    leftCam.preview.link(xoutRgb.input)

detectionNetwork.out.link(nnOut.input)

# Link left right cam to stereo
leftCam.isp.link(stereo.left)
rightCam.isp.link(stereo.right)
stereo.depth.link(xoutDepth.input)

maxDisp = stereo.initialConfig.getMaxDisparity()

# define mouse callback
def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        value = depth_map[y,x]
        if value is not None:
            print(f"Depth at ({x}, {y}): {value/1000} meters", end="\r")

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    cv2.namedWindow("depth")
    cv2.namedWindow("combined")
    cv2.namedWindow("rgb")
    # cv2.setMouseCallback("depth", on_mouse)
    # cv2.setMouseCallback("combined", on_mouse)
    # cv2.setMouseCallback("rgb", on_mouse)


    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
    qDepth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    frame = None
    depth_map = None
    detections = []
    startTime = time.monotonic()
    counter = 0
    color2 = (255, 255, 255)

    # nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
    def frameNorm(frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    def displayFrame(name, frame):
        color = (255, 0, 0)
        for detection in detections:
            # TODO: Investigate into the label index.
            # print(f"label index: {detection.label}")
            bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
            # print(bbox)
            cv2.putText(frame, labelMap[detection.label], (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)  # I added -1 because the label is one whne I only have one detect object
            cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        # Show the frame
        cv2.imshow(name, frame)
    
    while True:
        if syncNN:
            inRgb = qRgb.get()
            inDet = qDet.get()
            inDepth = qDepth.get()
        else:
            inRgb = qRgb.tryGet()
            inDet = qDet.tryGet()
            inDepth = qDepth.tryget()

        if inRgb is not None:
            frame = inRgb.getCvFrame()
            cv2.putText(frame, "NN fps: {:.2f}".format(counter / (time.monotonic() - startTime)),
                        (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color2)

        if inDet is not None:
            detections = inDet.detections
            counter += 1

        if inDepth is not None:
            depth_map = inDepth.getCvFrame()
            depth_map = depth_map[0:352,0:640] # trim stereo to mach nn
            depth_normalized = (depth_map * (255.0 / maxDisp)).astype(np.uint8)
            cv2.imshow("depth",depth_normalized)

        if frame is not None:
            displayFrame("rgb", frame)
            displayFrame("combined", depth_normalized)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('s'):
            for dect in detections:
                print(f"label: {dect.label}")
                print(f"confidence: {dect.confidence}")
                print(f"xmin: {dect.xmin}")

                print("\n")
