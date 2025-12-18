#!/bin/bash
# Requires python3 in order to work properly.

cd /
echo_msg(){
    echo -e "\033[1;32m$1\033[0m"
}

echo_msg "Starting OpenCV installation. This may take a while, so please be patient."
echo_msg "Installing..."
sudo apt update
sudo apt install libopencv-dev python3-opencv

echo_msg "Verifying that OpenCV installed successfully..."
python3 -c "import cv2; print(cv2.__version__)"

echo_msg "OpenCV installation complete."