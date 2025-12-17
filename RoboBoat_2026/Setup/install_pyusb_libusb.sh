#!/bin/bash
# Assumes python and pip have been installed.

cd /

echo_msg(){
    echo -e "\033[1;32m$1\033[0m"
}

echo_msg "Beginning PyUSB and libusb package installation."

sudo apt update
sudo apt install libusb-1.0-0-dev
pip install pyusb

echo_msg "Verifying pyusb has been installed..."
python3 -m pip show pyusb

echo_msg "PyUSB installation and libusb installation complete."