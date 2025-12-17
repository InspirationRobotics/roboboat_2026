#!/bin/bash
# Requires python3 in order to work properly.

cd /
echo_msg(){
    echo -e "\033[1;32m$1\033[0m"
}

echo_msg "Starting Adafruit and PYNMEAGPS library installation."
echo_msg "Installing..."
sudo pip3 install Adafruit-Blinka
sudo python3 -m pip install --upgrade pynmeagps
echo_msg "Process complete."
