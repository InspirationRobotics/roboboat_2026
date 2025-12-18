#!/bin/bash

# If run on WSL, will lead to externally managed issues.
# Use Git Bash if you are on Windows.
echo_msg(){
    echo -e "\033[1;32m$1\033[0m"
}
cd /

echo_msg "Beginning install for smopy..."
pip install smopy

echo_msg "Beginning install for ipython..."
pip install smopy[ipython]

echo_msg "Installation of smopy and ipython complete."