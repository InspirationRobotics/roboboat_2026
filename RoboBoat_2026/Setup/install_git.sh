#!/bin/bash

echo "Beginning Git installation."
sudo add-apt-repository -y ppa:git-core/ppa
sudo apt update
sudo apt install -y git
echo "Git installation complete."
