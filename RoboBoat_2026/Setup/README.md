# Setup Scripts for RoboBoat 2025 Documentation
Contained in this folder are all of the scripts to install dependencies that are needed to run the code in this repository.
This is an ideal situation -- there may be dependencies that need to be installed that have been overlooked/have not been implemented here.
NOTE: Scripts are meant for use on Ubuntu versions later 20.04 and later. If you are using Windows, please run
```Windows terminal
wsl --install
```
if you wish to use bash scripts. You can also use Git Bash (which seems to work better -- not many external management issues). Simply download Git Bash, and open it.

## Setup for WSL
```bash
wsl --install
```
You will then have to setup some basic account information.
```bash
# Enter WSL
wsl
# Exit WSL
exit
```
```bash
# To access documents from C: drive (Windows)
wsl
cd /mnt/c/Users/[username]
```
To install dos2unix
```bash
sudo apt-get update
sudo apt install dos2unix
```

## Usage
For each file you wish to execute, navigate to the parent folder of the file, then, replacing the "file_name" with the file you wish to execute, run:
```bash
sudo chmod +x [file_name].sh
sudo ./[file_name].sh
```

## Sequence
At the time of writing, the setup.sh file is not ready. The intention is that by running that file, all other files will be called and ran without having to deal with running multiple scripts.

Please run in the following order. This sequence assumes a fully new OS (of some Ubuntu version). If some dependencies are already installed, just skip to the next one:
1. install_git.sh (This is the one file in which you will have to copy and paste the file somewhere on your operating system before running -- run this file like all other files using the process documeted in [Usage](#usage).)

You can then clone the repository onto your local machine:
```bash
git clone https://github.com/InspirationRobotics/RoboBoat_2025
```
Navigate into the Setup folder, then continue the sequence:
2. install_python_pip_simple.sh (install_python_from_source.sh works as well, but you are more likely to encounter issues)
3. install_open_cv.sh
4. install_pyusb_libusb.sh
5. v4ctl.setup.sh

## List of files
- python3
- pip
- OpenCV
- Smopy, ipython
- YOLO/Ultralytics
- Filterpy (for sensor fusion)
- PyUSB/LibUSB
- Numpy
