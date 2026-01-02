"""
Test script to test IMU functionality. This includes connectivity and data parsing into a readable format.
Either puts data into a log or prints parsed data directly on the screen.

This test is considered successful when the IMU is connected either to a personal user's computer or to the Jetson on the ASV, and either:
- Prints the data from the GPS quickly (> 3 Hz) on terminal.
- Creates a log (.txt) file in the directory RoboBoat_2025/Test_Scripts/IMU_Tests/IMULogs.
The data should be in the form of: 
    "Time: {self.timestamp}, Acceleration: {self.accel}, Gyroscopic rotation: {self.gyro}\n 
    Magnetic field strength: {self.mag}, Quaternion: {self.quat}, Euler: {self.euler}"
"""

import time

from API.IMU import imu_api
from API.IMU.imu_api import IMUData

def print_imu_data():
    """
    Callback function to simply print IMU data to terminal.
    """
    def print_callback(data : IMUData):
        print(data)

    imu = imu_api.IMU(callback=print_callback)

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            break

    del imu

def log_imu_data():
    """
    Callback function to write IMU data to a text file.
    """
    log = open(f'Test_Scripts/IMU_Tests/IMULogs/IMUlog_{int(time.time())}.txt', "w")

    def log_callback(data: IMUData):
        log.write(str(data) + "\n")

    imu = imu_api(callback=log_callback)

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            break

    del imu

if __name__ == "main":
    # Test print ability
    print_imu_data()

    # Test log ability
    # log_imu_data()

