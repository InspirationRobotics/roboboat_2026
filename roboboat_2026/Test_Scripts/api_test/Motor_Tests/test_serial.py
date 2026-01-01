"""
Test in order to verify that we can send serial connections to the Arduino on Barco Polo.
This serves the dual purpose of testing port connectivity (hardware) and verifying that the file on our arduino, motor_interface.ino, both work properly.

This test is considered successful when:
- The thrusters remain quiet for around 5 seconds. The print statement should read "First command complete (neutral commands)."
- The forward starboard thruster fires for 5 seconds.
- The forward port thruster fires for 5 seconds.
- The forward starboard thruster fires for 5 seconds.
- The thrusters go quiet.
This should be a continuous sequence.
"""

import serial
import time
from API.Util import device_helper

arduino_port = device_helper.dataFromConfig("arduino_port")

connection = serial.Serial(port = arduino_port, baudrate = 9600, timeout = 0.1)
time.sleep(1)

print("Starting test.")

command = "1500,1500,1500,1500"
connection.write(command.encode())

print("First command complete (neutral commands).")
time.sleep(5)

command = "1500,1540,1500,1500"
connection.write(command.encode())

print("Command complete (forward starboard).")
time.sleep(5)

command = "1540,1500,1500,1500"
connection.write(command.encode())

print("Command complete (forward port).")
time.sleep(5)

command = "1500,1540,1500,1500"
connection.write(command.encode())

print("Command complete (forward starboard).")
time.sleep(5)

command = "1500,1500,1500,1500"
connection.write(command.encode())

print("Going back to neutral, last command complete.")
time.sleep(5)

connection.close()