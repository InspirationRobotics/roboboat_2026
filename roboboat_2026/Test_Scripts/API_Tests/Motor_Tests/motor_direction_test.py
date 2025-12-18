"""
This is a motor direction test. It will run each of the motors at a moderate forward speed for 4 seconds each.
It is meant to testdirectionalities of the thrusters for when the thrusters are firing "forward". This is a WATER test.

The test is considered successful when:
- The forward port thruster runs for 4 seconds
- The forward starboard thruster runs for 4 seconds
- The aft port thruster runs for 4 seconds
- The aft starboard thruster runs for 4 seconds
- The thrusters stop moving.
This should be a continuous sequence. The way that the ASV moves should be documented.
"""

import time
from API.Motors import t200

# Double-check to see if the hardcoded port /tty/ACM0 is correct
motors = t200.T200()

# Run each thruster moderately forward for 4 secs
motors.set_thrusters(0.5, 0, 0, 0)
time.sleep(4)
motors.set_thrusters(0, 0.5, 0, 0)
time.sleep(4)
motors.set_thrusters(0, 0, 0.5, 0)
time.sleep(4)
motors.set_thrusters(0, 0, 0, 0.5)
time.sleep(4)

motors.stop_thrusters()
