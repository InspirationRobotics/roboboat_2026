"""
This is a motor direction test. It will run each of the motors at a slow forward speed for 1.5 seconds each.
It is meant to test the t200 API's functionality. This is a BENCH test.

The test is considered successful when:
- The forward port thruster runs for 5 seconds
- The forward starboard thruster runs for 5 seconds
- The aft port thruster runs for 5 seconds
- The aft starboard thruster runs for 5 seconds
- The thrusters stop moving.
This should be a continuous sequence.
"""

import time
from API.Motors import t200

# Double-check to see if the hardcoded port /tty/ACM0 is correct
motors = t200.T200()

# Run each thruster slowly forward for 5 secs
motors.set_thrusters(0.1, 0, 0, 0)
time.sleep(5)
motors.set_thrusters(0, 0.1, 0, 0)
time.sleep(5)
motors.set_thrusters(0, 0, 0.1, 0)
time.sleep(5)
motors.set_thrusters(0, 0, 0, 0.1)
time.sleep(5)

motors.stop_thrusters()