"""
To test ability to move in all directions (surge, sway, yaw), as well as conducting autonomous square maneuver.

This script is considered successful if all of the following keyboard inputs create the expected physical result:
- 'q' -> Quitting/ending the program
- 'w' -> Move forward
- 's' -> Move backward
- 'a' -> Strafe left (move laterally left)
- 'd' -> Strafe right (move laterally right)
- '0' -> Stop thrusters
- 'c' -> Yaw clockwise (rotate clockwise)
- 'z' -> Yaw counterclockwise (rotate counterclockwise)
- 'square' -> Execute square maneuver (forward, move right, backward, move left)
"""

import time
from GNC.Control_Core import motor_core

motors = motor_core.MotorCore("/dev/ttyACM0")


while(True):
    user_input = input("command>> ")
    if(str(user_input) == "q"):
        print("quiting the code")
        start = time.perf_counter_ns()
        motors.stop()
        end = time.perf_counter_ns()
        print(f"Takes {end-start} ns to stop")
        break
    elif(str(user_input) == "w"):
        start = time.perf_counter_ns()
        motors.surge(0.5)
        end = time.perf_counter_ns()
        print(f"elapsed time: {end-start} ns")
    elif (str(user_input) == "s"):
        start = time.perf_counter_ns()
        motors.surge(-0.5)
        end = time.perf_counter_ns()
        print(f"elapsed time: {end - start} ns")
    elif(str(user_input) =="a"):
        start = time.perf_counter_ns()
        motors.slide(0.5)
        end = time.perf_counter_ns()
        print(f"elapsed time: {end-start} ns")
    elif(str(user_input) =="d"):
        start = time.perf_counter_ns()
        motors.slide(-0.5)
        end = time.perf_counter_ns()
        print(f"elapsed time: {end - start} ns")
    elif(str(user_input) =="0"):
        start = time.perf_counter_ns()
        motors.stay()
        end = time.perf_counter_ns()
        print(f"elapsed time: {end - start} ns")
    elif(str(user_input) =="c"):
        # Clockwise
        start = time.perf_counter_ns()
        motors.rotate(0.5)
        end = time.perf_counter_ns()
        print(f"elapsed time: {end - start} ns")
    elif(str(user_input) =="z"):
        # Counterclockwise
        start = time.perf_counter_ns()
        motors.rotate(-0.5)
        end = time.perf_counter_ns()
        print(f"elapsed time: {end - start} ns")
    elif(str(user_input) =="square"):
        def wait():
            print("Stay for 2 s")
            motors.stay()
            time.sleep(2)

        start = time.perf_counter_ns()
        motors.surge(0.3)
        time.sleep(4)
        end = time.perf_counter_ns()
        print(f"Takes {time.perf_counter_ns()} ns to go forward")

        wait()

        start = time.perf_counter_ns()
        motors.slide(-0.3)
        time.sleep(4)
        end = time.perf_counter_ns()
        print(f"Takes {time.perf_counter_ns()} ns to slide right")

        wait()

        start = time.perf_counter_ns()
        motors.surge(-0.3)
        time.sleep(4)
        end = time.perf_counter_ns()
        print(f"Takes {time.perf_counter_ns()} ns to go backward")

        wait()

        start = time.perf_counter_ns()
        motors.slide(0.3)
        time.sleep(4)
        end = time.perf_counter_ns()
        print(f"Takes {time.perf_counter_ns()} ns to slide left")

        wait()

        print("Finished the square")