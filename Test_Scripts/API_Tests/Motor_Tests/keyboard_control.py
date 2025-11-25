import time
from API.Motors import t200
motors = t200.T200(port="COM10",debug=False)

while(True):
    user_input = input("command>> ")
    if(str(user_input) == "q"):
        print("quiting the code")
        motors.stop_thrusters()
        break
    elif(str(user_input) == "1"):
        start = time.perf_counter_ns()
        motors.set_thrusters(0.2, 0, 0, 0)
        end = time.perf_counter_ns()
        print(f"elapsed time: {end-start} ns")
    elif(str(user_input) == "2"):
        start = time.perf_counter_ns()
        motors.set_thrusters(0, 0.2, 0, 0)
        end = time.perf_counter_ns()
        print(f"elapsed time: {end - start} ns")
    elif(str(user_input) == "3"):
        start = time.perf_counter_ns()
        motors.set_thrusters(0, 0, 0.2, 0)
        end = time.perf_counter_ns()
        print(f"elapsed time: {end - start} ns")
    elif(str(user_input) == "4"):
        start = time.perf_counter_ns()
        motors.set_thrusters(0, 0, 0, 0.2)
        end = time.perf_counter_ns()
        print(f"elapsed time: {end - start} ns")
    elif(str(user_input) == "0"):
        start = time.perf_counter_ns()
        motors.set_thrusters(0, 0, 0, 0)
        end = time.perf_counter_ns()
        print(f"elapsed time: {end - start} ns")


