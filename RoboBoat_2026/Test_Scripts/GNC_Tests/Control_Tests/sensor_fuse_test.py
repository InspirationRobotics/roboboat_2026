"""
Logging test to make sure GPS data through SensorFuse interface is valid.
This test is considred successful if a text file is found in the directory "Test_Scripts/GNC_Tests/Control_Tests/GPSLogs" with
data in the form of "{'Position': ([POSITION]), 'Heading': ([HEADING])}"
"""

from GNC.Control_Core.sensor_fuse import SensorFuse
import time
import numpy as np

def log_gps():
    log = open(f'Test_Scripts/GNC_Tests/Control_Tests/GPSLogs/GPSlog_{int(time.time())}.txt', "w")

    def callback(data):
        log.write(str(data) + '\n')

    sf = SensorFuse(enable_filter=False)
    print("Waiting...")
    time.sleep(5)

    print("Started GPS log with Sensor Fuse interface.")

    while True:
        try:
            lat, lon = sf.get_position()
            heading = sf.get_heading()
            velocity = sf.get_velocity()
            dictionary = {"Position" : (lat, lon), "Heading" : heading}
            callback()
            # print(heading)
            # print(velocity)
            time.sleep(0.4)
        except KeyboardInterrupt:
            break

    del sf
    log.close()

log_gps()

