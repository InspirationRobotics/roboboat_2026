"""
Code to test whether we can save GPS data (latitude, longitude, heading), in a given path.

Test is considered successful if there is a log (.txt) file saved to the directory RoboBoat_2025/Test_Scripts/GPS_tests/missions with GPS data.
The data should be in the form of: [{latitude} % {longitude} % {heading}].
"""

from API.GPS.gps_api import GPS, GPSData

gps = GPS('/dev/ttyUSB0', 115200, offset=270.96788823529414)
gps.save_waypoints()
