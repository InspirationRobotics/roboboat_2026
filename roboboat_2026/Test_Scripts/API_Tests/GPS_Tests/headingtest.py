from API.GPS.gps_api import GPS

gps = GPS(serialport='/dev/ttyUSB0',offset=0)

gps.calibrate_heading_offset()
