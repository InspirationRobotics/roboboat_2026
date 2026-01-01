import math

"""
Test for the odometry conversion from GPS (lat, lon) to local XY coordinates.

Results of test:
Origin: (37.7749, -122.4194)
Current: (37.775, -122.4193)
Resulting Local Coordinates: X: 8.79 meters, Y: 11.12 meters
"""

def latlon_to_xy_meters(lat, lon, lat0, lon0):
    R = 6371000
    phi = math.radians(lat)
    lmd = math.radians(lon)
    phi0 = math.radians(lat0)
    lmd0 = math.radians(lon0)
    
    x = R * math.cos(phi0) * (lmd - lmd0)
    y = R * (phi - phi0)
    return x, y

# Simulation
origin = (37.7749, -122.4194)
current = (37.7750, -122.4193)

x, y = latlon_to_xy_meters(current[0], current[1], origin[0], origin[1])

print(f"Origin: {origin}")
print(f"Current: {current}")
print(f"Resulting Local Coordinates: X: {x:.2f} meters, Y: {y:.2f} meters")