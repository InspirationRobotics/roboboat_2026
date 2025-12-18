#!/usr/bin/env python3
"""
Simple Autonomous Navigation without PID

This script uses GPS data and basic motor commands to navigate the boat
to a desired waypoint. It rotates the boat to face the target and then
surges forward until the target is reached.
"""

import time
import math
import threading

from API.GPS.gps_api import GPS, GPSData
from GNC.Control_Core import motor_core

# ------------------------------
# Global Variable for GPS Data
# ------------------------------
current_gps = None
gps_lock = threading.Lock()

def gps_callback(data: GPSData):
    global current_gps
    with gps_lock:
        current_gps = data

def run_gps():
    gps = GPS('/dev/ttyUSB0', 115200, callback=gps_callback)
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        del gps

# ------------------------------
# Helper Functions
# ------------------------------

def haversine_distance(lat1, lon1, lat2, lon2):
    """Return distance in meters between two lat/lon points."""
    R = 6371000  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Return the bearing in degrees from point 1 to point 2."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)
    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(delta_lambda)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360

# ------------------------------
# Simple Navigation Logic
# ------------------------------
def simple_navigation(motors, target_lat, target_lon):
    # Set tolerance thresholds (adjust as needed)
    distance_threshold = 1.0  # meters
    heading_tolerance = 5.0   # degrees

    while True:
        with gps_lock:
            if current_gps is None or not current_gps.is_valid():
                continue
            current_lat = current_gps.lat
            current_lon = current_gps.lon
            current_heading = current_gps.heading

        # Calculate distance and desired bearing
        distance = haversine_distance(current_lat, current_lon, target_lat, target_lon)
        desired_bearing = calculate_bearing(current_lat, current_lon, target_lat, target_lon)
        heading_diff = (desired_bearing - current_heading + 360) % 360
        if heading_diff > 180:
            heading_diff -= 360  # Convert to range [-180, 180]

        print(f"Distance: {distance:.2f} m | Desired Bearing: {desired_bearing:.2f}° | Current Heading: {current_heading:.2f}° | Heading Diff: {heading_diff:.2f}°")

        if distance < distance_threshold:
            print("Target reached!")
            motors.stop()
            break

        # If the boat is not facing the target within the tolerance, rotate it
        if abs(heading_diff) > heading_tolerance:
            if heading_diff > 0:
                print("Rotating clockwise...")
                motors.rotate(0.3)  # Adjust rotation speed as needed
            else:
                print("Rotating counterclockwise...")
                motors.rotate(-0.3)
            time.sleep(0.5)
        else:
            print("Surging forward...")
            motors.surge(0.5)  # Adjust surge speed as needed
            time.sleep(0.5)

    motors.stop()

# ------------------------------
# Main Loop
# ------------------------------
def main():
    # Initialize MotorCore (assumes port "/dev/ttyACM0" is correct)
    motors = motor_core.MotorCore("/dev/ttyACM0")

    print("Waiting for initial GPS fix...")
    while True:
        with gps_lock:
            if current_gps is not None and current_gps.is_valid():
                current_lat = current_gps.lat
                current_lon = current_gps.lon
                break
        time.sleep(0.1)
    print(f"Initial GPS fix: lat={current_lat}, lon={current_lon}")

    # Get target waypoint from user
    print("Enter desired waypoint:")
    target_lat = float(input("  Latitude: "))
    target_lon = float(input("  Longitude: "))

    simple_navigation(motors, target_lat, target_lon)

if __name__ == "__main__":
    gps_thread = threading.Thread(target=run_gps, daemon=True)
    gps_thread.start()
    main()
