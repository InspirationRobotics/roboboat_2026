"""
Various mathematical functions
"""

import math


def heading_error(heading, target):
    """
    Calculate signed heading error between current and target heading.

    Args:
        heading (float): Current heading in degrees [0, 360)
        target (float): Target heading in degrees [0, 360)

    Returns:
        float: Signed heading error in degrees, range [-180, 180]
    """
    error = (target - heading + 180) % 360 - 180
    return error



def get_norm(x, y):
    """
    Calculate Euclidean norm(distance) of a vectior

    Args:
        x (float): X-component of the vector
        y (float): Y-component of the vector
    
    Returns:
        float: Euclidean norm of the vector
    """
    norm = math.sqrt(x**2 + y**2)
    return norm

def get_distance(v1, v2):
    """
    Calculate the Euclidean distance between two points

    Args:
        v1 (tuple): Coordinates of the first point (x1, y1)
        v2 (tuple): Coordinates of the second point (x2, y2)
    
    Returns:
        float: Euclidean distance between the two points
    """
    dist = math.sqrt((v1[0] - v2[0])**2 + (v1[1] - v2[1])**2)
    return dist

def rotate_vector(x, y, heading):
    """
    Rotate a vector by a given heading in degrees (counter-clockwise).

    Args:
        x (float): X-component of the vector
        y (float): Y-component of the vector
        heading (float): Angle in degrees to rotate the vector by
    
    Returns:
        tuple: Rotated vector components (x_rot, y_rot)
    """
    theta = math.radians(heading)
    x_rot = x * math.cos(theta) - y * math.sin(theta)
    y_rot = x * math.sin(theta) + y * math.cos(theta)
    return x_rot, y_rot

def inv_rotate_vector(x, y, heading):
    """
    Rotate a vector by the *negative* of the given heading (i.e., inverse rotation)
    in the clockwise direction.

    Args:
        x (float): X-component of the vector
        y (float): Y-component of the vector
        heading (float): Angle in degrees to inverse-rotate the vector by
    
    Returns:
        tuple: Rotated vector components (x_rot, y_rot)
    """
    theta = math.radians(heading)
    x_rot = x * math.cos(theta) + y * math.sin(theta)
    y_rot = -x * math.sin(theta) + y * math.cos(theta)
    return x_rot, y_rot

def get_heading_from_coords(x, y):
    """
    Get heading (angle) from given coordinates
    Assumes the Y-axis is North and the X-axis is East
    
    Parameters:
        x (float): X-coordinate
        y (float): Y-coordinate
    
    Returns:
        float: Heading angle in degrees
    """
    return math.degrees(math.atan2(x, y))

def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the great-circle distance between two points
    on the Earth (specified in decimal degrees).
    Returns distance in meters.
    """
    # Convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(a ** 0.5)  # Faster than math.sqrt(a)
    r = 6371000  # Radius of Earth in meters (use 3956 for miles)
    
    return c * r  # Distance in meters