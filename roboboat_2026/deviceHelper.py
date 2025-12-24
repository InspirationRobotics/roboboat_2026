"""
To get the configuration of the devices connected to the boat(thrusters, cameras, etc.)
"""

import os # For interacting with the operating system
import sys # For system-specific variables and functionalities
import platform # For platform-specific functionalities
import json

def load_json(path):
    """Load JSON data from a file and return it, given its file path"""
    # Open the file, load the file
    with open(path, "r") as f:
        data = json.load(f)
    return data

variables = None
file_dir = os.path.dirname(os.path.abspath(__file__)) # Obtain the file directory path of the current script

# [NOTE] ask on what header we should be using to discern the boats
# [NOTE] PLANNED: If "jetson-desktop" is in the platform node name, Barco Polo is the boat, else Charlie
# Load the configuration of Barco Polo/Charlie
hostname = platform.node()
print(f"Hostname is: {hostname}")
variables = load_json(f"{file_dir}/config/roboboat.json")
    
def findFromId(ids):
    """
    Finds USB serial device paths (/dev/tty*) by matching platform paths from JSON config 
    against usbLink.sh output. Ignores volatile port numbers (ACM0, USB0, etc.).
    
    Args:
        ids: List of platform paths from JSON like ["/devices/.../tty/ttyACM0"]
    
    Returns:
        str: First matching device path (e.g., "/dev/ttyACM0") or None if not found
    """
    print(f"Starting findFromId for {ids}")
    
    # Execute usbLink.sh to get current USB device mapping (format: "DEVICE - PLATFORM_PATH")
    bash = os.popen("bash /root/rb_ws/src/roboboat_2026/roboboat_2026/usbLink.sh").read()
    bashSplit = bash.split("\n")  # Split into individual device lines
    result = []  # Will hold matched device paths, one per input id

    # decoding loop - matches up to /tty/ttyACM (excludes volatile port number)
    for index, platform_id in enumerate(ids):
        result.append("")  # placeholder for this id - will be replaced if found

        # Scan each line from usbLink.sh output
        for line in bashSplit:
            # Skip non-tty lines (cameras, etc. handled by findCam)
            if "/dev/tty" not in line:
                continue
            
            # Remove any trailing newline or carriage return chars from the line
            line = line.strip()
            
            # Parse line format: "/dev/ttyACM0 - /devices/platform/.../tty/ttyACM0"
            parts = line.split(" - ", 1)  # Split once on first " - "
            if len(parts) != 2:  # Skip malformed lines
                continue

            dev_path, dev_platform = parts[0], parts[1]  # dev_path="/dev/ttyACM0"

            # Strip volatile port number from both config and usbLink.sh output
            # config:  "/devices/.../1-2.4:1.0/tty/ttyACM0"  -> "/devices/.../1-2.4:1.0/tty/ttyACM"
            config_base = platform_id
            # actual: "/devices/.../1-2.4:1.0/tty/ttyACM5"  -> "/devices/.../1-2.4:1.0/tty/ttyACM"
            actual_base = dev_platform.rsplit('/', 2)[0]
            actual_base_full = actual_base[:-3]
            # DEBUGGING STEP: line temporarily to see exact values
            print(f"Comparing: '{config_base}' == '{actual_base_full}'") 
            
            # Exact match on stable hardware topology (ignores ACM0 vs ACM5)
            if config_base == actual_base_full:
                result[index] = dev_path
                break  # Found match for this device, move to next id

    # Clean up: remove empty entries (devices not found)
    # Note: reversed() prevents index shifting during removal
    for i in reversed(result):
        if i == "":
            result.remove(i)
    
    # No devices found - show full usbLink.sh output for debugging
    if len(result) == 0:
        print(bash)
        print("Device not found, above is list of all available devices")
        return None
    
    # Return first (and typically only) matched device
    print(f"Find port {result[0]}")
    return result[0]

# def findCam(ids):
#     """To find cameras based on their IDs"""
#     # Read the list of USB devices, split into lines at "\n"
#     # NOTE you may want to change this path next year
#     # [NOTE] this should be changed from manual change to automatic change by year and input, through a config file
#     bash = os.popen("bash /root/rb_ws/src/roboboat_2026/roboboat_2026/usbLink.sh").read() # Read usbLink.sh
#     bash = bash.split("\n")
#     result = []
#     # For each ID, go over each line, and if the device is written to "dev/video", which means it's a camera,
#     # then split the line at the "-".
#     for id in enumerate(ids):
#         result.append("")
#         minVal = 100
#         for line in bash:
#             if "/dev/video" in line:
#                 line = line.split(" - ")
#                 # If the ID paths correspond
#                 if id[1] == line[1]:
#                     # Extract the numerical value from the device path
#                     val = int(line[0][10:])
#                     # If the value is less than the current minimum value, then set the name of the device to the device name
#                     # This is important to make sure we get the camera with the lowest numerical value (default) 
#                     if val < minVal:
#                         minVal = val
#                         result[id[0]] = line[0]

#     # Remove any empty strings
#     for i in reversed(result):
#         if i == "":
#             result.remove(i)
#     return result


def dataFromConfig(name):
    """Obtain the configuration or resolved device path for a device by name."""
    device_cfg = variables.get(name)
    if device_cfg is None:
        raise Exception("Invalid Name")

    # Serial devices with a 'port' in the JSON
    if name in ("teensy", "ball_launcher", "gps"):
        platform_path = device_cfg.get("port")
        if platform_path is None:
            print("id not found")
            return None
        print(f"Platform path for {name}: {platform_path}")
        # Resolve to /dev/tty* using findFromId
        return findFromId([platform_path])

    # Oak-D: return its ID (findCam or depthai will use this elsewhere)
    if name == "oakd_lr":
        cam_id = device_cfg.get("id")
        if cam_id is None:
            print("id not found")
            return None
        return cam_id

    # USB webcam: return its platform 'port' (likely consumed by findCam or similar)
    if name == "web_came":
        return device_cfg.get("port")

    # Fallback: return raw dict
    return device_cfg

if __name__ == "__main__":
    """
    When running the script directly, get the configuration of the device 
    using a command line argument that is the name of the device
    """
    print(dataFromConfig("teensy"))
    # print(dataFromConfig(sys.argv[1]))