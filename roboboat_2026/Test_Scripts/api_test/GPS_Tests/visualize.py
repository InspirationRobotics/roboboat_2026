import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

# ----- 1. Hard-code the background image and geographic bounds -----
image_path = '/home/chaser/Downloads/abby_bound.png'
img = Image.open(image_path)
img = np.array(img)
imgHeight, imgWidth, _ = img.shape

# Define geographic boundaries (latitudes and longitudes)
latlim = [32.924198, 32.923938]  # [max, min]
lonlim = [-117.039175, -117.037906]  # [min, max]

# Pre-compute the grid for the background image (if needed for other uses)
x, y = np.meshgrid(np.linspace(lonlim[0], lonlim[1], imgWidth),
                   np.linspace(latlim[0], latlim[1], imgHeight))

# ----- 2. Read and parse the GPS log file with labels -----
gps_file = '/home/chaser/Documents/RoboBoat_2025/Test_Scripts/API_Tests/GPS_Tests/GPSLogs/GPSlog_1740346367.txt'
data = []
with open(gps_file, 'r') as f:
    for line in f:
        line = line.strip()
        if not line:
            continue  # skip empty lines
        # Expected line format: "Lat: 32.9238897117, Lon: -117.0381234567, Heading: 45.0"
        parts = line.split(', ')
        try:
            lat = float(parts[0].split(': ')[1])
            lon = float(parts[1].split(': ')[1])
            heading = float(parts[2].split(': ')[1])
            data.append([lat, lon, heading])
        except Exception as e:
            print(f"Error parsing line: '{line}'. Error: {e}")

# Convert the parsed data to a NumPy array
data = np.array(data)

# ----- 3. Plotting each GPS point with its heading vector -----
# Define a scale for the arrow length (1/10th of the longitude span)
arrowScale = (lonlim[1] - lonlim[0]) / 10

numPoints = data.shape[0]
for i in range(numPoints):
    currentLat, currentLon, heading = data[i, :]
    
    # Calculate vector components for the arrow:
    # Assuming heading is in degrees clockwise from north:
    dLat = arrowScale * np.cos(np.radians(heading))
    dLon = arrowScale * np.sin(np.radians(heading))
    
    # Create a new figure
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Display the background image with georeferenced coordinates
    ax.imshow(img, extent=[lonlim[0], lonlim[1], latlim[0], latlim[1]])
    
    # Plot the current point (red circle)
    ax.scatter(currentLon, currentLat, c='r', s=30, label='Point')
    
    # Overlay the heading vector using quiver (blue arrow)
    ax.quiver(currentLon, currentLat, dLon, dLat, angles='xy', scale_units='xy', scale=1, color='b', label='Heading')
    
    # Add labels and title
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_title(f'Point {i + 1}: Heading {heading:.2f}°')
    ax.legend()
    
    # Save the figure as a PNG image
    filename = f'point_{i + 1}.png'
    plt.savefig(filename)
    plt.close(fig)

print("Processing complete. Images saved.")
