"""
File to handle the visualization of GPS data. 
Remember that all files are run as modules: run "python3 -m API.GPS.gps_visualizer" in the RoboBoat_2025 root directory to get good results.
"""
# NOTE: This file stil needs to be documented.

# Smopy eeds to be imported onto host computer. See Setup/install_smopy.sh to install smopy individually, 
# if you haven't already setup all necessary dependencies, or just run "pip install smopy" in terminal.
import smopy
import cv2
import numpy as np
from pathlib import Path
from API.GPS.waypoint_data_parser import GPSDataParser
from datetime import datetime
from typing import Union

class GPSVisualizer:
    """
    Class to visualize GPS data.

    Args:
        file_path (str) : Path directory to GPS data file to visualize.
        zoom (int, kwarg): Zoom magnitude for the Smopy-based image. Defaults to 19.
        playback_speed (int, kwarg): Speed to play visualization video at. Defaults to 10 (10x speed of normal).
        frame_size (int, kwarg): Side length of frame in pixels (since frame is a square), defaults to 600.
        heading_offset (float, kwarg): Heading offset of the GPS to take into account in visualizer, defaults to 0.
    """

    
    def __init__(self, file_path: Union[Path, str], *, zoom: int = 19, playback_speed: int = 10, frame_size: int = 600, heading_offset: float = 0):
            self.target = {}
            self.position = {}
            self.heading = {}
    
            dp = GPSDataParser(file_path)
            self.position, self.heading = dp.parse_data()
            
            self.playback_speed = playback_speed
            self.zoom = zoom
            self.frame_size = frame_size
            self.heading_offset = heading_offset
            self.get_map()
        
    def get_map(self, save : bool = False):
        lat, lon = self.position[next(iter(self.position))]
        zoom = self.zoom
        map = smopy.Map((lat, lon, lat, lon), z=zoom)
        if save:
            map.save_png("map.png")
        self.map_obj = map
        frame = cv2.cvtColor(map.to_numpy(), cv2.COLOR_RGB2BGR)
        frame = cv2.resize(frame, (self.frame_size, self.frame_size), interpolation=cv2.INTER_LINEAR)
        # sharpen the frame
        frame = cv2.GaussianBlur(frame, (0, 0), 1.0)
        frame = cv2.addWeighted(frame, 1.5, frame, -0.5, 0)
        kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        frame = cv2.filter2D(frame, -1, kernel)
        self.frame = frame
        return map
    
    def convert_heading_to_arrow(self, heading : float, size : int = 10):
        # Assume top of image is north
        # Heading is in degrees, we need to create an arrow offset of x and y ratio
        # We will assume the arrow is 10 pixels long
        x = size * np.sin(np.radians(heading))
        y = size * np.cos(np.radians(heading))
        return int(x), int(-y)
    
    def rescale(self, x : int, y : int):
        size_w = self.map_obj.w
        size_h = self.map_obj.h
        x = int(x * self.frame_size / size_w)
        y = int(y * self.frame_size / size_h)
        return x, y

    def draw_test(self):
        x, y = self.map_obj.to_pixels(self.position[next(iter(self.position))])
        # scale the x,y to the frame size
        x, y = self.rescale(x, y)
        frame = self.frame.copy()
        frame = cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("map", frame)
        cv2.waitKey(0)

    def draw(self):
        for ts in self.position:
            x, y = self.map_obj.to_pixels(self.position[ts])
            x, y = self.rescale(x, y)
            # create a copy of the frame
            frame = self.frame.copy()
            # draw the red point underneath the blue point and on the saved frame
            self.frame = cv2.circle(self.frame, (x, y), 1, (0, 0, 255), -1)
            # draw the blue point on top of the frame
            frame = cv2.circle(frame, (x, y), 2, (255, 0, 0), -1)
            # draw the green heading arrow
            heading = self.heading.get(ts, None)
            if heading is not None:
                dx, dy = self.convert_heading_to_arrow(heading + self.heading_offset)
                x1, y1 = x + dx, y + dy
                frame = cv2.arrowedLine(frame, (x, y), (x1, y1), (0, 0, 0), 2, tipLength=0.5)
            # draw the green point for the target
            if ts in self.target.keys():
                x, y = self.map_obj.to_pixels(self.target[ts])
                x, y = self.rescale(x, y)
                frame = cv2.circle(frame, (x, y), 5, (6, 64, 43), -1)
            # print the time on the frame
            stamp = datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
            cv2.putText(frame, stamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            # print the speed on the frame
            speed = f'Playback Speed: {self.playback_speed}x'
            cv2.putText(frame, speed, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            # show the frame
            cv2.imshow("map", frame)
            key = cv2.waitKey(int(1000/self.playback_speed)) & 0xFF
            if  key == ord('q'):
                break
            elif key == ord(' '):
                cv2.waitKey(0)
            elif key == ord('.'):
                if self.playback_speed < 1000:
                    self.playback_speed *= 10
            elif key == ord(','):
                if self.playback_speed > 1:
                    self.playback_speed /= 10
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Test for GPSVisualizer.
    file_path = r'/Users/brandontran3/GPS_log.txt'
    visualizer = GPSVisualizer(file_path, playback_speed=5, frame_size=900, zoom=17)
    visualizer.draw()
