"""
File to handle the parsing of GPS data from the .txt files created by the save_waypoints method in gps_api.py.
The class GPSDataParser (defined below) is meant to be called in gps_visualizer.py in order to turn the GPS data into easily visualizable 
waypoints over time.
"""

from pathlib import Path
from typing import Union

class GPSDataParser:
    """
    Class to handle the parsing of GPS data. 

    Args:
        file_path (Path(str)): A Path object that takes in a string. That string is the relative file path from the directory of the execution.
    """

    def __init__(self, file_path : Union[Path, str]):
        if isinstance(file_path, str):
            file_path = Path(file_path)
        self.file_path = file_path
        self.position = {}
        self.heading = {}

    def parse_data(self):
        """
        Parses the data from the GPS.

        The data will be in the form of
        "{rounded_time} % {data.lat} % {data.lon} % {data.heading}\n"

        Returns:
            self.position (dict): Position (lat, lon) of GPS, with the key being the time stamp.
            self.heading (dict): Heading of the GPS, with the key being the time stamp.
        """

        with open(self.file_path, 'r') as file:
            for line in file:
                line = line.strip()
                # Parse the data by " %", with unlimited number of splits.
                time_stamp, latitude, longitude, heading = line.split(" %", -1)
                time_stamp = int(time_stamp)

                # Converting latitude, longitude, heading to floats in order for calculations in GPS vizualizer.
                self.position[time_stamp] = tuple([float(latitude), float(longitude)])
                self.heading[time_stamp] = float(heading)
        return self.position, self.heading

if __name__ == "__main__":
    file_path = Path(r'Test_Scripts/GPS_Tests/missions/GPS_Parser_Test.txt')
    data_parser = GPSDataParser(file_path)
    data_parser.parse_data()
    print(data_parser.heading)
