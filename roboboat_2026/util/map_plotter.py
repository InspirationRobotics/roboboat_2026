import tkinter as tk
import tkintermapview
from pyproj import Transformer

class WaypointMapGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GPS Waypoint Visualizer (NED)")

        self.map_widget = tkintermapview.TkinterMapView(
            root, width=900, height=600, corner_radius=0
        )
        self.map_widget.pack(fill="both", expand=True)

        # Initial map position (can be anything)
        self.map_widget.set_position(32.9159147, -117.0990487)  # San Diego example
        self.map_widget.set_zoom(18)

        self.reference_lat = None
        self.reference_lon = None
        self.transformer = None

        self.waypoints = []

        self.map_widget.add_left_click_map_command(self.on_click)

    def setup_transformer(self):
        self.transformer = Transformer.from_crs(
            "epsg:4326",
            f"+proj=aeqd +lat_0={self.reference_lat} "
            f"+lon_0={self.reference_lon} +datum=WGS84",
            always_xy=True
        )

    def on_click(self, coords):
        # Support both tuple and Coordinate object
        if isinstance(coords, tuple):
            lat, lon = coords
        else:
            lat = coords.latitude
            lon = coords.longitude

        # First click → define reference (0,0)
        if self.reference_lat is None:
            self.reference_lat = lat
            self.reference_lon = lon
            self.setup_transformer()

            self.map_widget.set_marker(lat, lon, text="START (0,0)")
            print("Reference set at (0,0)")
            return

        # Convert to NED
        e, n = self.transformer.transform(lon, lat)
        d = 0.0

        self.waypoints.append((n, e, d))
        self.map_widget.set_marker(lat, lon, text=str(len(self.waypoints)))

        print(
            f"Waypoint {len(self.waypoints)} → "
            f"NED: N={n:.2f} m, E={e:.2f} m, D={d:.2f}"
        )


if __name__ == "__main__":
    root = tk.Tk()
    app = WaypointMapGUI(root)
    root.mainloop()
