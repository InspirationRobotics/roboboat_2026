#!/usr/bin/env python3
"""
Path Planner Node with A* Algorithm
Generates paths using A* on occupancy grid or predefined patterns
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import yaml
from queue import PriorityQueue


class AStarPlanner:
    """A* path planning algorithm"""
    
    def __init__(self, occupancy_grid, resolution, origin):
        """
        occupancy_grid: 2D numpy array (0=free, 100=occupied, -1=unknown)
        resolution: meters per cell
        origin: (x, y) position of grid origin in meters
        """
        self.grid = occupancy_grid
        self.resolution = resolution
        self.origin = origin
        self.height, self.width = occupancy_grid.shape
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """Convert grid coordinates to world coordinates"""
        x = gx * self.resolution + self.origin[0]
        y = gy * self.resolution + self.origin[1]
        return x, y
    
    def is_valid(self, gx, gy):
        """Check if grid cell is valid and free"""
        if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
            return False
        return self.grid[gy, gx] < 50  # Consider cells < 50 as free
    
    def heuristic(self, a, b):
        """Euclidean distance heuristic"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(self, node):
        """Get 8-connected neighbors"""
        gx, gy = node
        neighbors = []
        
        # 8-connected grid
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nx, ny = gx + dx, gy + dy
            if self.is_valid(nx, ny):
                # Diagonal moves cost more
                cost = np.sqrt(2) if dx != 0 and dy != 0 else 1.0
                neighbors.append(((nx, ny), cost))
        
        return neighbors
    
    def plan(self, start_x, start_y, goal_x, goal_y):
        """
        A* path planning
        Returns: list of (x, y) waypoints in world coordinates, or None if no path
        """
        start = self.world_to_grid(start_x, start_y)
        goal = self.world_to_grid(goal_x, goal_y)
        
        if not self.is_valid(start[0], start[1]):
            return None
        if not self.is_valid(goal[0], goal[1]):
            return None
        
        # Priority queue: (f_score, counter, node)
        open_set = PriorityQueue()
        counter = 0
        open_set.put((0, counter, start))
        counter += 1
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        open_set_hash = {start}
        
        while not open_set.empty():
            current = open_set.get()[2]
            open_set_hash.remove(current)
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    x, y = self.grid_to_world(current[0], current[1])
                    path.append([x, y])
                    current = came_from[current]
                x, y = self.grid_to_world(start[0], start[1])
                path.append([x, y])
                path.reverse()
                return path
            
            for neighbor, cost in self.get_neighbors(current):
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    
                    if neighbor not in open_set_hash:
                        open_set.put((f_score[neighbor], counter, neighbor))
                        counter += 1
                        open_set_hash.add(neighbor)
        
        return None  # No path found


class PathPlannerNode(Node):
    """Generates or loads a reference path for the vehicle to follow"""
    
    def __init__(self):
        super().__init__('path_planner_node')
        
        # Parameters
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('loop_path', False)
        self.declare_parameter('path_type', 'astar')  # 'astar', 'figure8', 'circle', 'straight', 'square'
        self.declare_parameter('path_resolution', 0.5)  # meters between waypoints
        
        waypoint_file = self.get_parameter('waypoint_file').value
        self.loop_path = self.get_parameter('loop_path').value
        self.path_type = self.get_parameter('path_type').value
        self.path_resolution = self.get_parameter('path_resolution').value
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/fused/odometry',
            self.odom_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/local_map',
            self.map_callback,
            10
        )
        
        # Publisher
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        
        # State
        self.current_pose = None
        self.goal_pose = None
        self.occupancy_grid = None
        self.map_resolution = None
        self.map_origin = None
        self.waypoints = None
        self.path_generated = False
        self.astar_planner = None
        
        # Timer to republish path periodically
        self.timer = self.create_timer(2.0, self.publish_path)
        
        # Load or generate static path
        if self.path_type not in ['astar', 'goal']:
            if waypoint_file:
                self.waypoints = self.load_waypoints_from_file(waypoint_file)
            else:
                self.waypoints = self.generate_path(self.path_type)
            self.path_generated = True
        
        self.get_logger().info('Path Planner Node Started')
        if self.waypoints:
            self.get_logger().info(f'Generated {self.path_type} path with {len(self.waypoints)} waypoints')
        else:
            self.get_logger().info(f'Path type: {self.path_type}')
            if self.path_type == 'astar':
                self.get_logger().info('Waiting for local map and goal pose...')
            else:
                self.get_logger().info('Waiting for goal pose...')
        self.get_logger().info(f'Loop path: {self.loop_path}')
    
    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose
        
        # If waiting for goal and have required data, plan path
        if not self.path_generated and self.goal_pose is not None:
            if self.path_type == 'astar':
                if self.astar_planner is not None:
                    self.plan_path_astar()
            else:
                self.plan_path_to_goal()
    
    def goal_callback(self, msg):
        """Receive goal pose and plan path"""
        self.goal_pose = msg.pose
        self.path_generated = False
        
        self.get_logger().info(f'Received goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        
        # If we have current pose, plan immediately
        if self.current_pose is not None:
            if self.path_type == 'astar':
                if self.astar_planner is not None:
                    self.plan_path_astar()
                else:
                    self.get_logger().warn('Waiting for local map to plan with A*')
            else:
                self.plan_path_to_goal()
    
    def map_callback(self, msg):
        """Receive occupancy grid map"""
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        # Convert flat array to 2D grid
        grid_data = np.array(msg.data).reshape((height, width))
        
        self.occupancy_grid = grid_data
        self.map_resolution = resolution
        self.map_origin = (origin_x, origin_y)
        
        # Create A* planner
        self.astar_planner = AStarPlanner(grid_data, resolution, (origin_x, origin_y))
        
        self.get_logger().info(f'Received map: {width}x{height}, resolution={resolution}m')
        
        # If waiting to plan with A*, do it now
        if self.path_type == 'astar' and not self.path_generated:
            if self.current_pose is not None and self.goal_pose is not None:
                self.plan_path_astar()
    
    def plan_path_astar(self):
        """Plan path using A* algorithm"""
        if self.current_pose is None or self.goal_pose is None or self.astar_planner is None:
            return
        
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        
        self.get_logger().info(f'Planning A* path from ({start_x:.2f}, {start_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})')
        
        # Plan with A*
        path = self.astar_planner.plan(start_x, start_y, goal_x, goal_y)
        
        if path is None:
            self.get_logger().error('A* failed to find a path!')
            return
        
        # Optionally smooth/downsample path
        self.waypoints = self.smooth_path(path)
        self.path_generated = True
        
        self.get_logger().info(f'A* planned path with {len(self.waypoints)} waypoints')
        
        # Publish immediately
        self.publish_path()
    
    def smooth_path(self, path):
        """Downsample path to achieve desired resolution"""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        accumulated_dist = 0.0
        
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            dist = np.sqrt(dx**2 + dy**2)
            accumulated_dist += dist
            
            if accumulated_dist >= self.path_resolution:
                smoothed.append(path[i])
                accumulated_dist = 0.0
        
        # Always include the last point
        if smoothed[-1] != path[-1]:
            smoothed.append(path[-1])
        
        return smoothed
    
    def plan_path_to_goal(self):
        """Plan a straight-line path from current pose to goal pose"""
        if self.current_pose is None or self.goal_pose is None:
            return
        
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        
        # Simple straight line path
        distance = np.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
        num_points = max(2, int(distance / self.path_resolution))
        
        waypoints = []
        for i in range(num_points + 1):
            t = i / num_points
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)
            waypoints.append([x, y])
        
        self.waypoints = waypoints
        self.path_generated = True
        
        self.get_logger().info(f'Planned straight path to goal with {len(waypoints)} waypoints')
        
        # Publish immediately
        self.publish_path()
    
    def generate_path(self, path_type):
        """Generate different test paths"""
        if path_type == 'figure8':
            return self.generate_figure8()
        elif path_type == 'circle':
            return self.generate_circle()
        elif path_type == 'straight':
            return self.generate_straight()
        elif path_type == 'square':
            return self.generate_square()
        else:
            self.get_logger().warn(f'Unknown path type: {path_type}, using figure8')
            return self.generate_figure8()
    
    def generate_figure8(self):
        """Generate a figure-8 path"""
        waypoints = []
        t = np.linspace(0, 2 * np.pi, 50)
        scale = 20.0
        
        for i in range(len(t)):
            x = scale * np.sin(t[i])
            y = scale * np.sin(t[i]) * np.cos(t[i])
            waypoints.append([x, y])
        
        return waypoints
    
    def generate_circle(self):
        """Generate a circular path"""
        waypoints = []
        t = np.linspace(0, 2 * np.pi, 40)
        radius = 15.0
        
        for i in range(len(t)):
            x = radius * np.cos(t[i])
            y = radius * np.sin(t[i])
            waypoints.append([x, y])
        
        return waypoints
    
    def generate_straight(self):
        """Generate a straight line path"""
        waypoints = []
        for i in range(50):
            x = i * 1.0
            y = 0.0
            waypoints.append([x, y])
        
        return waypoints
    
    def generate_square(self):
        """Generate a square path"""
        waypoints = []
        side_length = 30.0
        points_per_side = 10
        
        # Bottom side (left to right)
        for i in range(points_per_side):
            waypoints.append([i * side_length / points_per_side, 0.0])
        
        # Right side (bottom to top)
        for i in range(points_per_side):
            waypoints.append([side_length, i * side_length / points_per_side])
        
        # Top side (right to left)
        for i in range(points_per_side):
            waypoints.append([side_length - i * side_length / points_per_side, side_length])
        
        # Left side (top to bottom)
        for i in range(points_per_side):
            waypoints.append([0.0, side_length - i * side_length / points_per_side])
        
        return waypoints
    
    def load_waypoints_from_file(self, filename):
        """Load waypoints from YAML file
        
        Expected format:
        waypoints:
          - [x1, y1]
          - [x2, y2]
          ...
        """
        try:
            with open(filename, 'r') as f:
                data = yaml.safe_load(f)
                waypoints = data['waypoints']
                self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {filename}')
                return waypoints
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return self.generate_figure8()  # Fallback to default
    
    def publish_path(self):
        """Publish the path as a nav_msgs/Path message"""
        if self.waypoints is None or len(self.waypoints) == 0:
            return
        
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        waypoints_to_publish = self.waypoints.copy()
        
        # If loop path, add first point at the end
        if self.loop_path and len(waypoints_to_publish) > 0:
            waypoints_to_publish.append(waypoints_to_publish[0])
        
        for wp in waypoints_to_publish:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()