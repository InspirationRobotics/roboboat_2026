#!/usr/bin/env python3
"""
ROS2 A* Path Planner Node

Subscribes to:
  - /odom (nav_msgs/Odometry): Robot odometry
  - /local_costmap (nav_msgs/OccupancyGrid): Local costmap from LIDAR
  - /goal_pose (geometry_msgs/PoseStamped): Goal position

Publishes:
  - /planned_path (nav_msgs/Path): Generated path using A*
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import heapq
from typing import List, Tuple, Optional

from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class AStarPathfinder:
    """A* pathfinding algorithm"""
    
    def __init__(self, costmap: np.ndarray, resolution: float, threshold: int = 50):
        """
        Initialize pathfinder with costmap
        costmap: 2D numpy array with occupancy values (0-100)
        resolution: meters per cell
        threshold: occupancy threshold for obstacle (default 50)
        """
        self.costmap = costmap
        self.rows, self.cols = costmap.shape
        self.resolution = resolution
        self.threshold = threshold
        
        # Create binary grid (0 = free, 1 = obstacle)
        self.grid = (costmap > threshold).astype(int)
    
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring cells (8-directional)"""
        row, col = pos
        neighbors = []
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # cardinal
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # diagonal
        ]
        
        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if (0 <= new_row < self.rows and 
                0 <= new_col < self.cols and 
                self.grid[new_row][new_col] == 0):
                neighbors.append((new_row, new_col))
        
        return neighbors
    
    def get_move_cost(self, from_pos: Tuple[int, int], to_pos: Tuple[int, int]) -> float:
        """Calculate movement cost"""
        dr = abs(to_pos[0] - from_pos[0])
        dc = abs(to_pos[1] - from_pos[1])
        
        # Diagonal movement costs more
        base_cost = 1.414 if (dr + dc) == 2 else 1.0
        
        # Add cost based on proximity to obstacles
        cost_value = self.costmap[to_pos[0], to_pos[1]] / 100.0
        terrain_cost = 1.0 + cost_value * 0.5  # Scale cost influence
        
        return base_cost * terrain_cost
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """Find shortest path using A*"""
        # Validate start and goal
        if not self.is_valid(start) or not self.is_valid(goal):
            return None
        
        if self.grid[start[0], start[1]] == 1 or self.grid[goal[0], goal[1]] == 1:
            return None
        
        counter = 0
        open_set = [(0, counter, start)]
        counter += 1
        
        came_from = {}
        closed_set = set()
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        open_set_hash = {start}
        
        while open_set:
            _, _, current = heapq.heappop(open_set)
            open_set_hash.remove(current)
            
            if current == goal:
                return self._reconstruct_path(came_from, current)
            
            closed_set.add(current)
            
            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + self.get_move_cost(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    
                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
                        counter += 1
                        open_set_hash.add(neighbor)
        
        return None
    
    def is_valid(self, pos: Tuple[int, int]) -> bool:
        """Check if position is valid"""
        row, col = pos
        return 0 <= row < self.rows and 0 <= col < self.cols
    
    def _reconstruct_path(self, came_from, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstruct path from came_from dictionary"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


class AStarPlannerNode(Node):
    """ROS2 Node for A* path planning"""
    
    def __init__(self):
        super().__init__('astar_planner_node')
        
        # Parameters
        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('planning_frequency', 2.0)  # Hz
        
        self.occupancy_threshold = self.get_parameter('occupancy_threshold').value
        self.planning_frequency = self.get_parameter('planning_frequency').value
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/lidar_map',
            self.costmap_callback,
            qos_profile
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            qos_profile
        )
        
        # Publisher
        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            qos_profile
        )
        
        # State variables
        self.current_pose = None
        self.costmap = None
        self.costmap_info = None
        self.goal_pose = None
        
        # Timer for periodic planning
        self.planning_timer = self.create_timer(
            1.0 / self.planning_frequency,
            self.plan_callback
        )
        
        self.get_logger().info('A* Planner Node initialized')
        self.get_logger().info(f'Planning frequency: {self.planning_frequency} Hz')
        self.get_logger().info(f'Occupancy threshold: {self.occupancy_threshold}')
    
    def odom_callback(self, msg: Odometry):
        """Store current robot pose from odometry"""
        self.current_pose = msg.pose.pose
    
    def costmap_callback(self, msg: OccupancyGrid):
        """Store costmap data"""
        self.costmap_info = msg.info
        
        # Convert costmap to 2D numpy array
        width = msg.info.width
        height = msg.info.height
        
        # ROS costmap is row-major
        costmap_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # Handle unknown cells (-1) by treating them as free space
        costmap_data[costmap_data < 0] = 0
        
        self.costmap = costmap_data
    
    def goal_callback(self, msg: PoseStamped):
        """Store goal pose and trigger planning"""
        self.goal_pose = msg.pose
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        
        # Plan immediately when new goal received
        self.plan_path()
    
    def plan_callback(self):
        """Periodic planning callback"""
        if self.goal_pose is not None:
            self.plan_path()
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        if self.costmap_info is None:
            return None
        
        origin_x = self.costmap_info.origin.position.x
        origin_y = self.costmap_info.origin.position.y
        resolution = self.costmap_info.resolution
        
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        return (grid_y, grid_x)  # Note: (row, col) format
    
    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        if self.costmap_info is None:
            return None
        
        origin_x = self.costmap_info.origin.position.x
        origin_y = self.costmap_info.origin.position.y
        resolution = self.costmap_info.resolution
        
        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution
        
        return (x, y)
    
    def plan_path(self):
        """Plan path using A*"""
        # Check if we have all required data
        if self.current_pose is None:
            self.get_logger().warn('No odometry data received yet')
            return
        
        if self.costmap is None:
            self.get_logger().warn('No costmap data received yet')
            return
        
        if self.goal_pose is None:
            self.get_logger().warn('No goal pose set')
            return
        
        # Convert poses to grid coordinates
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        start_grid = self.world_to_grid(start_x, start_y)
        
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        goal_grid = self.world_to_grid(goal_x, goal_y)
        
        if start_grid is None or goal_grid is None:
            self.get_logger().error('Failed to convert coordinates to grid')
            return
        
        # Run A* pathfinding
        pathfinder = AStarPathfinder(
            self.costmap,
            self.costmap_info.resolution,
            self.occupancy_threshold
        )
        
        path_grid = pathfinder.find_path(start_grid, goal_grid)
        
        if path_grid is None:
            self.get_logger().warn('No path found to goal')
            return
        
        # Convert path to Path message
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        
        for grid_pos in path_grid:
            world_pos = self.grid_to_world(grid_pos[0], grid_pos[1])
            
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = world_pos[0]
            pose_stamped.pose.position.y = world_pos[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)
        
        # Publish path
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_grid)} waypoints')


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()