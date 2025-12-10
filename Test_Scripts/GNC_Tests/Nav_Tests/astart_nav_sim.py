import heapq
import cv2
import numpy as np
from typing import List, Tuple, Set, Dict, Optional
from scipy.interpolate import CubicSpline

class AStarPathfinder:
    """A* pathfinding algorithm with cost map and priority queue"""
    
    def __init__(self, grid: np.ndarray, cost_map: Optional[np.ndarray] = None):
        self.grid = grid
        self.rows, self.cols = grid.shape
        
        if cost_map is None:
            self.cost_map = np.ones_like(grid, dtype=float)
        else:
            self.cost_map = cost_map
    
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring cells (8-directional movement)"""
        row, col = pos
        neighbors = []
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]
        
        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if (0 <= new_row < self.rows and 
                0 <= new_col < self.cols and 
                self.grid[new_row][new_col] == 0):
                neighbors.append((new_row, new_col))
        
        return neighbors
    
    def get_move_cost(self, from_pos: Tuple[int, int], to_pos: Tuple[int, int]) -> float:
        """Calculate movement cost considering terrain cost and diagonal movement"""
        # we can add some robotcs kinematics here, like awarding forward/yaw motion, and give less to sway and backward
        dr = abs(to_pos[0] - from_pos[0])
        dc = abs(to_pos[1] - from_pos[1])
        base_cost = 1.414 if (dr + dc) == 2 else 1.0
        terrain_cost = self.cost_map[to_pos[0], to_pos[1]]
        return base_cost * terrain_cost
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """Find shortest path using A* with priority queue"""
        counter = 0
        open_set = [(0, counter, start)]
        counter += 1
        
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        closed_set: Set[Tuple[int, int]] = set()
        g_score: Dict[Tuple[int, int], float] = {start: 0}
        f_score: Dict[Tuple[int, int], float] = {start: self.heuristic(start, goal)}
        open_set_hash: Set[Tuple[int, int]] = {start}
        
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
    
    def _reconstruct_path(self, came_from: Dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstruct path from came_from dictionary"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


class LidarSensor:
    """Simulated LIDAR sensor for obstacle detection"""
    
    def __init__(self, max_range=100, num_rays=360, fov=360):
        self.max_range = max_range
        self.num_rays = num_rays
        self.fov = np.radians(fov)  # Field of view in radians
        
    def scan(self, robot_x, robot_y, robot_theta, ground_truth_grid, cell_size):
        """
        Perform LIDAR scan and return detected obstacles
        Returns list of (x, y) points in world coordinates
        """
        detections = []
        angle_step = self.fov / self.num_rays
        start_angle = robot_theta - self.fov / 2
        
        grid_rows, grid_cols = ground_truth_grid.shape
        
        for i in range(self.num_rays):
            angle = start_angle + i * angle_step
            
            # Cast ray
            for dist in range(1, int(self.max_range), 2):
                # Calculate point along ray
                x = robot_x + dist * np.cos(angle)
                y = robot_y + dist * np.sin(angle)
                
                # Convert to grid coordinates
                grid_col = int(x / cell_size)
                grid_row = int(y / cell_size)
                
                # Check if out of bounds
                if grid_row < 0 or grid_row >= grid_rows or grid_col < 0 or grid_col >= grid_cols:
                    break
                
                # Check if obstacle detected
                if ground_truth_grid[grid_row, grid_col] == 1:
                    detections.append((x, y))
                    break
        
        return detections


class LocalMap:
    """Local occupancy grid map built from LIDAR scans"""
    
    def __init__(self, grid_size, cell_size):
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.grid = np.zeros((grid_size, grid_size), dtype=int)  # 0 = unknown, 1 = obstacle, 2 = free
        self.cost_map = np.ones((grid_size, grid_size), dtype=float)
        
    def update(self, robot_x, robot_y, lidar_detections):
        """Update local map with new LIDAR scan"""
        robot_grid = self.pixel_to_grid((robot_x, robot_y))
        
        # Mark robot position as free
        if self.is_valid_grid(robot_grid):
            self.grid[robot_grid] = 2
        
        # Process LIDAR detections
        for det_x, det_y in lidar_detections:
            det_grid = self.pixel_to_grid((det_x, det_y))
            
            if self.is_valid_grid(det_grid):
                # Mark obstacle
                self.grid[det_grid] = 1
                
                # Inflate obstacles (safety margin)
                self._inflate_obstacle(det_grid, radius=1)
        
        # Mark cells along rays as free
        for det_x, det_y in lidar_detections:
            self._mark_ray_free(robot_x, robot_y, det_x, det_y)
    
    def _inflate_obstacle(self, grid_pos, radius=1):
        """Inflate obstacle for safety margin"""
        row, col = grid_pos
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                new_row, new_col = row + dr, col + dc
                if self.is_valid_grid((new_row, new_col)):
                    if self.grid[new_row, new_col] != 1:  # Don't overwrite existing obstacles
                        self.grid[new_row, new_col] = 1
    
    def _mark_ray_free(self, robot_x, robot_y, det_x, det_y):
        """Mark cells along ray as free space"""
        # Bresenham-like algorithm
        steps = int(np.sqrt((det_x - robot_x)**2 + (det_y - robot_y)**2) / self.cell_size)
        
        for i in range(steps):
            t = i / max(steps, 1)
            x = robot_x + t * (det_x - robot_x)
            y = robot_y + t * (det_y - robot_y)
            
            grid_pos = self.pixel_to_grid((x, y))
            if self.is_valid_grid(grid_pos):
                if self.grid[grid_pos] == 0:  # Only mark unknown cells as free
                    self.grid[grid_pos] = 2
    
    def pixel_to_grid(self, pixel_pos):
        """Convert pixel coordinates to grid coordinates"""
        x, y = pixel_pos
        return (int(y / self.cell_size), int(x / self.cell_size))
    
    def is_valid_grid(self, grid_pos):
        """Check if grid position is valid"""
        row, col = grid_pos
        return 0 <= row < self.grid_size and 0 <= col < self.grid_size
    
    def get_planning_grid(self):
        """Get grid for A* planning (1 = obstacle, 0 = free/unknown)"""
        planning_grid = np.zeros_like(self.grid)
        planning_grid[self.grid == 1] = 1  # Obstacles
        return planning_grid


class RobotSimulator:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
        
        self.vx = 0
        self.vy = 0
        self.omega = 0
        
        self.mass = 10.0
        self.inertia = 2.0
        self.drag_linear = 2.0
        self.drag_angular = 1.5
        
        self.surge_factor = 50.0
        self.sway_factor = 50.0
        self.yaw_factor = 20.0
        
        self.max_pwm = 1.0
        self.min_pwm = -1.0
        
    def apply_pwm(self, surge_pwm, sway_pwm, yaw_pwm):
        surge_pwm = np.clip(surge_pwm, self.min_pwm, self.max_pwm)
        sway_pwm = np.clip(sway_pwm, self.min_pwm, self.max_pwm)
        yaw_pwm = np.clip(yaw_pwm, self.min_pwm, self.max_pwm)
        
        force_surge = surge_pwm * self.surge_factor
        force_sway = sway_pwm * self.sway_factor
        torque_yaw = yaw_pwm * self.yaw_factor
        
        return force_surge, force_sway, torque_yaw
    
    def update(self, surge_pwm, sway_pwm, yaw_pwm, dt):
        f_surge, f_sway, torque = self.apply_pwm(surge_pwm, sway_pwm, yaw_pwm)
        
        drag_surge = -self.drag_linear * self.vx
        drag_sway = -self.drag_linear * self.vy
        drag_yaw = -self.drag_angular * self.omega
        
        total_surge = f_surge + drag_surge
        total_sway = f_sway + drag_sway
        total_torque = torque + drag_yaw
        
        ax = total_surge / self.mass
        ay = total_sway / self.mass
        alpha = total_torque / self.inertia
        
        self.vx += ax * dt
        self.vy += ay * dt
        self.omega += alpha * dt
        
        vx_world = self.vx * np.cos(self.theta) - self.vy * np.sin(self.theta)
        vy_world = self.vx * np.sin(self.theta) + self.vy * np.cos(self.theta)
        
        self.x += vx_world * dt
        self.y += vy_world * dt
        self.theta += self.omega * dt
        
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))


class PurePursuitController:
    def __init__(self, lookahead_distance=30.0):
        self.lookahead = lookahead_distance
        self.waypoints = []
        self.current_waypoint_idx = 0
        
    def set_waypoints(self, waypoints):
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        
    def get_target_point(self, robot_x, robot_y):
        if not self.waypoints or self.current_waypoint_idx >= len(self.waypoints) - 1:
            return self.waypoints[-1] if self.waypoints else (robot_x, robot_y)
        
        target = self.waypoints[self.current_waypoint_idx]
        
        for i in range(self.current_waypoint_idx, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            dist = np.sqrt((wx - robot_x)**2 + (wy - robot_y)**2)
            
            if dist < 15 and i < len(self.waypoints) - 1:
                self.current_waypoint_idx = i + 1
            
            if dist >= self.lookahead:
                target = (wx, wy)
                break
        
        wx_ss, wy_ss = self.waypoints[-1]
        dist_ss = np.sqrt((wx_ss - robot_x)**2 + (wy_ss - robot_y)**2)
        if dist_ss < 40:
            target = self.waypoints[-1]

        return target
    
    def compute_control(self, robot):
        if not self.waypoints:
            return 0, 0, 0
        
        target_x, target_y = self.get_target_point(robot.x, robot.y)
        
        dx = target_x - robot.x
        dy = target_y - robot.y
        
        dx_body = dx * np.cos(-robot.theta) - dy * np.sin(-robot.theta)
        dy_body = dx * np.sin(-robot.theta) + dy * np.cos(-robot.theta)
        
        dist = np.sqrt(dx_body**2 + dy_body**2)
        desired_angle = np.arctan2(dy_body, dx_body)
        
        surge_pwm = 0.6 if dist > 10 else 0.3
        sway_pwm = np.clip(dy_body * 0.01, -0.3, 0.3)
        yaw_pwm = np.clip(desired_angle * 0.5, -0.5, 0.5)
        
        return surge_pwm, sway_pwm, yaw_pwm


def create_test_environment():
    """Create ground truth grid with obstacles (robot doesn't know this)"""
    grid_size = 60
    cell_size = 10
    
    grid = np.zeros((grid_size, grid_size), dtype=int)
    cost_map = np.ones((grid_size, grid_size), dtype=float)
    
    # Add obstacles (unknown to robot initially)
    # Vertical walls
    grid[15:45, 20] = 1
    grid[15:45, 40] = 1
    
    # Horizontal obstacles
    grid[25, 10:20] = 1
    grid[25, 40:50] = 1
    grid[35, 25:35] = 1
    
    # Random obstacles
    np.random.seed(42)
    for _ in range(40):
        i, j = np.random.randint(0, grid_size, 2)
        if grid[i, j] == 0:
            grid[i, j] = 1
    
    return grid, cost_map, cell_size


def grid_to_pixel(grid_pos, cell_size):
    """Convert grid coordinates to pixel coordinates"""
    row, col = grid_pos
    return (col * cell_size + cell_size // 2, row * cell_size + cell_size // 2)


def pixel_to_grid(pixel_pos, cell_size):
    """Convert pixel coordinates to grid coordinates"""
    x, y = pixel_pos
    return (int(y // cell_size), int(x // cell_size))


def smooth_path(path_grid, cell_size, num_points=200):
    """Smooth A* path using cubic spline interpolation"""
    if len(path_grid) < 3:
        return [grid_to_pixel(p, cell_size) for p in path_grid]
    
    path_pixels = [grid_to_pixel(p, cell_size) for p in path_grid]
    
    x_points = np.array([p[0] for p in path_pixels])
    y_points = np.array([p[1] for p in path_pixels])
    
    t = np.linspace(0, 1, len(path_pixels))
    
    cs_x = CubicSpline(t, x_points, bc_type='natural')
    cs_y = CubicSpline(t, y_points, bc_type='natural')
    
    t_fine = np.linspace(0, 1, num_points)
    x_fine = cs_x(t_fine)
    y_fine = cs_y(t_fine)
    
    return [(x_fine[i], y_fine[i]) for i in range(len(t_fine))]


def visualize(robot, controller, waypoints, local_map, ground_truth_grid, cell_size, 
              trajectory, lidar_detections, replan_counter):
    """Combined visualization showing local map and robot view"""
    grid_size = ground_truth_grid.shape[0]
    width = height = grid_size * cell_size
    
    # Create two side-by-side views
    local_view = np.ones((height, width, 3), dtype=np.uint8) * 255
    global_view = np.ones((height, width, 3), dtype=np.uint8) * 255
    
    # Draw LOCAL MAP (what robot knows)
    for i in range(grid_size):
        for j in range(grid_size):
            x1, y1 = j * cell_size, i * cell_size
            x2, y2 = x1 + cell_size, y1 + cell_size
            
            if local_map.grid[i, j] == 1:  # Known obstacle
                cv2.rectangle(local_view, (x1, y1), (x2, y2), (50, 50, 50), -1)
            elif local_map.grid[i, j] == 2:  # Known free
                cv2.rectangle(local_view, (x1, y1), (x2, y2), (240, 240, 240), -1)
            else:  # Unknown
                cv2.rectangle(local_view, (x1, y1), (x2, y2), (180, 180, 180), -1)
            
            cv2.rectangle(local_view, (x1, y1), (x2, y2), (200, 200, 200), 1)
    
    # Draw GROUND TRUTH (for comparison)
    for i in range(grid_size):
        for j in range(grid_size):
            x1, y1 = j * cell_size, i * cell_size
            x2, y2 = x1 + cell_size, y1 + cell_size
            
            if ground_truth_grid[i, j] == 1:  # Obstacle
                cv2.rectangle(global_view, (x1, y1), (x2, y2), (50, 50, 50), -1)
            
            cv2.rectangle(global_view, (x1, y1), (x2, y2), (200, 200, 200), 1)
    
    # Draw LIDAR detections on both views
    for det_x, det_y in lidar_detections:
        cv2.circle(local_view, (int(det_x), int(det_y)), 2, (0, 0, 255), -1)
        cv2.circle(global_view, (int(det_x), int(det_y)), 2, (0, 0, 255), -1)
    
    # Draw LIDAR range circle
    cv2.circle(local_view, (int(robot.x), int(robot.y)), 100, (200, 200, 0), 1)
    cv2.circle(global_view, (int(robot.x), int(robot.y)), 100, (200, 200, 0), 1)
    
    # Draw waypoints path
    if waypoints:
        for i in range(len(waypoints) - 1):
            pt1 = (int(waypoints[i][0]), int(waypoints[i][1]))
            pt2 = (int(waypoints[i+1][0]), int(waypoints[i+1][1]))
            cv2.line(local_view, pt1, pt2, (0, 200, 200), 2)
            cv2.line(global_view, pt1, pt2, (0, 200, 200), 2)
    
    # Draw trajectory
    for i in range(len(trajectory) - 1):
        cv2.line(local_view, trajectory[i], trajectory[i+1], (255, 0, 255), 2)
        cv2.line(global_view, trajectory[i], trajectory[i+1], (255, 0, 255), 2)
    
    # Draw current target
    if waypoints:
        target = controller.get_target_point(robot.x, robot.y)
        cv2.circle(local_view, (int(target[0]), int(target[1])), 8, (0, 255, 0), 2)
        cv2.circle(global_view, (int(target[0]), int(target[1])), 8, (0, 255, 0), 2)
    
    # Draw robot
    robot_pos = (int(robot.x), int(robot.y))
    cv2.circle(local_view, robot_pos, 8, (255, 0, 0), -1)
    cv2.circle(global_view, robot_pos, 8, (255, 0, 0), -1)
    
    # Draw heading
    heading_len = 15
    end_x = int(robot.x + heading_len * np.cos(robot.theta))
    end_y = int(robot.y + heading_len * np.sin(robot.theta))
    cv2.arrowedLine(local_view, robot_pos, (end_x, end_y), (0, 0, 255), 2)
    cv2.arrowedLine(global_view, robot_pos, (end_x, end_y), (0, 0, 255), 2)
    
    # Add labels
    cv2.putText(local_view, "LOCAL MAP (Robot's View)", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    cv2.putText(global_view, "GROUND TRUTH", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    
    # Add info
    info_text = [
        f"Pos: ({robot.x:.1f}, {robot.y:.1f})",
        f"Heading: {np.degrees(robot.theta):.1f} deg",
        f"Replans: {replan_counter}",
        f"Waypoints: {len(waypoints) if waypoints else 0}"
    ]
    
    for i, text in enumerate(info_text):
        cv2.putText(local_view, text, (10, 45 + i * 18), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    
    # Combine views
    combined = np.hstack([local_view, global_view])
    
    return combined


def main():
    print("=" * 70)
    print("A* PATH PLANNING + PURE PURSUIT with LIDAR-BASED REPLANNING")
    print("=" * 70)
    print("\nFeatures:")
    print("  - Robot has NO global map knowledge")
    print("  - LIDAR sensor scans environment (100 pixel range)")
    print("  - Local map built from LIDAR data")
    print("  - Dynamic replanning when new obstacles detected")
    print("\nInitializing environment...")
    
    # Create environment (robot doesn't know this!)
    ground_truth_grid, cost_map, cell_size = create_test_environment()
    grid_size = ground_truth_grid.shape[0]
    
    # Define start and goal
    start_grid = (5, 5)
    goal_grid = (55, 55)
    
    # Initialize robot systems
    start_pixel = grid_to_pixel(start_grid, cell_size)
    goal_pixel = grid_to_pixel(goal_grid, cell_size)
    
    robot = RobotSimulator(x=start_pixel[0], y=start_pixel[1], theta=0)
    lidar = LidarSensor(max_range=100, num_rays=360)
    local_map = LocalMap(grid_size, cell_size)
    controller = PurePursuitController(lookahead_distance=30.0)
    
    # Initial state
    waypoints = []
    trajectory = []
    replan_counter = 0
    steps_since_replan = 0
    replan_interval = 20  # Replan every N steps
    
    dt = 0.05
    
    print(f"✓ Environment created: {grid_size}x{grid_size} grid")
    print(f"✓ Start: {start_grid}, Goal: {goal_grid}")
    print(f"✓ LIDAR range: 100 pixels, 360 rays")
    print("\n" + "=" * 70)
    print("SIMULATION STARTED")
    print("=" * 70)
    print("Controls: 'q' - Quit | 'r' - Reset")
    print("=" * 70)
    
    while True:
        # LIDAR scan
        lidar_detections = lidar.scan(robot.x, robot.y, robot.theta, 
                                      ground_truth_grid, cell_size)
        
        # Update local map
        local_map.update(robot.x, robot.y, lidar_detections)
        
        # Replan periodically or if no path exists
        if steps_since_replan >= replan_interval or not waypoints:
            current_grid = pixel_to_grid((robot.x, robot.y), cell_size)
            planning_grid = local_map.get_planning_grid()
            
            # Make sure start and goal are free
            if local_map.is_valid_grid(current_grid):
                planning_grid[current_grid[0], current_grid[1]] = 0
            if local_map.is_valid_grid(goal_grid):
                planning_grid[goal_grid[0], goal_grid[1]] = 0
            
            # Run A* on local map
            pathfinder = AStarPathfinder(planning_grid, local_map.cost_map)
            path_grid = pathfinder.find_path(current_grid, goal_grid)
            
            if path_grid:
                waypoints = smooth_path(path_grid, cell_size, num_points=200)
                controller.set_waypoints(waypoints)
                replan_counter += 1
                print(f"✓ Replanned path #{replan_counter}: {len(waypoints)} waypoints")
            else:
                print(f"✗ No path found in local map!")
                waypoints = [goal_pixel]  # Try to reach goal directly
                controller.set_waypoints(waypoints)
            
            steps_since_replan = 0
        
        # Compute control
        surge_pwm, sway_pwm, yaw_pwm = controller.compute_control(robot)
        
        # Update robot
        robot.update(surge_pwm, sway_pwm, yaw_pwm, dt)
        
        # Record trajectory
        trajectory.append((int(robot.x), int(robot.y)))
        if len(trajectory) > 1000:
            trajectory.pop(0)
        
        steps_since_replan += 1
        
        # Visualize
        img = visualize(robot, controller, waypoints, local_map, ground_truth_grid, 
                       cell_size, trajectory, lidar_detections, replan_counter)
        
        cv2.imshow("LIDAR-based Navigation", img)
        
        # Check goal
        dist_to_goal = np.sqrt((robot.x - goal_pixel[0])**2 + 
                              (robot.y - goal_pixel[1])**2)
        
        if dist_to_goal < 15:
            cv2.putText(img, "GOAL REACHED!", (400, 300), 
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4)
            cv2.imshow("LIDAR-based Navigation", img)
            print("\n✓ GOAL REACHED!")
            print(f"Total replans: {replan_counter}")
            cv2.waitKey(3000)
            break
        
        # Handle keyboard
        key = cv2.waitKey(int(dt * 1000)) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            robot = RobotSimulator(x=start_pixel[0], y=start_pixel[1], theta=0)
            local_map = LocalMap(grid_size, cell_size)
            controller.current_waypoint_idx = 0
            trajectory = []
            waypoints = []
            replan_counter = 0
            steps_since_replan = 0
    
    cv2.destroyAllWindows()
    print("\n✓ Simulation ended")


if __name__ == "__main__":
    main()