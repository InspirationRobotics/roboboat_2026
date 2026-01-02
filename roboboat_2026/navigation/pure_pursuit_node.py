import math
class PurePursuitPlanner:
    def __init__(self, lookahead_dist, wheelbase):
        self.ld = lookahead_dist
        self.L = wheelbase
        self.path = []

    def set_path(self, path):
        self.path = path

    def get_target_setpoints(self, current_pose):
        # Use internal self.path and self.ld
        target_pt = self._find_lookahead_point((current_pose.x, current_pose.y), self.path, self.ld)
        
        dx = target_pt[0] - current_pose.x
        dy = target_pt[1] - current_pose.y
        target_heading = math.degrees(math.atan2(dy, dx))

        target_velocity = 0.5 
        return target_velocity, target_heading

    def _find_lookahead_point(self, robot_pose, path, lookahead_dist):
        # robot_pose is (x, y), path is [(x1, y1), (x2, y2), ...]
        target_point = None
        
        # Iterate through segments
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i+1]
            
            # Find intersection of circle (center=robot, radius=Ld) and line segment (p1 to p2)
            intersect = self._get_segment_intersection(p1, p2, robot_pose, lookahead_dist)
            
            if intersect:
                # If multiple segments intersect, we take the one further along the path
                target_point = intersect
                
        # Fallback: If no intersection found, return the last point in the path 
        # or the closest point to prevent the robot from stopping.
        return target_point if target_point else path[-1]

    def _get_segment_intersection(self, p1, p2, center, r):
        # Shift coordinates so the circle is at the origin (0,0)
        d = (p2[0] - p1[0], p2[1] - p1[1])
        f = (p1[0] - center[0], p1[1] - center[1])
        
        # Quadratic formula coefficients: a*t^2 + b*t + c = 0
        a = d[0]**2 + d[1]**2
        b = 2 * (f[0] * d[0] + f[1] * d[1])
        c = (f[0]**2 + f[1]**2) - r**2
        
        discriminant = b**2 - 4*a*c
        
        if discriminant < 0:
            return None # No intersection
        
        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)
        
        # We want t to be between 0 and 1 (on the segment)
        # We prioritize t2 because it is the "further" intersection on the segment
        for t in [t2, t1]:
            if 0 <= t <= 1:
                return (p1[0] + t * d[0], p1[1] + t * d[1])
                
        return None