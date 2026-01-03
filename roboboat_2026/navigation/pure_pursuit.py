import math
class PurePursuitPlanner:
    def __init__(self, lookahead_dist, wheelbase):
        self.ld = lookahead_dist
        self.L = wheelbase
        self.path = []
        self.last_index = 0  # Track progress to prevent "skipping"

    def get_target_setpoints(self, current_pose):
        if not self.path:
            return 0, 0

        target_pt = self._find_lookahead_point((current_pose.x, current_pose.y))
        
        dx = target_pt[0] - current_pose.x
        dy = target_pt[1] - current_pose.y
        
        # Heading in radians is usually better for PID math
        target_heading = math.atan2(dy, dx)

        # Basic slowdown logic: slower when heading error is high
        # You'll need to normalize current_pose.heading to match atan2 range
        heading_error = abs(target_heading - math.radians(current_pose.theta))
        target_velocity = 0.5 * math.cos(min(heading_error, math.pi/2))
        
        return target_velocity, math.degrees(target_heading)

    def _find_lookahead_point(self, robot_pose):
        # Start searching from the last known segment
        for i in range(self.last_index, len(self.path) - 1):
            p1 = self.path[i]
            p2 = self.path[i+1]
            intersect = self._get_segment_intersection(p1, p2, robot_pose, self.ld)
            
            if intersect:
                self.last_index = i # Update progress
                return intersect
                
        return self.path[-1]

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