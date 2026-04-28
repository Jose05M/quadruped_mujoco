import numpy as np

class WaypointTracker:
    def __init__(self, waypoints, threshold=0.15, speed=0.12):
        self.waypoints = waypoints
        self.idx = 0
        self.threshold = threshold
        self.speed = speed

    def get_reference(self, x):
        pos = x[0:3]
        target = self.waypoints[self.idx]

        error = target - pos
        dist = np.linalg.norm(error)

        if dist < self.threshold and self.idx < len(self.waypoints) - 1:
            self.idx += 1
            target = self.waypoints[self.idx]
            error = target - pos

        max_dist = 0.1
        error = np.clip(error, -max_dist, max_dist)

        direction = error / (np.linalg.norm(error) + 1e-6)

        alpha = 0.05
        desired_pos = pos + alpha * error

        x_ref = np.zeros(12)
        x_ref[0:3] = desired_pos
        x_ref[3:6] = direction * self.speed

        return x_ref