import numpy as np
import math


def motion_update(pose, odom):
    x, y, theta = pose
    dx, dy, dtheta = odom

    x += dx * math.cos(theta)
    y += dx * math.sin(theta)
    theta += dtheta

    return (x, y, theta)


def measurement_update(pose, landmarks, measurements):
    # Simple nearest landmark correction
    x, y, theta = pose

    for (lx, ly), dist in zip(landmarks, measurements):
        expected = math.hypot(lx - x, ly - y)
        error = dist - expected

        # crude correction
        x += 0.1 * error
        y += 0.1 * error

    return (x, y, theta)