"""
Utility functions for SafeWay
"""

import numpy as np
from typing import Tuple


def normalize_angle(angle: float) -> float:
    """Normalize angle to [0, 2*pi]"""
    while angle < 0:
        angle += 2 * np.pi
    while angle >= 2 * np.pi:
        angle -= 2 * np.pi
    return angle


def angle_difference(angle1: float, angle2: float) -> float:
    """Calculate smallest difference between two angles"""
    diff = angle2 - angle1
    while diff > np.pi:
        diff -= 2 * np.pi
    while diff < -np.pi:
        diff += 2 * np.pi
    return diff


def distance(pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
    """Calculate Euclidean distance between two positions"""
    return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp value between min and max"""
    return max(min_val, min(value, max_val))

