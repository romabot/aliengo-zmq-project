"""Small math helpers shared by simulator and controller."""

import numpy as np

from src.common.constants import THIGH_LENGTH, CALF_LENGTH


def quat_to_euler_xyz(q):
    w, x, y, z = q

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_xyz_to_quat(roll, pitch, yaw):
    cr = np.cos(0.5 * roll)
    sr = np.sin(0.5 * roll)
    cp = np.cos(0.5 * pitch)
    sp = np.sin(0.5 * pitch)
    cy = np.cos(0.5 * yaw)
    sy = np.sin(0.5 * yaw)

    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def inverse_kinematics(x, y):
    d2 = x * x + y * y
    a1 = THIGH_LENGTH
    a2 = CALF_LENGTH
    cos2 = (d2 - a1 * a1 - a2 * a2) / (2 * a1 * a2)
    cos2 = np.clip(cos2, -1.0, 1.0)

    theta2 = -np.arccos(cos2)
    theta1 = np.arctan2(x, y) + np.arctan2(a2 * np.sin(-theta2), a1 + a2 * np.cos(-theta2))
    return theta1, theta2


def smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)
