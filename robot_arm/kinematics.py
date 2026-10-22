from robot_arm.motor_config import (
    limitsById,
    MOTOR_1_CAN_ID,
    MOTOR_2_CAN_ID,
    MOTOR_3_CAN_ID,
)
import math
import numpy as np

LENGTH_L1_M = 0.19
LENGTH_L2_M = 0.19

OFFSET_S = 0
OFFSET_Z = 0

MAX_OP = math.sqrt(LENGTH_L1_M**2 + LENGTH_L2_M**2 - 2 * LENGTH_L1_M * LENGTH_L2_M * math.cos(math.radians(150)))
MIN_OP = math.sqrt(LENGTH_L1_M**2 + LENGTH_L2_M**2 - 2 * LENGTH_L1_M * LENGTH_L2_M * math.cos(math.radians(30)))


JOINT_LIMITS = [
    limitsById[MOTOR_1_CAN_ID]["position_rad"],
    limitsById[MOTOR_2_CAN_ID]["position_rad"],
    limitsById[MOTOR_3_CAN_ID]["position_rad"],
]

def isWithinJointLimits(angles):
    for angle, (lower, upper) in zip(angles, JOINT_LIMITS):
        if not lower <= angle <= upper:
            return False
    return True

def forwardKinematics(axis1Angle_rad, axis2Angle_rad, axis3Angle_rad):
    s = LENGTH_L1_M * math.cos(axis2Angle_rad) + LENGTH_L2_M * math.cos(-axis3Angle_rad) + OFFSET_S
    z = LENGTH_L1_M * math.sin(axis2Angle_rad) + LENGTH_L2_M * math.sin(-axis3Angle_rad) + OFFSET_Z
    x = s * math.sin(axis1Angle_rad)
    y = s * math.cos(axis1Angle_rad)

    return x, y, z

def inverseKinmatics(x, y, z):
    s = math.sqrt(x**2 + y**2) - OFFSET_S
    z = z - OFFSET_Z
   
    op = math.sqrt(s**2 + z**2)

    if op > MAX_OP:
        print("到達範囲外です")
        return None

    if op < MIN_OP:
        print("到達範囲外です")
        return None

    cosAlpha = (LENGTH_L1_M**2 + LENGTH_L2_M**2 - op**2) / (2 * LENGTH_L1_M * LENGTH_L2_M)
    cosBeta = (LENGTH_L1_M**2 + op**2 - LENGTH_L2_M**2) / (2 * LENGTH_L1_M * op)

    alpha = math.acos(cosAlpha)
    beta = math.acos(cosBeta)
    gamma = math.atan2(z, s)

    axis1Angle_rad = math.atan2(x, y)
    axis2Angle_rad = gamma + beta
    axis3Angle_rad = -(axis2Angle_rad - (math.pi - alpha))

    angles = (
        axis1Angle_rad,
        axis2Angle_rad,
        axis3Angle_rad,
    )

    if not isWithinJointLimits(angles):
        print("動作範囲外です")
        return None

    return angles


def jacobianTranspose(axis1Angle_rad, axis2Angle_rad, axis3Angle_rad):
    s = LENGTH_L1_M * math.cos(axis2Angle_rad) + LENGTH_L2_M * math.cos(-axis3Angle_rad) + OFFSET_S
    a = s * math.cos(axis1Angle_rad)
    b = -s * math.sin(axis1Angle_rad)
    c = 0
    d = -LENGTH_L1_M * math.sin(axis1Angle_rad) * math.sin(axis2Angle_rad)
    e = -LENGTH_L1_M * math.cos(axis1Angle_rad) * math.sin(axis2Angle_rad)
    f = LENGTH_L1_M * math.cos(axis2Angle_rad)
    g = -LENGTH_L2_M * math.sin(axis1Angle_rad) * math.sin(axis3Angle_rad)
    h = -LENGTH_L2_M * math.cos(axis1Angle_rad) * math.sin(axis3Angle_rad)
    i = -LENGTH_L2_M * math.cos(axis3Angle_rad)

    return np.array([
        [a, b, c],
        [d, e, f],
        [g, h, i],
    ], dtype=float)