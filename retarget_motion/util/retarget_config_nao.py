import numpy as np

URDF_FILENAME = "E:/my_data/MachineLearning/Python_Code/Bair_paper_code/robot_data/nao.urdf"

MOTOR_NAMES = [
    "HeadYaw",
    "HeadPitch",
    "LHipYawPitch",
    "LHipRoll",
    "LHipPitch",
    "LKneePitch",
    "LAnklePitch",
    "LAnkleRoll",
    "RHipYawPitch",
    "RHipRoll",
    "RHipPitch",
    "RKneePitch",
    "RAnklePitch",
    "RAnkleRoll",
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw",
    "LHand",
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw",
    "RHand"
]

REF_POS_SCALE = 1

SIM_TOE_JOINT_IDS = [
    60, # left hand
    43, # left foot
    31, # right hand
    18 # right foot
]
SIM_MID_JOINT_IDS = [58, 41, 29, 16]
SIM_HIP_JOINT_IDS = [56, 39, 27, 14]
SIM_TOE_MID_JOINT_IDS = [60, 58, 43, 41, 31, 29, 18, 16]
SIM_ROOT_OFFSET = np.array([0, -0.5, -0.115])
SIM_TOE_OFFSET_LOCAL = [
    np.array([0, 0.0, -0.36]),
    np.array([0, 0.0, -0.36]),
    np.array([0, 0.0, -0.36]),
    np.array([0, 0.0, -0.36])
]
SIM_MID_OFFSET_LOCAL = [
    np.array([0, 0.0, -0.36]),
    np.array([0, 0.0, -0.36]),
    np.array([0, 0.0, -0.36]),
    np.array([0, 0.0, -0.36])
]

DEFAULT_JOINT_POSE = np.array([0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0,
                               0, 0, 0, 0, -0,
                               0, 0, 0, 0, 0,
                               -0, 0, 0, 0, 0, 0])  # 26 joints
JOINT_DAMPING = [0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01, 0.5, 0.05]


FORWARD_DIR_OFFSET = np.array([0, 0, 0.015])
