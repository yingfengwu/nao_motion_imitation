"""
@author: yingfengwu
time: 2021/06/26
"""

import time
import numpy as np

from pybullet_utils import transformations
import pybullet
import pybullet_data as pd
from qibullet import SimulationManager
import pandas as pds

import util.retarget_config_nao as config
from util.read_txt import read_txt_file

POS_SIZE = 3
ROT_SIZE = 4
DEFAULT_ROT = np.array([0, 0, 0, 1])
FORWARD_DIR = np.array([1, 0, 0])

GROUND_URDF_FILENAME = "plane_implicit.urdf"

# reference motion
REF_COORD_ROT = transformations.quaternion_from_euler(0.5 * np.pi, 0, 0)
REF_POS_OFFSET = np.array([0, 0, 0])
REF_ROOT_ROT = transformations.quaternion_from_euler(0, 0, 0.5 * np.pi)

REF_jOINT_NUM = 15
REF_HIP_JOINT_ID = 0
REF_NECK_JOINT_ID = 7
REF_HEAD_JOINT_ID = 14
REF_HIP_JOINT_IDS = [1, 4, 8, 11]
REF_MID_JOINT_IDS = [2, 5, 9, 12]
REF_TOE_JOINT_IDS = [3, 6, 10, 13]
lock = 0


def QuaternionRotatePoint(point, quat):
    q_point = np.array([point[0], point[1], point[2], 0.0])
    quat_inverse = transformations.quaternion_inverse(quat)
    q_point_rotated = transformations.quaternion_multiply(
      transformations.quaternion_multiply(quat, q_point), quat_inverse)
    return q_point_rotated[:3]


def calc_heading(q):
    """
    Returns the heading of a rotation q, specified as a quaternion.
    The heading represents the rotational component of q along the vertical
    axis (z axis).

    Args:
    q: A quaternion that the heading is to be computed from.

    Returns:
    An angle representing the rotation about the z axis.
    """
    ref_dir = np.array([1, 0, 0])
    rot_dir = QuaternionRotatePoint(ref_dir, q)
    heading = np.arctan2(rot_dir[1], rot_dir[0])
    return heading


def calc_heading_rot(q):
    """
    Return a quaternion representing the heading rotation of q along the vertical axis (z axis).
    Args:
      q: A quaternion that the heading is to be computed from.

    Returns:
      A quaternion representing the rotation about the z axis.
    """
    heading = calc_heading(q)
    q_heading = transformations.quaternion_about_axis(heading, [0, 0, 1])
    return q_heading


def build_markers(num_markers):
    marker_radius = 0.02

    markers = []
    for i in range(num_markers):
        if (i == REF_NECK_JOINT_ID) or (i == REF_HIP_JOINT_ID) or (i in REF_HIP_JOINT_IDS):
            col = [0, 0, 1, 1]  # blue
        elif (i in REF_TOE_JOINT_IDS) or (i == REF_HEAD_JOINT_ID):
            col = [1, 0, 0, 1]  # red
        else:
            col = [0, 1, 0, 1]  # green

        virtual_shape_id = pybullet.createVisualShape(shapeType=pybullet.GEOM_SPHERE,
                                                      radius=marker_radius,
                                                      rgbaColor=col)
        body_id =  pybullet.createMultiBody(baseMass=0,
                                            baseCollisionShapeIndex=-1,
                                            baseVisualShapeIndex=virtual_shape_id,
                                            basePosition=[0, 0, 0],
                                            useMaximalCoordinates=True)
        markers.append(body_id)

    return markers


def get_joint_limits(robot):
    num_joints = pybullet.getNumJoints(robot)
    joint_limit_low = []
    joint_limit_high = []

    for i in range(num_joints):
        joint_info = pybullet.getJointInfo(robot, i)
        joint_name = joint_info[1].decode('utf-8')

        if joint_name in config.MOTOR_NAMES:
          joint_limit_low.append(joint_info[8])
          joint_limit_high.append(joint_info[9])

    return joint_limit_low, joint_limit_high


def get_root_pos(pose):
    return pose[0:POS_SIZE]


def get_root_rot(pose):
    return pose[POS_SIZE:(POS_SIZE + ROT_SIZE)]


def get_joint_pose(pose):
    return pose[(POS_SIZE + ROT_SIZE):]


def set_root_pos(root_pos, pose):
    pose[0:POS_SIZE] = root_pos
    return


def set_root_rot(root_rot, pose):
    pose[POS_SIZE:(POS_SIZE + ROT_SIZE)] = root_rot
    return


def set_joint_pose(joint_pose, pose):
    pose[(POS_SIZE + ROT_SIZE):] = joint_pose
    return


def set_pose(robot, pose):
    num_joints = pybullet.getNumJoints(robot)
    root_pos = get_root_pos(pose)
    root_rot = get_root_rot(pose)

    pybullet.resetBasePositionAndOrientation(robot, root_pos, root_rot)

    j_pose_idx = len(root_pos) + len(root_rot)
    for j in range(num_joints):
        j_info = pybullet.getJointInfo(robot, j)
        j_state = pybullet.getJointStateMultiDof(robot, j)

        j_name = j_info[1].decode('utf-8')
        #  = j_info[3]
        j_pose_size = len(j_state[0])
        j_vel_size = len(j_state[1])

        if j_name in config.MOTOR_NAMES:
            j_pose = pose[j_pose_idx:(j_pose_idx + j_pose_size)]
            j_vel = np.zeros(j_vel_size)
            pybullet.resetJointStateMultiDof(robot, j, j_pose, j_vel)
            j_pose_idx += 1

    return


def set_maker_pos(marker_pos, marker_ids):
    num_markers = len(marker_ids)
    assert(num_markers == marker_pos.shape[0])

    for i in range(num_markers):
        curr_id = marker_ids[i]
        curr_pos = marker_pos[i]

        pybullet.resetBasePositionAndOrientation(curr_id, curr_pos, DEFAULT_ROT)

    return


def process_ref_joint_pos_data(joint_pos):
    proc_pos = joint_pos.copy()
    num_pos = joint_pos.shape[0]

    for i in range(num_pos):
        curr_pos = proc_pos[i]
        curr_pos = QuaternionRotatePoint(curr_pos, REF_COORD_ROT)
        curr_pos = QuaternionRotatePoint(curr_pos, REF_ROOT_ROT)
        curr_pos = curr_pos * config.REF_POS_SCALE + REF_POS_OFFSET  # change pose size for different skeleton
        proc_pos[i] = curr_pos

    return proc_pos


def retarget_root_pose(ref_joint_pos, get_motion_name, init_rotation):
    pelvis_pos = ref_joint_pos[REF_HIP_JOINT_ID]
    neck_pos = ref_joint_pos[REF_NECK_JOINT_ID]

    left_shoulder_pos = ref_joint_pos[REF_HIP_JOINT_IDS[1]]
    right_shoulder_pos = ref_joint_pos[REF_HIP_JOINT_IDS[0]]
    left_hip_pos = ref_joint_pos[REF_HIP_JOINT_IDS[3]]
    right_hip_pos = ref_joint_pos[REF_HIP_JOINT_IDS[2]]

    up_dir = neck_pos - pelvis_pos  # vector difference
    up_dir = up_dir / np.linalg.norm(up_dir)  # unitize

    delta_shoulder = left_shoulder_pos - right_shoulder_pos
    delta_hip = left_hip_pos - right_hip_pos
    dir_shoulder = delta_shoulder / np.linalg.norm(delta_shoulder)
    dir_hip = delta_hip / np.linalg.norm(delta_hip)    # unitize

    # left_dir = 0.66 * (dir_shoulder + dir_hip)
    left_dir = dir_hip

    forward_dir = np.cross(up_dir, left_dir)
    forward_dir = forward_dir / np.linalg.norm(forward_dir)

    left_dir = np.cross(forward_dir, up_dir)
    left_dir = left_dir / np.linalg.norm(left_dir)  # unitize

    rot_mat = np.array([[forward_dir[0], left_dir[0], up_dir[0], 0],
                      [forward_dir[1], left_dir[1], up_dir[1], 0],
                      [forward_dir[2], left_dir[2], up_dir[2], 0],
                      [0, 0, 0, 1]])
    if get_motion_name == 'walk':
        root_pos = pelvis_pos
        root_pos[2] -= 0.10  # let robot move upward 0.10
    elif get_motion_name == 'jump':
        root_pos = pelvis_pos
        root_pos[2] -= 0.12

        global lock, limit_root_low
        if lock == 0:
            limit_root_low = root_pos[2]+0.06
            lock = 1
        if root_pos[2] <= limit_root_low:
            root_pos[2] = limit_root_low
    else:
        print("This motion is not implemented.")
        exit(0)
    # root_pos = 0.16 * (left_shoulder_pos + right_shoulder_pos + left_hip_pos + right_hip_pos)
    root_rot = transformations.quaternion_from_matrix(rot_mat)
    root_rot = transformations.quaternion_multiply(root_rot, init_rotation)
    root_rot = root_rot / np.linalg.norm(root_rot)

    return root_pos, root_rot


# compute the pose of nao in every frame
def retarget_pose(robot, default_pose, ref_joint_pos, _motion_name, init_rot):
    joint_lim_low, joint_lim_high = get_joint_limits(robot)

    root_pos, root_rot = retarget_root_pose(ref_joint_pos, _motion_name, init_rot)  # INIT_POS, INIT_ROT
    root_pos += config.SIM_ROOT_OFFSET

    pybullet.resetBasePositionAndOrientation(robot, root_pos, root_rot)

    inv_init_rot = transformations.quaternion_inverse(init_rot)
    heading_rot = calc_heading_rot(transformations.quaternion_multiply(root_rot, inv_init_rot))  # 得到绕着z轴旋转的四元数

    tar_toe_mid_pos = []
    for i in range(len(REF_TOE_JOINT_IDS)):
        ref_toe_id = REF_TOE_JOINT_IDS[i]
        ref_mid_id = REF_MID_JOINT_IDS[i]
        ref_hip_id = REF_HIP_JOINT_IDS[i]
        sim_toe_id = config.SIM_TOE_JOINT_IDS[i]
        sim_mid_id = config.SIM_MID_JOINT_IDS[i]
        sim_hip_id = config.SIM_HIP_JOINT_IDS[i]
        toe_offset_local = config.SIM_TOE_OFFSET_LOCAL[i]
        mid_offset_local = config.SIM_MID_OFFSET_LOCAL[i]

        ref_toe_pos = ref_joint_pos[ref_toe_id]
        ref_mid_pos = ref_joint_pos[ref_mid_id]
        ref_hip_pos = ref_joint_pos[ref_hip_id]

        toe_link_state = pybullet.getLinkState(robot, sim_toe_id, computeForwardKinematics=True)
        mid_link_state = pybullet.getLinkState(robot, sim_mid_id, computeForwardKinematics=True)
        hip_link_state = pybullet.getLinkState(robot, sim_hip_id, computeForwardKinematics=True)
        sim_toe_pos = np.array(toe_link_state[4])  # world position
        sim_mid_pos = np.array(mid_link_state[4])
        sim_hip_pos = np.array(hip_link_state[4])

        toe_offset_world = QuaternionRotatePoint(toe_offset_local, heading_rot)
        mid_offset_world = QuaternionRotatePoint(mid_offset_local, heading_rot)

        ref_hip_toe_delta = ref_toe_pos - ref_hip_pos
        sim_tar_toe_pos = sim_hip_pos + ref_hip_toe_delta

        ref_hip_mid_delta = ref_mid_pos - ref_hip_pos
        sim_tar_mid_pos = sim_mid_pos + ref_hip_mid_delta

        tar_toe_mid_pos.append(sim_tar_toe_pos)
        tar_toe_mid_pos.append(sim_tar_mid_pos)
    # depending on the state of toe, the relative position of all joints can be acquired
    joint_pose = pybullet.calculateInverseKinematics2(robot, config.SIM_TOE_MID_JOINT_IDS,
                                                      tar_toe_mid_pos,
                                                      jointDamping=config.JOINT_DAMPING,
                                                      lowerLimits=joint_lim_low,
                                                      upperLimits=joint_lim_high,
                                                      restPoses=default_pose)
    joint_pose = np.array(np.concatenate([joint_pose[0:20], joint_pose[28:34]]))

    pose = np.concatenate([root_pos, root_rot, joint_pose])

    return pose


def update_camera(robot):
    base_pos = np.array(pybullet.getBasePositionAndOrientation(robot)[0])
    [yaw, pitch, dist] = pybullet.getDebugVisualizerCamera()[8:11]
    pybullet.resetDebugVisualizerCamera(dist, yaw, pitch, base_pos)
    return


def load_ref_data(MOTION_NAME, JOINT_POS_FILENAME, FRAME_START, FRAME_END):
    joint_pos_data = pds.read_csv(JOINT_POS_FILENAME)
    joint_pos_data.dropna(axis='columns', how='any')

    start_frame = 0 if (FRAME_START is None) else FRAME_START
    end_frame = joint_pos_data.shape[0] if (FRAME_END is None) else FRAME_END
    joint_pos_data = joint_pos_data[start_frame:end_frame]
    joint_pos_data = joint_pos_data.values[:, 2:REF_jOINT_NUM*3+2]
    if MOTION_NAME == 'walk':
        for frame in range(joint_pos_data.shape[0]):
            joint_pos_data[frame][1 * 3 + 2] -= 0.1   # rightshoulder move outward 0.1  (0,1,2)=(y,x,z)
            joint_pos_data[frame][4 * 3 + 2] += 0.1   # leftshoulder
            joint_pos_data[frame][2 * 3 + 1] -= 0.13  # rightforearm
            joint_pos_data[frame][2 * 3 + 0] -= 0.02  # rightforearm
            joint_pos_data[frame][3 * 3 + 1] -= 0.1   # rightarm
            joint_pos_data[frame][5 * 3 + 1] -= 0.13  # leftforearm
            joint_pos_data[frame][5 * 3 + 0] += 0.02  # leftforearm
            joint_pos_data[frame][6 * 3 + 1] -= 0.1   # leftarm
    elif MOTION_NAME == 'jump':
        for frame in range(joint_pos_data.shape[0]):
            joint_pos_data[frame][1 * 3 + 0] -= 0.1   # rightshoulder
            joint_pos_data[frame][4 * 3 + 0] += 0.1   # leftshoulder
            joint_pos_data[frame][2 * 3 + 1] -= 0.13  # rightforearm
            joint_pos_data[frame][2 * 3 + 0] -= 0.02  # rightforearm
            joint_pos_data[frame][3 * 3 + 1] -= 0.1   # rightarm
            joint_pos_data[frame][5 * 3 + 1] -= 0.13  # leftforearm
            joint_pos_data[frame][5 * 3 + 0] += 0.02  # leftforearm
            joint_pos_data[frame][6 * 3 + 1] -= 0.1   # leftarm

    return joint_pos_data


def retarget_motion(robot, joint_pos_data, motion_name, init_rotat):
    num_frames = joint_pos_data.shape[0]

    for f in range(num_frames):
        ref_joint_pos = joint_pos_data[f]
        ref_joint_pos = np.reshape(ref_joint_pos, [-1, POS_SIZE])
        ref_joint_pos = process_ref_joint_pos_data(ref_joint_pos)

        curr_pose = retarget_pose(robot, config.DEFAULT_JOINT_POSE, ref_joint_pos, motion_name, init_rotat)
        set_pose(robot, curr_pose)

        if f == 0:
            pose_size = curr_pose.shape[-1]
            new_frames = np.zeros([num_frames, pose_size])

        new_frames[f] = curr_pose

    return new_frames


def output_motion(frames, out_filename, frame_duration):
    with open(out_filename, "a+") as f:
        f.write("{\n")
        f.write("\"LoopMode\": \"Wrap\",\n")
        f.write("\"FrameDuration\": " + str(frame_duration) + ",\n")
        f.write("\"EnableCycleOffsetPosition\": true,\n")
        f.write("\"EnableCycleOffsetRotation\": true,\n")
        f.write("\n")

        f.write("\"Frames\":\n")

        f.write("[")
        for i in range(frames.shape[0]):
            curr_frame = frames[i]

            if i != 0:
                f.write(",")
            f.write("\n  [")

            for j in range(frames.shape[1]):
                curr_val = curr_frame[j]
                if j != 0:
                    f.write(", ")
                f.write("%.5f" % curr_val)

            f.write("]")

        f.write("\n]")
        f.write("\n}")

    return


def main(txt_data):
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True, auto_step=False)

    pybullet.setAdditionalSearchPath(pd.getDataPath())
    init_rot = transformations.quaternion_from_euler(txt_data['init_rot'][0], txt_data['init_rot'][1],
                                                     txt_data['init_rot'][2], axes="sxyz")
    init_pos = np.array(txt_data['init_pos'])
    while True:

        pybullet.resetSimulation()
        pybullet.setGravity(0, 0, 0)

        ground = pybullet.loadURDF(GROUND_URDF_FILENAME)
        robot = pybullet.loadURDF(config.URDF_FILENAME, init_pos, init_rot,
                                  useFixedBase=False,
                                  globalScaling=1.0,
                                  physicsClientId=client,
                                  flags=pybullet.URDF_USE_SELF_COLLISION | pybullet.URDF_USE_MATERIAL_COLORS_FROM_MTL)

        pybullet.removeAllUserDebugItems()
        print("mocap_name=", txt_data['motion_name'])
        joint_pos_data = load_ref_data(txt_data['motion_name'], txt_data['motion_path'],
                                       txt_data['start_frame'], txt_data['end_frame'])

        num_markers = joint_pos_data.shape[-1] // POS_SIZE
        marker_ids = build_markers(num_markers)  # build the key points

        retarget_frames = retarget_motion(robot, joint_pos_data, txt_data['motion_name'], init_rot)
        output_motion(retarget_frames, txt_data['output_path'], txt_data['frame_duration'])

        f = 0
        num_frames = joint_pos_data.shape[0]

        for repeat in range(5*num_frames):
            time_start = time.time()

            f_idx = f % num_frames
            print("Frame {:d}".format(f_idx))

            ref_joint_pos = joint_pos_data[f_idx]
            ref_joint_pos = np.reshape(ref_joint_pos, [-1, POS_SIZE])
            ref_joint_pos = process_ref_joint_pos_data(ref_joint_pos)

            pose = retarget_frames[f_idx]

            set_pose(robot, pose)
            set_maker_pos(ref_joint_pos, marker_ids)

            update_camera(robot)
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SINGLE_STEP_RENDERING, 1)
            f += 1

            time_end = time.time()
            sleep_dur = txt_data['frame_duration'] - (time_end - time_start)
            sleep_dur = max(0, sleep_dur)

            # time.sleep(sleep_dur)
        for m in marker_ids:
            pybullet.removeBody(m)

    pybullet.disconnect()
    return


if __name__ == "__main__":
    txt_dict = read_txt_file("./setting/jump_setting.txt")
    main(txt_dict)

