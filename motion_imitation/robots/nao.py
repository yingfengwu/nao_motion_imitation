# coding=utf-8
# Copyright 2020 The Google Research Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)


"""Pybullet simulation of a nao robot."""
import math
import os
import re
import numpy as np
import pybullet as pyb  # pytype: disable=import-error

from motion_imitation.robots import nao_pose_utils
from motion_imitation.robots import nao_constants
from motion_imitation.robots import nao_motor
from motion_imitation.robots import humanoid
from motion_imitation.robots import robot_config
from motion_imitation.envs import locomotion_gym_config

NUM_MOTORS = 26
NUM_LEGS = 2
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
INIT_RACK_POSITION = [0, 0, 0.335]
INIT_POSITION = [0, 0, 0.335]
JOINT_DIRECTIONS = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
HIP_JOINT_OFFSET = 0.01
UPPER_LEG_JOINT_OFFSET = -0.06
KNEE_JOINT_OFFSET = 0.06
DOFS_PER_LEG = 3
INIT_JOINT_OFFSETS = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5707096, 0, 0, 0, 0, 0, 1.5707096, 0, 0, 0, 0, 0])
JOINT_OFFSETS = np.array([0.0] * NUM_MOTORS)
PI = math.pi

MAX_MOTOR_ANGLE_CHANGE_PER_STEP = 0.5
_DEFAULT_HIP_POSITIONS = (
    (0.21, -0.1157, 0),
    (0.21, 0.1157, 0),
    (-0.21, -0.1157, 0),
    (-0.21, 0.1157, 0),
)

HEAD_P_GAIN = 100
HEAD_D_GAIN = 10
ELBOW_P_GAIN = 100
ELBOW_D_GAIN = 10
SHOULDER_P_GAIN = 100.0
SHOULDER_D_GAIN = 10
WRIST_P_GAIN = 0
WRIST_D_GAIN = 0
HIP_P_GAIN = 130.0
HIP_D_GAIN = 13
KNEE_P_GAIN = 130.0
KNEE_D_GAIN = 13
ANKLE_P_GAIN = 130
ANKLE_D_GAIN = 13

# Bases on the readings from nao's default pose.
INIT_MOTOR_ANGLES = np.array([0.0, ] * NUM_MOTORS)

_MOTOR_NAME_PATTERN = re.compile(r"Head\D*")
_ARM_NAME_PATTERN1 = re.compile(r"\w+Shoulder\w+")
_ARM_NAME_PATTERN2 = re.compile(r"\w+Elbow\w+")
_ARM_NAME_PATTERN3 = re.compile(r"\w+Hand\w+")
_ARM_NAME_PATTERN4 = re.compile(r"\w+Wrist\w+")
_ARM_NAME_PATTERN5 = re.compile(r"\w+Hand")
_BRACKET_NAME_PATTERN = re.compile(r"motor\D*")
_LEG_NAME_PATTERN1 = re.compile(r"\w+Hip\w+")
_LEG_NAME_PATTERN2 = re.compile(r"\w+Knee\w+")
_LEG_NAME_PATTERN3 = re.compile(r"\w+Ankle\w+")

URDF_FILENAME = "E:/my_data/MachineLearning/Python_Code/Bair_paper_code/robot_data/nao.urdf"

_BODY_B_FIELD_NUMBER = 2
_LINK_A_FIELD_NUMBER = 3

UPPER_BOUND = [2.08567, 0.514872, 0.740718, 0.79046, 0.48398, 2.11255, 0.922581, 0.768992, 0.740718, 0.379435, 0.48398,
               2.11255, 0.932006, 0.397761, 2.08567, 1.32645, 2.08567, -0.0349066, 1.82387, 1.0, 2.08567, 0.314159,
               2.08567, 1.54462, 1.82387, 1.0]
LOWER_BOUND = [-2.08567, -0.671952, -1.14529, -0.379435, -1.53589, -0.0923279, -1.18944, -0.397761, -1.14529, -0.79046,
               -1.53589, -0.0923279, -1.1863, -0.768992, -2.08567, -0.314159, -2.08567, -1.54462, -1.82387, 0.0, -2.08567,
               -1.32645, -2.08567, 0.0349066, -1.82387, 0.0]


class Nao(humanoid.Humanoid):
  """A simulation for the nao robot."""
  MPC_BODY_MASS = 215/9.8
  MPC_BODY_INERTIA = (0.07335, 0, 0, 0, 0.25068, 0, 0, 0, 0.25447)
  MPC_BODY_HEIGHT = 0.42
  ACTION_CONFIG = [
      locomotion_gym_config.ScalarField(name="motor_angle_0",
                                        upper_bound=UPPER_BOUND[0],
                                        lower_bound=LOWER_BOUND[0]),
      locomotion_gym_config.ScalarField(name="motor_angle_1",
                                        upper_bound=UPPER_BOUND[1],
                                        lower_bound=LOWER_BOUND[1]),
      locomotion_gym_config.ScalarField(name="motor_angle_2",
                                        upper_bound=UPPER_BOUND[2],
                                        lower_bound=LOWER_BOUND[2]),
      locomotion_gym_config.ScalarField(name="motor_angle_3",
                                        upper_bound=UPPER_BOUND[3],
                                        lower_bound=LOWER_BOUND[3]),
      locomotion_gym_config.ScalarField(name="motor_angle_4",
                                        upper_bound=UPPER_BOUND[4],
                                        lower_bound=LOWER_BOUND[4]),
      locomotion_gym_config.ScalarField(name="motor_angle_5",
                                        upper_bound=UPPER_BOUND[5],
                                        lower_bound=LOWER_BOUND[5]),
      locomotion_gym_config.ScalarField(name="motor_angle_6",
                                        upper_bound=UPPER_BOUND[6],
                                        lower_bound=LOWER_BOUND[6]),
      locomotion_gym_config.ScalarField(name="motor_angle_7",
                                        upper_bound=UPPER_BOUND[7],
                                        lower_bound=LOWER_BOUND[7]),
      locomotion_gym_config.ScalarField(name="motor_angle_8",
                                        upper_bound=UPPER_BOUND[8],
                                        lower_bound=LOWER_BOUND[8]),
      locomotion_gym_config.ScalarField(name="motor_angle_9",
                                        upper_bound=UPPER_BOUND[9],
                                        lower_bound=LOWER_BOUND[9]),
      locomotion_gym_config.ScalarField(name="motor_angle_10",
                                        upper_bound=UPPER_BOUND[10],
                                        lower_bound=LOWER_BOUND[10]),
      locomotion_gym_config.ScalarField(name="motor_angle_11",
                                        upper_bound=UPPER_BOUND[11],
                                        lower_bound=LOWER_BOUND[11]),
      locomotion_gym_config.ScalarField(name="motor_angle_12",
                                        upper_bound=UPPER_BOUND[12],
                                        lower_bound=LOWER_BOUND[12]),
      locomotion_gym_config.ScalarField(name="motor_angle_13",
                                        upper_bound=UPPER_BOUND[13],
                                        lower_bound=LOWER_BOUND[13]),
      locomotion_gym_config.ScalarField(name="motor_angle_14",
                                        upper_bound=UPPER_BOUND[14],
                                        lower_bound=LOWER_BOUND[14]),
      locomotion_gym_config.ScalarField(name="motor_angle_15",
                                        upper_bound=UPPER_BOUND[15],
                                        lower_bound=LOWER_BOUND[15]),
      locomotion_gym_config.ScalarField(name="motor_angle_16",
                                        upper_bound=UPPER_BOUND[16],
                                        lower_bound=LOWER_BOUND[16]),
      locomotion_gym_config.ScalarField(name="motor_angle_17",
                                        upper_bound=UPPER_BOUND[17],
                                        lower_bound=LOWER_BOUND[17]),
      locomotion_gym_config.ScalarField(name="motor_angle_18",
                                        upper_bound=UPPER_BOUND[18],
                                        lower_bound=LOWER_BOUND[18]),
      locomotion_gym_config.ScalarField(name="motor_angle_19",
                                        upper_bound=UPPER_BOUND[19],
                                        lower_bound=LOWER_BOUND[19]),
      locomotion_gym_config.ScalarField(name="motor_angle_20",
                                        upper_bound=UPPER_BOUND[20],
                                        lower_bound=LOWER_BOUND[20]),
      locomotion_gym_config.ScalarField(name="motor_angle_21",
                                        upper_bound=UPPER_BOUND[21],
                                        lower_bound=LOWER_BOUND[21]),
      locomotion_gym_config.ScalarField(name="motor_angle_22",
                                        upper_bound=UPPER_BOUND[22],
                                        lower_bound=LOWER_BOUND[22]),
      locomotion_gym_config.ScalarField(name="motor_angle_23",
                                        upper_bound=UPPER_BOUND[23],
                                        lower_bound=LOWER_BOUND[23]),
      locomotion_gym_config.ScalarField(name="motor_angle_24",
                                        upper_bound=UPPER_BOUND[24],
                                        lower_bound=LOWER_BOUND[24]),
      locomotion_gym_config.ScalarField(name="motor_angle_25",
                                        upper_bound=UPPER_BOUND[25],
                                        lower_bound=LOWER_BOUND[25])
  ]

  def __init__(
      self,
      pybullet_client,
      motor_control_mode,
      urdf_filename=URDF_FILENAME,
      enable_clip_motor_commands=False,
      time_step=0.001,
      action_repeat=33,
      sensors=None,
      control_latency=0.002,
      on_rack=False,
      enable_action_interpolation=True,
      enable_action_filter=False,
      reset_time=-1,
      allow_knee_contact=False,
  ):
    self._urdf_filename = urdf_filename
    self._allow_knee_contact = allow_knee_contact
    self._enable_clip_motor_commands = enable_clip_motor_commands

    motor_kp = [
        HEAD_P_GAIN, HEAD_P_GAIN, HIP_P_GAIN, HIP_P_GAIN, HIP_P_GAIN,
        KNEE_P_GAIN, ANKLE_P_GAIN, ANKLE_P_GAIN, HIP_P_GAIN, HIP_P_GAIN, HIP_P_GAIN,
        KNEE_P_GAIN, ANKLE_P_GAIN, ANKLE_P_GAIN, SHOULDER_P_GAIN, SHOULDER_P_GAIN, ELBOW_P_GAIN, ELBOW_P_GAIN,
        WRIST_P_GAIN, WRIST_P_GAIN, SHOULDER_P_GAIN, SHOULDER_P_GAIN, ELBOW_P_GAIN, ELBOW_P_GAIN,
        WRIST_P_GAIN, WRIST_P_GAIN
    ]
    motor_kd = [
        HEAD_D_GAIN, HEAD_D_GAIN, HIP_D_GAIN, HIP_D_GAIN, HIP_D_GAIN,
        KNEE_D_GAIN, ANKLE_D_GAIN, ANKLE_D_GAIN, HIP_D_GAIN, HIP_D_GAIN, HIP_P_GAIN,
        KNEE_D_GAIN, ANKLE_D_GAIN, ANKLE_D_GAIN, SHOULDER_D_GAIN, SHOULDER_D_GAIN, ELBOW_D_GAIN, ELBOW_D_GAIN,
        WRIST_D_GAIN, WRIST_D_GAIN, SHOULDER_D_GAIN, SHOULDER_D_GAIN, ELBOW_D_GAIN, ELBOW_D_GAIN,
        WRIST_D_GAIN, WRIST_D_GAIN
    ]

    super(Nao, self).__init__(
        pybullet_client=pybullet_client,
        time_step=time_step,
        action_repeat=action_repeat,
        num_motors=NUM_MOTORS,
        dofs_per_leg=DOFS_PER_LEG,
        motor_direction=JOINT_DIRECTIONS,
        motor_offset=JOINT_OFFSETS,
        motor_overheat_protection=False,
        motor_control_mode=motor_control_mode,
        motor_model_class=nao_motor.NaoMotorModel,
        sensors=sensors,
        motor_kp=motor_kp,
        motor_kd=motor_kd,
        control_latency=control_latency,
        on_rack=on_rack,
        enable_action_interpolation=enable_action_interpolation,
        enable_action_filter=enable_action_filter,
        reset_time=reset_time)

  def _LoadRobotURDF(self):
    nao_urdf_path = self.GetURDFFile()
    if self._self_collision_enabled:
      self.biped = self._pybullet_client.loadURDF(
          nao_urdf_path,
          self._GetDefaultInitPosition(),
          self._GetDefaultInitOrientation(),
          flags=self._pybullet_client.URDF_USE_SELF_COLLISION |
          self._pybullet_client.URDF_USE_MATERIAL_COLORS_FROM_MTL)
    else:
      self.biped = self._pybullet_client.loadURDF(
          nao_urdf_path, self._GetDefaultInitPosition(),
          self._GetDefaultInitOrientation(), flags=self._pybullet_client.URDF_USE_MATERIAL_COLORS_FROM_MTL)

  def _SettleDownForReset(self, default_motor_angles, reset_time):
    self.ReceiveObservation()

    if reset_time <= 0:
      return

    for _ in range(500):
      self._StepInternal(
          INIT_MOTOR_ANGLES,
          motor_control_mode=robot_config.MotorControlMode.POSITION)
    if default_motor_angles is not None:
      num_steps_to_reset = int(reset_time / self.time_step)
      for _ in range(num_steps_to_reset):
        self._StepInternal(
            default_motor_angles,
            motor_control_mode=robot_config.MotorControlMode.POSITION)

  def GetHipPositionsInBaseFrame(self):
    return _DEFAULT_HIP_POSITIONS

  def GetFootContacts(self):
    all_contacts = self._pybullet_client.getContactPoints(bodyA=self.biped)

    contacts = [False, False, False, False]
    for contact in all_contacts:
      # Ignore self contacts
      if contact[_BODY_B_FIELD_NUMBER] == self.biped:
        continue
      try:
        toe_link_index = self._leg_link_ids.index(
            contact[_LINK_A_FIELD_NUMBER])
        contacts[toe_link_index] = True
      except ValueError:
        continue

    return contacts

  def ComputeJacobian(self, leg_id):
    """Compute the Jacobian for a given leg."""
    # Because of the default rotation in the Laikago URDF, we need to reorder
    # the rows in the Jacobian matrix.
    return super(Nao, self).ComputeJacobian(leg_id)[(2, 0, 1), :]

  def ResetPose(self, add_constraint):
    del add_constraint
    for name in self._joint_name_to_id:
      joint_id = self._joint_name_to_id[name]
      self._pybullet_client.setJointMotorControl2(
          bodyIndex=self.biped,
          jointIndex=(joint_id),
          controlMode=self._pybullet_client.VELOCITY_CONTROL,
          targetVelocity=0,
          force=0)
    for name, i in zip(MOTOR_NAMES, range(len(MOTOR_NAMES))):
      if "Hip" or "Shoulder" in name:
        angle = INIT_MOTOR_ANGLES[i] + HIP_JOINT_OFFSET
      elif "Ankle" or "Elbow" in name:
        angle = INIT_MOTOR_ANGLES[i] + UPPER_LEG_JOINT_OFFSET
      elif "Knee" or "Wrist" in name:
        angle = INIT_MOTOR_ANGLES[i] + KNEE_JOINT_OFFSET
      else:
        raise ValueError("The name %s is not recognized as a motor joint." %
                         name)
      self._pybullet_client.resetJointState(self.biped,
                                            self._joint_name_to_id[name],
                                            angle,
                                            targetVelocity=0)

  def GetURDFFile(self):
    return self._urdf_filename

  def _BuildUrdfIds(self):
    """Build the link Ids from its name in the URDF file.

    Raises:
      ValueError: Unknown category of the joint name.
    """
    num_joints = self._pybullet_client.getNumJoints(self.biped)
    self._torso_link_ids = [-1, 0, 2, 8]
    self._leg_link_ids = []
    self._motor_link_ids = []
    self._arm_link_ids = []

    for i in range(num_joints):
      joint_info = self._pybullet_client.getJointInfo(self.biped, i)
      joint_name = joint_info[1].decode("UTF-8") if joint_info[1].decode("UTF-8") in MOTOR_NAMES else None
      if joint_name == None:
          continue
      joint_id = self._joint_name_to_id[joint_name]
      if _MOTOR_NAME_PATTERN.match(joint_name):
        self._motor_link_ids.append(joint_id)
      # We either treat the lower leg or the toe as the foot link, depending on
      # the urdf version used.
      elif (_LEG_NAME_PATTERN1.match(joint_name) or _LEG_NAME_PATTERN2.match(joint_name)
            or _LEG_NAME_PATTERN3.match(joint_name)):
        self._leg_link_ids.append(joint_id)
      elif _ARM_NAME_PATTERN1.match(joint_name) or _ARM_NAME_PATTERN2.match(joint_name) or \
              _ARM_NAME_PATTERN3.match(joint_name) or _ARM_NAME_PATTERN4.match(joint_name) or \
              _ARM_NAME_PATTERN5.match(joint_name):
        self._arm_link_ids.append(joint_id)
      else:
        raise ValueError("Unknown category of joint %s" % joint_name)

    self._motor_link_ids.extend(self._arm_link_ids)
    self._motor_link_ids.extend(self._leg_link_ids)

    self._torso_link_ids.sort()
    self._motor_link_ids.sort()
    self._arm_link_ids.sort()
    self._leg_link_ids.sort()

  def _GetMotorNames(self):
    return MOTOR_NAMES

  def _GetDefaultInitPosition(self):
    if self._on_rack:
      return INIT_RACK_POSITION
    else:
      return INIT_POSITION

  def _GetDefaultInitOrientation(self):
    # The Laikago URDF assumes the initial pose of heading towards z axis,
    # and belly towards y axis. The following transformation is to transform
    # the Laikago initial orientation to our commonly used orientation: heading
    # towards -x direction, and z axis is the up direction.
    init_orientation = pyb.getQuaternionFromEuler(
        [0, 0, 0])
    return init_orientation

  def GetDefaultInitPosition(self):
    """Get default initial base position."""
    return self._GetDefaultInitPosition()

  def GetDefaultInitOrientation(self):
    """Get default initial base orientation."""
    return self._GetDefaultInitOrientation()

  def GetDefaultInitJointPose(self):
    """Get default initial joint pose."""
    joint_pose = (INIT_MOTOR_ANGLES + INIT_JOINT_OFFSETS) * JOINT_DIRECTIONS
    return joint_pose

  def ApplyAction(self, motor_commands, motor_control_mode):
    """Clips and then apply the motor commands using the motor model.

    Args:
      motor_commands: np.array. Can be motor angles, torques, hybrid commands,
        or motor pwms (for Minitaur only).N
      motor_control_mode: A MotorControlMode enum.
    """
    if self._enable_clip_motor_commands:
      motor_commands = self._ClipMotorCommands(motor_commands)

    super(Nao, self).ApplyAction(motor_commands, motor_control_mode)

  def _ClipMotorCommands(self, motor_commands):
    """Clips motor commands.

    Args:
      motor_commands: np.array. Can be motor angles, torques, hybrid commands,
        or motor pwms (for Minitaur only).

    Returns:
      Clipped motor commands.
    """

    # clamp the motor command by the joint limit, in case weired things happens
    max_angle_change = MAX_MOTOR_ANGLE_CHANGE_PER_STEP
    current_motor_angles = self.GetMotorAngles()
    motor_commands = np.clip(motor_commands,
                             current_motor_angles - max_angle_change,
                             current_motor_angles + max_angle_change)
    return motor_commands

  @classmethod
  def GetConstants(cls):
    del cls
    return nao_constants
