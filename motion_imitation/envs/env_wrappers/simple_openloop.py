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

"""Simple openloop trajectory generators."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import attr
from gym import spaces
import numpy as np

from motion_imitation.robots import nao_pose_utils
from motion_imitation.robots import minitaur_pose_utils
# from motion_imitation.envs.env_wrappers.imitation_task import

class MinitaurPoseOffsetGenerator(object):
  """A trajectory generator that return a constant leg pose."""

  def __init__(self,
               init_swing=0,
               init_extension=2.0,
               init_pose=None,
               action_scale=1.0,
               action_limit=0.5):
    """Initializes the controller.

    Args:
      init_swing: the swing of the default pose offset
      init_extension: the extension of the default pose offset
      init_pose: the default pose offset, which is None by default. If not None,
        it will define the default pose offset while ignoring init_swing and
        init_extension.
      action_scale: changes the magnitudes of actions
      action_limit: clips actions
    """
    if init_pose is None:
      self._pose = np.array(
          attr.astuple(
              minitaur_pose_utils.MinitaurPose(
                  swing_angle_0=init_swing,
                  swing_angle_1=init_swing,
                  swing_angle_2=init_swing,
                  swing_angle_3=init_swing,
                  extension_angle_0=init_extension,
                  extension_angle_1=init_extension,
                  extension_angle_2=init_extension,
                  extension_angle_3=init_extension)))
    else:  # Ignore init_swing and init_extension
      self._pose = np.array(init_pose)
    action_high = np.array([action_limit] * minitaur_pose_utils.NUM_MOTORS)
    self.action_space = spaces.Box(-action_high, action_high, dtype=np.float32)
    self._action_scale = action_scale

  def reset(self):
    pass

  def get_action(self, current_time=None, input_action=None):
    """Computes the trajectory according to input time and action.

    Args:
      current_time: The time in gym env since reset.
      input_action: A numpy array. The input leg pose from a NN controller.

    Returns:
      A numpy array. The desired motor angles.
    """
    del current_time
    return minitaur_pose_utils.leg_pose_to_motor_angles(self._pose +
                                                        self._action_scale *
                                                        np.array(input_action))

  def get_observation(self, input_observation):
    """Get the trajectory generator's observation."""

    return input_observation


class NaoPoseOffsetGenerator(object):
  """A trajectory generator that return constant motor angles."""
  def __init__(
      self,
      init_abduction=nao_pose_utils.NAO_DEFAULT_ABDUCTION_ANGLE,
      init_hip=nao_pose_utils.NAO_DEFAULT_HIP_ANGLE,
      init_knee=nao_pose_utils.NAO_DEFAULT_KNEE_ANGLE,
      action_limit_upper=0.5,
      action_limit_lower=0.5,
  ):
    """Initializes the controller.
    Args:
      action_limit: a tuple of [limit_abduction, limit_hip, limit_knee]
    """
    self._pose = np.array(
        attr.astuple(
            nao_pose_utils.NaoPose(HeadYaw_angle=init_abduction,
                                           HeadPitch=init_hip,
                                           LHipYawPitch_angle=init_knee,
                                           LHipRoll_angle=init_abduction,
                                           LHipPitch_angle=init_hip,
                                           LKneePitch_angle=init_knee,
                                           LAnklePitch_angle=init_abduction,
                                           LAnkleRoll_angle=init_abduction,
                                           RHipYawPitch_angle=init_abduction,
                                           RHipRoll_angle=init_hip,
                                           RHipPitch_angle=init_hip,
                                           RKneePitch_angle=init_knee,
                                           RAnklePitch_angle=init_abduction,
                                           RAnkleRoll_angle=init_abduction,
                                           LShoulderPitch_angle=init_abduction,
                                           LShoulderRoll_angle=init_abduction,
                                           LElbowYaw_angle=init_abduction,
                                           LElbowRoll_angle=init_abduction,
                                           LWristYaw_angle=init_abduction,
                                           LHand_angle=init_abduction,
                                           RShoulderPitch_angle=init_abduction,
                                           RShoulderRoll_angle=init_abduction,
                                           RElbowYaw_angle=init_abduction,
                                           RElbowRoll_angle=init_abduction,
                                           RWristYaw_angle=init_abduction,
                                           RHand_angle=init_abduction)))
    action_high = np.reshape(np.array([action_limit_upper]), [26,])
    action_low  = np.reshape(np.array([action_limit_lower]), [26,])
    self.action_space = spaces.Box(action_low, action_high, dtype=np.float32)

  def reset(self):
    pass

  def get_action(self, current_time=None, input_action=None):
    """Computes the trajectory according to input time and action.

    Args:
      current_time: The time in gym env since reset.
      input_action: A numpy array. The input leg pose from a NN controller.

    Returns:
      A numpy array. The desired motor angles.
    """
    del current_time
    return self._pose + input_action

  def get_observation(self, input_observation):
    """Get the trajectory generator's observation."""

    return input_observation
