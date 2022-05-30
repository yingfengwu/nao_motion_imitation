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

"""Utility functions to calculate Nao's pose and motor angles."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import attr

NAO_DEFAULT_ABDUCTION_ANGLE = 0
NAO_DEFAULT_HIP_ANGLE = 0
NAO_DEFAULT_KNEE_ANGLE = 0

@attr.s
class NaoPose(object):
  # Default pose of the nao.
  HeadYaw_angle = attr.ib(type=float, default=0)
  HeadPitch = attr.ib(type=float, default=0)
  LHipYawPitch_angle = attr.ib(type=float, default=0)
  LHipRoll_angle = attr.ib(type=float, default=0)
  LHipPitch_angle = attr.ib(type=float, default=0)
  LKneePitch_angle = attr.ib(type=float, default=0)
  LAnklePitch_angle = attr.ib(type=float, default=0)
  LAnkleRoll_angle = attr.ib(type=float, default=0)
  RHipYawPitch_angle = attr.ib(type=float, default=0)
  RHipRoll_angle = attr.ib(type=float, default=0)
  RHipPitch_angle = attr.ib(type=float, default=0)
  RKneePitch_angle = attr.ib(type=float, default=0)
  RAnklePitch_angle = attr.ib(type=float, default=0)
  RAnkleRoll_angle = attr.ib(type=float, default=0)
  LShoulderPitch_angle = attr.ib(type=float, default=0)
  LShoulderRoll_angle = attr.ib(type=float, default=0)
  LElbowYaw_angle = attr.ib(type=float, default=0)
  LElbowRoll_angle = attr.ib(type=float, default=0)
  LWristYaw_angle = attr.ib(type=float, default=0)
  LHand_angle = attr.ib(type=float, default=0)
  RShoulderPitch_angle = attr.ib(type=float, default=0)
  RShoulderRoll_angle = attr.ib(type=float, default=0)
  RElbowYaw_angle = attr.ib(type=float, default=0)
  RElbowRoll_angle = attr.ib(type=float, default=0)
  RWristYaw_angle = attr.ib(type=float, default=0)
  RHand_angle = attr.ib(type=float, default=0)
