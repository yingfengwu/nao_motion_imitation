"""
@author: yingfengwu
time: 2021/06/26

This file is to transform the angle of joint to the radian of joint and delete some unused columns
"""
import pandas as pd
import numpy as np
from tqdm import tqdm
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(0, parentdir)

path_pos = "../bvh_converter/0005_2FeetJump001_worldpos.csv"
path_csv = "../data/nao_jump_pos.csv"

data = pd.read_csv(path_pos)

Keys=['Spine.X', 'Spine.Y', 'Spine.Z', 'Spine1.X', 'Spine1.Y', 'Spine1.Z',
      'RightHand.X', 'RightHand.Y', 'RightHand.Z', 'RightHandEnd.X',
      'RightHandEnd.Y', 'RightHandEnd.Z', 'RightHandThumb.X',
      'RightHandThumb.Y', 'RightHandThumb.Z', 'RightHandThumbEnd.X',
      'RightHandThumbEnd.Y', 'RightHandThumbEnd.Z', 'LeftHand.X', 'LeftHand.Y',
      'LeftHand.Z',	'LeftHandEnd.X', 'LeftHandEnd.Y', 'LeftHandEnd.Z',
      'LeftHandThumb.X', 'LeftHandThumb.Y',	'LeftHandThumb.Z',
      'LeftHandThumbEnd.X',	'LeftHandThumbEnd.Y', 'LeftHandThumbEnd.Z',
      'HeadEnd.X', 'HeadEnd.Y', 'HeadEnd.Z', 'RightToeBase.X', 'RightToeBase.Y',
      'RightToeBase.Z', 'RightToeBaseEnd.X', 'RightToeBaseEnd.Y', 'RightToeBaseEnd.Z',
      'LeftToeBase.X', 'LeftToeBase.Y', 'LeftToeBase.Z', 'LeftToeBaseEnd.X',
      'LeftToeBaseEnd.Y', 'LeftToeBaseEnd.Z'
      ]

for v in tqdm(range(len(Keys))):  # delete some unused columns
    data = data.drop(Keys[v], axis=1)

for i in tqdm(range(data.shape[0])):  # transform the angle of joint to the radian of joint
    for j in range(data.shape[1]):
        if j != 0:
            data.values[i][j] = data.values[i][j] * np.pi / 180

data.to_csv(path_csv)
print("Done!")

