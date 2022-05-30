# nao_motion_imitation

This is a code of imitating motion of human on NAO robot and this code is based on this paper:

Peng, Xue & Coumans, Erwin & Zhang, Tingnan & Lee, Tsang-Wei & Tan, Jie & Levine, Sergey. (2020). Learning Agile Robotic Locomotion Skills by Imitating Animals.https://arxiv.org/pdf/2004.00784.pdf


## First step:

Download the bvh file from https://mocap.cs.sfu.ca/, like 0005_2FeetJump001.bvh, and run "python __main__.py 0005_2FeetJump001.bvh" in retarget_motion/bvh_converter/ in terminal to convert the bvh data into 3D data, like 0005_2FeetJump001_worldpos.csv

## Second step:

run the angle2radian.py in retarget_motion/util/ to convert angle into radian and delete some unused columns (output file nao_jump_pos.csv).

## Third step:

run the run_nao_retarget.py in retarget_motion/ to acquire motion of the target joint angle (output file retarget_motion_nao_jump.txt).

## Forth step:

run the run.py in motion_imitation/ to train or test.

## issue:

1. Maybe lack of some .urdf files, you should download them online or find the absolute path in pybullet package.
