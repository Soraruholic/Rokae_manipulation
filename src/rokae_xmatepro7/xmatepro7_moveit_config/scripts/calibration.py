from Robotic.Rokae import RokaexMatePro7
import random
import time
from Env.BaseEnv import BaseEnv
import numpy as np
from utils.robot_utils import tcp_pose_trans
from Robotic.RosNode import RosMoveitPlanner

robot = RokaexMatePro7()
planner = RosMoveitPlanner()
env = BaseEnv(robot=robot, motion_planner=planner)
env.reset()

pose1 = np.array([0.57,  0.2316,  0.85593, 0.456897, -0.515897,  0.614818,-0.383528])
pose2 = np.array([0.4,  0.15,  0.5,  3.14155060e+00, -1.94625667e-04,  3.14155505e+00])
pose3 = np.array([0.5,  0.0,  0.5,  3.14155060e+00, -1.94625667e-04,  3.14155505e+00])
pose4 = np.array([0.615,  0.0,  0.3,  3.14155060e+00, -1.94625667e-04,  3.14155505e+00])
pose5 = np.array([0.1,  -0.4,  0.4,  3.14155060e+00, -1.94625667e-04,  3.14155505e+00])

pose1 = tcp_pose_trans(pose1)
pose2 = tcp_pose_trans(pose2)
pose3 = tcp_pose_trans(pose3)
pose4 = tcp_pose_trans(pose4)
pose5 = tcp_pose_trans(pose5)

pose_list = [pose1, pose2, pose3, pose4, pose5]

for pose in pose_list:
    env.movep(pose)
    time.sleep(10)



