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
env.gripper_close()
print(env.robot.joints, env.robot.gripper_pose)
'''
if pose is x,y,z,rx,ry,rz, as euler angle, use tcp_pose_trans() to quat ,then move
if pose is x,y,z, wx,wy,wz,ww, as quat , it can be use directly without trans
env.movep() input should be quat
euler 3.14155060e+00, -1.94625667e-04,  3.14155505e+00 is gripper down rotation
'''

# pose = np.array([0.57,  0.2316,  0.85593, 0.456897, -0.515897,  0.614818,-0.383528])
# pose2 = np.array([0.4,  0.15,  0.5,  3.14155060e+00, -1.94625667e-04,  3.14155505e+00])
# pose3 = np.array([0.5,  0.0,  0.5,  3.14155060e+00, -1.94625667e-04,  3.14155505e+00])
# pose4 = np.array([0.615,  0.0,  0.3,  3.14155060e+00, -1.94625667e-04,  3.14155505e+00])
# pose5 = np.array([0.1,  -0.4,  0.4,  3.14155060e+00, -1.94625667e-04,  3.14155505e+00])

# # pose = tcp_pose_trans(pose)
# pose2 = tcp_pose_trans(pose2)
# pose3 = tcp_pose_trans(pose3)
# pose4 = tcp_pose_trans(pose4)
# pose5 = tcp_pose_trans(pose5)
pose_list = []
pose1 =  np.array([0.615,  0.0,  0.5,  3.14155060e+00, -1.94625667e-04,  3.14155505e+00])
for i in range(5):
    dice = random.randint(0,10)
    if dice > 2:
        pose = pose1 - np.array([0.02, 0 ,0 , 0,0,0])
    else:
        noise = np.random.uniform(-0.005, 0.005, size=3) 
        pose = pose1 
        pose[0:3] += noise
    pose_list.append(pose)

pose1 = tcp_pose_trans(pose1)
env.movep(pose1)

for pose in pose_list:
    pose = tcp_pose_trans(pose)
    print(f'pose:{pose}')
    env.movep(pose)
    time.sleep(0.02)
env.reset_to_home()
# env.movep(pose)
# # time.sleep(0.5)
# env.movep(pose2)
# # time.sleep(0.5)
# env.movep(pose3)
# # time.sleep(0.5)
# env.movep(pose4)
# # time.sleep(0.5)
# env.movep(pose5)
# # time.sleep(0.5)
# env.reset_to_home()
