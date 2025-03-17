from Robotic.Rokae import RokaexMatePro7
from Robotic.RosNode import RosMoveitPlanner
from utils.robot_utils import rpy2quat, quat2rpy, interpolate_points
import numpy as np
import time
class BaseEnv:
    robot = None
    def __init__(self, robot: RokaexMatePro7, motion_planner:RosMoveitPlanner):
        self.robot = robot
        self.motion_planner = motion_planner
        self.reset_to_home()

    def reset(self):
        self.robot.reset()
    
    def movep(self, pose):
        '''
            pose should be like [x, y, z, w, x, y, z]
        '''
        assert isinstance(pose, (tuple, list, np.ndarray))
        trajectories = self.motion_planner.plan(pose)
        if trajectories is not False:
            for traj in trajectories:
                msg = {
                    "joint_position": list(traj)
                }
                self.robot.send_cmd(msg)
                time.sleep(0.00099)

    def movej(self, target_qpos):
        '''
            qpos should be like [j1,j2,...,jn]
        '''
        assert isinstance(target_qpos, (tuple, list, np.ndarray))
        trajectories = self.motion_planner.plan_qpos(target_qpos)
        if trajectories is not False:
            for traj in trajectories:
                msg = {
                    "joint_position": list(traj)
                }
                self.robot.send_cmd(msg)
                time.sleep(0.00099)
    
    def gripper_control(self, width, frequency=50, threshold=1.5e-2, use_all_control=True):
        '''
            使用归一化的宽度控制夹爪位置
        '''
        # 使用归一化的宽度
        assert 0<=width<=1
        dt = 1.0 / frequency

        delta_width = width-self.robot.gripper_pose
        gripper_velocity = 1.0
        if delta_width < 0:
            gripper_velocity = -1.0
        msg = {
            'gripper_velocity' : gripper_velocity
        }
        while(abs(width-self.robot.gripper_pose)>threshold):
            if use_all_control:
                self.robot.send_cmd(msg)
            else:
                self.robot.send_cmd(msg)
            if gripper_velocity<0 and self.robot.gripper_contact:
                break
            time.sleep(dt)

    def gripper_close(self):
        self.gripper_control(0.1)
    
    def gripper_open(self):
        self.gripper_control(1.0)

    def reset_to_home(self):
        trajectories = self.motion_planner.plan_qpos(self.robot.pose_home)
        if trajectories is not False:
            for traj in trajectories:
                msg = {
                    "joint_position": list(traj)
                }
                self.robot.send_cmd(msg)
                time.sleep(0.00099)