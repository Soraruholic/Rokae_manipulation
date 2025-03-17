from .RobotController import RokaeControlInterface, RokaeReceiveInterface
from utils.robot_utils import move_matrix, rpy2quat, quat2rpy
import time
import threading
import numpy as np
class RokaexMatePro7:
    def __init__(self):
        self.controller = RokaeControlInterface()
        self.robot_receiver = RokaeReceiveInterface()

        # 线程锁
        self.contorl_lock = threading.Lock()
        self.gripper_lock = threading.Lock()
        print('robot init success!')

    def reset(self):
        time.sleep(1)
        print(f"robot init state:\n{self.robot_state}")

    @property
    def pose_home(self):
        return np.asarray([ -0.82212712,  0.78896482,  1.06670555,  1.22280679, -0.67345472, 1.48878204,  0.15069204])

    def get_robot_state(self):
        '''获取机器人和夹爪当前的状态并更新'''
        try:
            with self.contorl_lock:
                tcp_pose, joints_state, gripper_pose, gripper_state =  self.robot_receiver.getRobotState()
                self.robot_state['joints'] = joints_state
                self.robot_state['transformation_matrix'] = move_matrix(tcp_pose)
                self.robot_state['tcp_pose'] = tcp_pose
                self.robot_state['gripper_pose'] = gripper_pose
                self.robot_state['gripper_state'] = gripper_state
                # print("======================================================")
        except Exception as e:
            print("error when update robot state:" )
            print(e)

    @property
    def robot_state(self):
        try:
            with self.contorl_lock:
                tcp_pose, joints_state, gripper_pose, gripper_state =  self.robot_receiver.getRobotState()
                return {
                    "joints":joints_state,
                    "tcp_pose": tcp_pose,
                    "transformation_matrix": move_matrix(tcp_pose),
                    "gripper_pose":gripper_pose,
                    "gripper_state": gripper_state
                }
        except Exception as e:
            print("error when update robot state:" )
            print(e)
            return {
                "joints":np.zeros(7),
                "tcp_pose": np.zeros(6),
                "transformation_matrix": np.eye(4),
                "gripper_pose":np.zeros(1),
                "gripper_state": None
            }

    @property
    def joints(self):
        return self.robot_state['joints']
    
    @property
    def tcp_pose(self):
        return self.robot_state['tcp_pose']
    
    @property
    def pose_matrix(self):
        return self.robot_state['transformation_matrix']
    
    @property
    def gripper_contact(self):
        if self.gripper_state == 'caught':
            return True
        elif self.gripper_state == 'drop':
            print("item drop!")
            return False
        elif self.gripper_state == 'moving':
            return False

    @property
    def gripper_state(self):
        return self.robot_state['gripper_state']
    
    @property
    def gripper_pose(self):
        return self.robot_state['gripper_pose']

    @property
    def position(self):
        return self.tcp_pose[:3]
    
    @property
    def rotation(self):
        return self.tcp_pose[3:]

    @property
    def tcp_pose_quat(self):
        quat = rpy2quat(self.rotation)
        pos = self.position
        return np.concatenate([pos, quat])

    def send_cmd(self, cmd):
        assert isinstance(cmd, dict)
        self.controller.sendCmd(cmd)
