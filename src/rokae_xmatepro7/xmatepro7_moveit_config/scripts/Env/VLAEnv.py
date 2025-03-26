from Env.BaseEnv import BaseEnv
from Robotic.Rokae import RokaexMatePro7
from Robotic.RosNode import RosMoveitPlanner
from utils.robot_utils import rpy2quat, quat2rpy, interpolate_points
import numpy as np
import time
class VLAEnv(BaseEnv):
    def __init__(self,  robot: RokaexMatePro7, motion_planner:RosMoveitPlanner):
        super().__init__(robot, motion_planner)

    def apply_action_p(self, action, time_step, threshold = 1e-2):
        '''
            time_step should be a list corresponds to action
        '''
        assert isinstance(action, (tuple, list, np.ndarray))
        pose = action[:7]
        width = action[-1]
        trajectories = self.motion_planner.plan_realtime(pose, time_step)

        delta_width = width-self.robot.gripper_pose
        gripper_velocity = 1.0
        if delta_width < 0:
            gripper_velocity = -1.0
            
        if trajectories is not False:
            for traj in trajectories:
                if abs(width-self.robot.gripper_pose)<=threshold:
                    gripper_velocity = 0
                msg = {
                    "joint_position": list(traj),
                    'gripper_velocity' : gripper_velocity
                }
                self.robot.send_cmd(msg)
                timestep = time.time()
                self.robot.send_cmd(msg)
                transtime = (time.time() - timestep)
                sleep_time = max(0.001-transtime, 0)
                time.sleep(sleep_time)