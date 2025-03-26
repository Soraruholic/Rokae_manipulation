#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander
from geometry_msgs.msg import Pose
from .RobotController import RokaeReceiveInterface
import numpy as np
from utils.robot_utils import interpolate_points, cubic_spline

class RosMoveitPlanner:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('moveit_trajectory_example', anonymous=True)
        # 初始化 MoveIt! Commander
        moveit_commander.roscpp_initialize(sys.argv)
        # 创建机器人和移动组的对象
        self.robot = RobotCommander()
        self.move_group = MoveGroupCommander("xmatepro7")  # 替换为您的移动组名称
            # 设置规划时间
        self.move_group.set_planning_time(5.0)  # 设置最大规划时间为 5 秒

        # 设置速度和加速度比例
        self.move_group.set_max_velocity_scaling_factor(0.1)  # 设置速度比例为 50%
        self.move_group.set_max_acceleration_scaling_factor(0.1)  # 设置加速度比例为 50%

    def extract(self, trajectory):
        trajectory_points = []
        time_step = []
        for point in trajectory.joint_trajectory.points:
            point_data =  point.positions
            time_data = point.time_from_start.secs + point.time_from_start.nsecs * 1e-9
            trajectory_points.append(point_data)
            time_step.append(time_data)
        return trajectory_points, time_step

    def smooth_path(self, trajectory_points, time_step):
        path = cubic_spline(points=trajectory_points, time_step=time_step)
        return path

    def plan_qpos(self, qpos):
        '''
            Pose必须是(x,y,z,w)的四元数
        '''
        self.move_group.set_joint_value_target(qpos)
        plan = self.move_group.plan()

        traj_points, time_step= self.extract(plan[1])
        
        if plan[0]:
            traj = cubic_spline(traj_points, time_step)[0]
            # return self.smooth_path(traj_points)
            return traj.tolist()
        return False

    def plan(self, pose):
        # 设置目标位姿
        '''
            Pose必须是(x,y,z,w)的四元数
        '''
        target_pose = Pose()
        target_pose.position.x = pose[0]
        target_pose.position.y = pose[1]
        target_pose.position.z = pose[2]
        target_pose.orientation.x = pose[3]
        target_pose.orientation.y = pose[4]
        target_pose.orientation.z = pose[5]
        target_pose.orientation.w = pose[6]

        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.plan()
        
        traj_points, time_step= self.extract(plan[1])
        
        if plan[0]:
            traj = cubic_spline(traj_points, time_step)[0]
            # return self.smooth_path(traj_points)
            return traj.tolist()
        return False

    def plan_realtime(self, pose, time_step):
        # 设置目标位姿
        '''
            Pose必须是(x,y,z,w)的四元数
        '''
        target_pose = Pose()
        target_pose.position.x = pose[0]
        target_pose.position.y = pose[1]
        target_pose.position.z = pose[2]
        target_pose.orientation.x = pose[3]
        target_pose.orientation.y = pose[4]
        target_pose.orientation.z = pose[5]
        target_pose.orientation.w = pose[6]

        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.plan()
        
        traj_points, _= self.extract(plan[1])
        
        if plan[0]:
            traj = cubic_spline(traj_points, time_step)[0]
            # return self.smooth_path(traj_points)
            return traj.tolist()
        return False

from sensor_msgs.msg import JointState

class RosPublisher:
    def __init__(self):
        # 初始化 ROS 节点
        self.robot_receiver = RokaeReceiveInterface()
        rospy.init_node('joint_state_publisher', anonymous=True)
        
        # 创建 Publisher
        self.publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # 设置发布频率
        self.rate = rospy.Rate(100)  # 10 Hz

    def publish_joint_states(self):
        joint_state_msg = JointState()
        
        while not rospy.is_shutdown():
            # 填充 JointState 消息
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5','joint6', 'joint7']  # 替换为实际关节名称
            tcp_pose_np, joints_state_np, gripper_pose, gripper_state = self.robot_receiver.getRobotState()
            joint_state_msg.position = joints_state_np.tolist()
            # 发布消息
            self.publisher.publish(joint_state_msg)
            # 等待下一次发布
            self.rate.sleep()

