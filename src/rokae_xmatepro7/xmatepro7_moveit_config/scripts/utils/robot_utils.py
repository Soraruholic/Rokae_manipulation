import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import CubicSpline
import geometry_msgs.msg
def cal_transformation_matrix(start, end):
    """
    计算从 start 到 end 的变换矩阵
    :param start: 六维数组 [x1, y1, z1, rx1, ry1, rz1]
    :param end: 六维数组 [x2, y2, z2, rx2, ry2, rz2]
    :return: 4x4 变换矩阵
    """
    x1, y1, z1, rx1, ry1, rz1 = start
    x2, y2, z2, rx2, ry2, rz2 = end

    # 计算初始和目标的旋转矩阵
    R1 = R.from_euler('xyz', [rx1,ry1,rz1],degrees=False).as_matrix()
    R2 =  R.from_euler('xyz', [rx2,ry2,rz2],degrees=False).as_matrix()

    # 计算平移向量
    translation = np.array([x2 - x1, y2 - y1, z2 - z1])

    # 构建 4x4 齐次变换矩阵
    T = np.eye(4)
    T[:3, :3] = R2 @ R1.T  # 确保旋转的组合方式正确
    T[:3, 3] = translation  # 平移部分
    
    return T

def move_matrix(target_position):
    # 计算变换矩阵
    transformation_matrix = cal_transformation_matrix(np.array([0,0,0,0,0,0]), np.asarray(target_position)) 
    
    return transformation_matrix

def rpy2quat(euler_angles):
    '''
        传出x,y,z,w顺序的四元数
    '''
    rotation = R.from_euler('xyz', euler_angles, degrees=False)
    quaternion = R.as_quat(rotation)
    return quaternion

def quat2rpy(quat):
    '''
        传入x,y,z,w顺序的四元数
    '''
    rotation_from_quat = R.from_quat(quat)
    euler_angles_converted = rotation_from_quat.as_euler('xyz', degrees=False)
    return euler_angles_converted

def tcp_pose_trans(tcp_pose):
    '''
        trans a rpy pose to quatation pose
    '''
    pos = tcp_pose[:3]
    rot = tcp_pose[3:]
    rot = rpy2quat(rot)
    return np.concatenate([pos, rot])

import math
def interpolate_points(points, max_diff=0.00022):
    if not points:
        return []
    
    interpolated = [points[0]]  # 初始点
    
    for i in range(len(points) - 1):
        current = points[i]
        next_p = points[i + 1]
        num_joints = len(current)
        
        # 计算各关节角度差
        deltas = [abs(next_p[j] - current[j]) for j in range(num_joints)]
        
        # 计算每个关节所需的分段数
        segments_per_joint = [math.ceil(delta / max_diff) if delta > 0 else 0 for delta in deltas]
        
        # 确定总分段数
        total_segments = max(segments_per_joint) if max(deltas) > 0 else 0
        
        if total_segments == 0:
            # 两点完全相同，无需插值
            interpolated.append(next_p)
            continue
        
        # 生成中间点
        for step in range(1, total_segments):
            ratio = step / total_segments
            new_point = [
                current[j] + (next_p[j] - current[j]) * ratio
                for j in range(num_joints)
            ]
            interpolated.append(new_point)
        
        interpolated.append(next_p)
    
    return interpolated

def cubic_spline(points,time_step, dt=0.001):
    # 假设有7个关节的轨迹点数据
    # 每个关节的轨迹点数量为n，时间间隔为0.001秒
    # 例如：5个时间点，每个时间点对应7个关节的角度
    points = np.asarray(points)
    joint_trajectories = points.T
    
    interpolated_trajectories = []
    # 对每个关节进行样条插值
    for i in range(7):
        cs = CubicSpline(time_step, joint_trajectories[i])
        # 生成新的时间戳
        new_timestamps = np.arange(time_step[0], time_step[-1], dt)
        # 计算插值
        interpolated_positions = cs(new_timestamps)
        interpolated_trajectories.append(interpolated_positions)

    interpolated_trajectories = np.asarray(interpolated_trajectories).T

    return np.asarray(interpolated_trajectories), new_timestamps