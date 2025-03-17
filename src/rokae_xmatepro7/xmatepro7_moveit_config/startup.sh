#!/bin/bash

# 启动 C++ 文件
echo "Starting C++ application..."
sudo /home/vector/workspace/rokae_imitation/build/all_control joint_pose
if [ $? -ne 0 ]; then
    echo "Failed to start C++ application."
    exit 1
fi

# 启动 ROS launch 文件
echo "Starting ROS launch file..."
source /home/vector/workspace/rokae_ws/devel/setup.bash
roslaunch xmatepro7_moveit_config demo.launch &
if [ $? -ne 0 ]; then
    echo "Failed to start ROS launch file."
    exit 1
fi
sleep 2
# 启动 Python 脚本
echo "Starting Python script..."
python /home/vector/workspace/rokae_ws/src/rokae_xmatepro7/xmatepro7_moveit_config/scripts/ros_pub.py
if [ $? -ne 0 ]; then
    echo "Failed to start Python script."
    exit 1
fi

echo "All processes started successfully."
但是第一个cpp程序和后续程序都会阻塞终端
