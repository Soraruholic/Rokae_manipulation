启动顺序
sudo /home/vector/workspace/rokae_imitation/build/all_control joint_pose

source ~/workspace/rokae_ws/devel/setup.bash
roslaunch xmatepro7_moveit_config demo.launch

conda activate ros
python scripts/ros_pub.py
