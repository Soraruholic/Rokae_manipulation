# Rokae_manipulation
a repo for roake_xmatepro7 robot manipulation based on rokae_imitation and moveit
1.首先安装对应版本的ros组件
编译
```
catkin_make clean
catkin_make
```
2.安装rokae_imitation并编译
```
git clone https://github.com/destroy314/rokae_imitation.git
cd build
cmake ..
make
```
进入 src/rokae_xmatepro7/xmatepro7_moveit_config
启动流程参考start.txt
