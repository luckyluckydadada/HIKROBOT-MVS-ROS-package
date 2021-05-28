# 此仓库不再维护， 请转到新库：https://github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS

# HIKROBOT-MVS-ROS-package
海康威视工业相机sdk的ros驱动包，照片已经转码为rgb格式，方便算法开发。

# Install
```
mkdir ws_hk_mvs_ros
git clone https://github.com/luckyluckydadada/HIKROBOT-MVS-ROS-package.git ws_hk_mvs_ros/src
cd ws_hk_mvs_ros
catkin_make
```
# 直接运行node
需要先启动roscore。
```
source ./devel/setup.bash 
rosrun hk_camera hk_camera_node
```
# launch启动node
```
source ./devel/setup.bash 
roslaunch hk_camera hk_camera.launch
```
# launch启动node和rviz node
用 rviz 订阅 /hk_camera_node/image_raw 查看照片
```
source ./devel/setup.bash 
roslaunch hk_camera hk_camera_rviz.launch
```
