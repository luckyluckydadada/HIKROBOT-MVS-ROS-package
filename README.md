# HIKROBOT-MVS-ROS-package
海康威视工业相机sdk的ros驱动包，照片已经转码为rgb格式，方便算法开发。。

# Install
mkdir ws_hk_mvs_ros
git clone https://github.com/luckyluckydadada/HIKROBOT-MVS-ROS-package.git ws_hk_mvs_ros/src
cd ws_hk_mvs_ros
catkin_make

# 运行node
需要先启动roscore,打开rviz。
source ./devel/setup.bash 
rosrun hk_camera hk_camera_node
用 riz 订阅 /hk_camera/image_raw 查看照片。

# 运行node 并展示照片
source ./devel/setup.bash 
roslaunch hk_camera hk_camera_view.launch

