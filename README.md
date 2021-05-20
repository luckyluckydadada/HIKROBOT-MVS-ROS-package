# HIKROBOT-MVS-ROS-package
海康威视工业相机sdk的ros驱动包。
mkdir -p hk_camera_ws/src
git clone hk_camera_ws/src
cd hk_camera_ws
catkin_make
source ./devel/setup.bash
rosrun hk_camera hk_camera

catkin_create_pkg hk_camera roscpp std_msgs  sensor_msgs std_srvs image_transport
