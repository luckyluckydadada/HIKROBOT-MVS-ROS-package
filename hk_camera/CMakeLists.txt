cmake_minimum_required(VERSION 3.0.2)
project(hk_camera)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES 
 CATKIN_DEPENDS image_transport roscpp sensor_msgs std_msgs std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(/opt/MVS/lib/64)
SET(CMAKE_INSTALL_RPATH /opt/MVS/lib/64)
add_executable(${PROJECT_NAME}_node src/hk_camera_node.cpp src/hk_camera.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  MvCameraControl
)

