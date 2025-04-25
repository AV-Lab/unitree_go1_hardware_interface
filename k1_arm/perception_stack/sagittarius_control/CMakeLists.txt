cmake_minimum_required(VERSION 2.8.3)
project(sagittarius_control)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  sdk_sagittarius_arm 
  rosbag
  roscpp
  tf2_ros
  rviz
  sensor_msgs
  std_msgs
  std_srvs
  realsense2_camera
)


catkin_package(
 CATKIN_DEPENDS rosbag roscpp tf2_ros rviz sensor_msgs std_msgs std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rviz_joint_mirror_node src/rviz_joint_mirror_node.cpp)
target_link_libraries(rviz_joint_mirror_node ${catkin_LIBRARIES})
add_dependencies(rviz_joint_mirror_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  nodes/send_pre_pick_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/multi_color_object_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/move_to_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/move_to_object.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

