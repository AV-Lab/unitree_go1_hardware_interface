cmake_minimum_required(VERSION 3.0.2)
project(sagittarius_control)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
  ${CMAKE_CURRENT_SOURCE_DIR}/../sagittarius_sdk
  ${CMAKE_CURRENT_SOURCE_DIR}/../sagittarius_sdk/sdk_sagittarius_arm
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../sagittarius_sdk/lib/x86_64
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

add_executable(arm_controller src/arm_controller.cpp)
add_executable(gripper_controller src/gripper_controller.cpp)

target_link_libraries(arm_controller
  ${catkin_LIBRARIES}
  sagittarius_sdk
  pthread
  ${Boost_LIBRARIES}
)

target_link_libraries(gripper_controller
  ${catkin_LIBRARIES}
  sagittarius_sdk
  pthread
  ${Boost_LIBRARIES}
)

install(TARGETS arm_controller gripper_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)