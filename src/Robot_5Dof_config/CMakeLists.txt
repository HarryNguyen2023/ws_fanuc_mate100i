cmake_minimum_required(VERSION 3.1.3)
project(Robot_5Dof_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pluginlib
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_ros_perception
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2_eigen
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  moveit_ros_move_group
  tf2_geometry_msgs
  DEPENDS
  EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_path_planning src/robot_5dof_path_planning.cpp)
target_link_libraries(robot_path_planning ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_5dof_robot_state src/robot_5dof_robot_state.cpp)
target_link_libraries(robot_5dof_robot_state ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_5dof_planning_scene src/robot_5dof_planning_scene.cpp)
target_link_libraries(robot_5dof_planning_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
