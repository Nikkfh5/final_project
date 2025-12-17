cmake_minimum_required(VERSION 3.0.2)
project(sim_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  gazebo_ros
  rospy
  geometry_msgs
  gazebo_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs gazebo_ros rospy geometry_msgs gazebo_msgs
)

catkin_install_python(PROGRAMS
  scripts/move_robot.py
  scripts/cmd_vel_circle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  PATTERN ".svn" EXCLUDE
)

# Install model files
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  PATTERN ".svn" EXCLUDE
)
