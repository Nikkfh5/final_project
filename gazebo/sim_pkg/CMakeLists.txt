cmake_minimum_required(VERSION 3.0.2)
project(sim_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  gazebo_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs gazebo_ros
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

