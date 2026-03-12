cmake_minimum_required(VERSION 3.0.2)
project(robocar_apriltag_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  apriltag_ros
)

catkin_package()

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/apriltag_navigator.py
  scripts/serial_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files and config
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
