cmake_minimum_required(VERSION 3.0.2)
project(mctilt_offboard)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  rospy
  std_msgs
  tf
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/offboard_figure8.py
  scripts/offboard_pitch_hold.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
