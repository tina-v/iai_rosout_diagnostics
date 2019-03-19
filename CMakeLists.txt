cmake_minimum_required(VERSION 2.8.3)
project(iai_rosout_diagnostics)

find_package(catkin REQUIRED COMPONENTS diagnostic_msgs rosgraph_msgs rospy)

catkin_package(CATKIN_DEPENDS diagnostic_msgs rosgraph_msgs rospy)

catkin_install_python(PROGRAMS nodes/rosout_diagnostics nodes/test_node
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
