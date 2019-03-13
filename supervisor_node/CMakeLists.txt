cmake_minimum_required(VERSION 2.8.3)
project(supervisor_node)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    )

catkin_package(CATKIN_DEPENDS
    message_runtime
    std_msgs
    )

catkin_install_python(PROGRAMS
  script/supervisor.py
  script/testNode.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/script
)
