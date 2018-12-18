cmake_minimum_required(VERSION 2.8.3)
project(monitor_nodes)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    drive_msgs
    std_msgs
    )

generate_messages(DEPENDENCIES
    drive_msgs
    )

catkin_package(CATKIN_DEPENDS
    message_runtime
    drive_msgs
    std_msgs
    )


catkin_install_python(PROGRAMS
  script/dummyDrive.py
  script/getStatistics.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/script
)

