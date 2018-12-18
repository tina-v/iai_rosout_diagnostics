cmake_minimum_required(VERSION 2.8.3)
project(drive_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    )

add_service_files(DIRECTORY srv
    FILES
    GetStatistics.srv
    GetLogger.srv
    )

add_message_files(DIRECTORY msg
    FILES
    SpeakingNumber.msg
    )

generate_messages(DEPENDENCIES
    std_msgs
    )

catkin_package(CATKIN_DEPENDS
    message_runtime
    std_msgs
    )
