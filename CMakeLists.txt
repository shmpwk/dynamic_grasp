cmake_minimum_required(VERSION 2.8.3)
project(soft_grasp)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    message_generation
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS
    actionlib_msgs
    message_runtime
)

