cmake_minimum_required(VERSION 2.8.3)
project(pybullet_ros)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    tf2_ros
    franka_panda_description
    cv_bridge
)

catkin_python_setup()
