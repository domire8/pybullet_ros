cmake_minimum_required(VERSION 2.8.3)
project(pybullet_ros)

find_package(catkin REQUIRED COMPONENTS
    franka_panda_description
    std_msgs
)

catkin_python_setup()
