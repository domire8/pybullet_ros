#!/usr/bin/env python3

"""
query robot state and publish position, velocity and effort values to /joint_states
"""

import rospy
from sensor_msgs.msg import JointState


class joinStatePub:
    def __init__(self, pybullet, robot):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # register this node in the network as a publisher in /joint_states topic
        self.pub_joint_states = rospy.Publisher(robot.get_namespace() + "joint_states", JointState, queue_size=1)
        print(robot.get_namespace())

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        # get joint state msg directly from robot
        joint_msg = self.robot.get_joint_state_msg()
        # update msg time using ROS time API
        joint_msg.header.stamp = rospy.Time.now()
        # publish joint states to ROS
        self.pub_joint_states.publish(joint_msg)
