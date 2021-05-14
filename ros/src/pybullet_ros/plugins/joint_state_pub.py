#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState


class JointStatePub:
    """
    Joint state publisher plugin.
    """

    def __init__(self, pybullet, robot, **kargs):
        self._robot = robot
        self._pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size=1)
        self._seq = 1

    def execute(self):
        """
        This function gets called from PyBulletROSWrapper main update loop.
        Publish the joint state of the robot.
        """
        message = JointState()
        state = self._robot.get_joint_state()
        message.name = state.joint_names
        message.position = state.joint_positions
        message.velocity = state.joint_velocities
        message.effort = state.joint_efforts
        message.header.stamp = rospy.Time.now()
        message.header.seq = self._seq
        self._seq = self._seq + 1
        self._pub_joint_states.publish(message)
