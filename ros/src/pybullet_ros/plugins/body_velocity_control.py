#!/usr/bin/env python3

"""
Subscribe to /robot_name/cmd_vel and apply desired speed to the robot, without any noise.

transformations explained:
pybullet requires that velocity of the robot is set w.r.t. world reference frame
however cmd_vel convention required velocity to be expressed w.r.t. robot base frame
therefore a transformation is needed.
"""

import math

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Twist


class CmdVelCtrl:
    def __init__(self, pybullet, robot):
        self._pb = pybullet
        self._robot = robot
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._cmd_vel_msg = None
        self._received_cmd_vel_time = None
        # subscribe to robot velocity commands
        rospy.Subscriber(self._robot.namespace + "cmd_vel", Twist, self._cmd_vel_cb)

    @staticmethod
    def _from_rotation(quaternion):
        """Copied from tf (listener.py)"""
        epsilon = np.finfo(float).eps * 4.0
        q = np.array(quaternion, dtype=np.float64, copy=True)
        nq = np.dot(q, q)
        if nq < epsilon:
            return np.identity(4)
        q *= math.sqrt(2.0 / nq)
        q = np.outer(q, q)
        return np.array((
            (1.0 - q[1, 1] - q[2, 2], q[0, 1] - q[2, 3], q[0, 2] + q[1, 3]),
            (q[0, 1] + q[2, 3], 1.0 - q[0, 0] - q[2, 2], q[1, 2] - q[0, 3]),
            (q[0, 2] - q[1, 3], q[1, 2] + q[0, 3], 1.0 - q[0, 0] - q[1, 1])
        ), dtype=np.float64)

    def _cmd_vel_cb(self, msg):
        """Callback to receive velocity commands."""
        self._cmd_vel_msg = msg
        self._received_cmd_vel_time = rospy.Time.now()

    def execute(self):
        """
        Execute the plugin. This function is called from main update loop in the pybullet ros node.
        """
        if not self._cmd_vel_msg:
            return
        # check if timestamp is recent
        if (rospy.Time.now() - rospy.Duration(0.5)) > self._received_cmd_vel_time:
            return

        # transform twist from base_link to odom (pybullet allows to set velocity only on world frame)
        try:
            transform = self._tf_buffer.lookup_transform("odom", "base_link", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logerr("[CmdVelControl::execute] " + ex)
            return
        rotation = transform.transform.rotation
        transformation_matrix = self._from_rotation([rotation.x, rotation.y, rotation.z, rotation.w])

        linear_velocity = [self._cmd_vel_msg.linear.x, self._cmd_vel_msg.linear.y, self._cmd_vel_msg.linear.z]
        angular_velocity = [self._cmd_vel_msg.angular.x, self._cmd_vel_msg.angular.y, self._cmd_vel_msg.angular.z]
        linear_velocity_in_odom = np.dot(transformation_matrix, linear_velocity)
        angular_velocity_in_odom = np.dot(transformation_matrix, angular_velocity)

        self._pb.resetBaseVelocity(self._robot.id, linear_velocity_in_odom, angular_velocity_in_odom)
