#!/usr/bin/env python3

"""
Query robot base pose and speed from pybullet and publish to /odom topic.
This component does not add any noise to it.
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class SimpleOdometry:
    def __init__(self, pybullet, robot):
        self._pb = pybullet
        self._robot = robot
        self._publisher = rospy.Publisher('odom', Odometry, queue_size=1)
        # save some overhead by setting some information only once
        self._odom_msg = Odometry()
        self._odom_msg.header.frame_id = rospy.get_param('~odom_frame', 'odom')
        self._odom_msg.child_frame_id = rospy.get_param('~robot_base_frame', 'base_link')
        self._br = tf2_ros.TransformBroadcaster()

    def execute(self):
        """
        Execute the plugin. This function is called from main update loop in the pybullet ros node.
        """
        self._odom_msg.header.stamp = rospy.Time.now()
        # query base state from robot and store in odom msg
        position, orientation, linear_velocity, angular_velocity = self._robot.get_base_state()
        [self._odom_msg.pose.pose.position.x,
         self._odom_msg.pose.pose.position.y,
         self._odom_msg.pose.pose.position.z] = position
        [self._odom_msg.pose.pose.orientation.x,
         self._odom_msg.pose.pose.orientation.y,
         self._odom_msg.pose.pose.orientation.z,
         self._odom_msg.pose.pose.orientation.w] = orientation
        [self._odom_msg.twist.twist.linear.x,
         self._odom_msg.twist.twist.linear.y,
         self._odom_msg.twist.twist.linear.z] = linear_velocity
        [self._odom_msg.twist.twist.angular.x,
         self._odom_msg.twist.twist.angular.y,
         self._odom_msg.twist.twist.angular.z] = angular_velocity
        self._publisher.publish(self._odom_msg)

        tf_msg = TransformStamped()
        tf_msg.header.frame_id = self._odom_msg.header.frame_id
        tf_msg.child_frame_id = self._odom_msg.child_frame_id
        tf_msg.transform.translation = self._odom_msg.pose.pose.position
        tf_msg.transform.rotation = self._odom_msg.pose.pose.orientation
        tf_msg.header.stamp = rospy.Time.now()
        self._br.sendTransform(tf_msg)
