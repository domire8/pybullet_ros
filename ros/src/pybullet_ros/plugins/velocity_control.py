#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray


class VelocityControlInterface:
    """
    Velocity control class to receive  commands.

    Available methods (for usage, see documentation at function definition):
        - get_last_cmd
        - new_command_available
    """

    def __init__(self, nb_joints):
        """
        Assumes joint_name is unique, creates multiple subscribers to receive commands
        joint_index - stores an integer joint identifier
        joint_name - string with the name of the joint as described in urdf model
        controller_type - position, velocity or effort
        """
        rospy.Subscriber('velocity_controller/command',
                         Float64MultiArray, self._callback, queue_size=1)
        self._cmd = [0.0] * nb_joints
        self._data_available = False

    def _callback(self, msg):
        """
        Effort callback

        :param msg: The message passed by ROS topics
        :type msg: Float64MultiArray
        """
        if len(msg.data) == len(self._cmd):
            self._data_available = True
            self._cmd = list(msg.data)
        else:
            rospy.logwarn("[VelocityController::callback] Ignoring message, incorrect number of joints.")

    def get_last_cmd(self):
        """
        Get the last received command.

        :return: Most recently received command
        :rtype: list of float
        """
        self._data_available = False
        return self._cmd

    def new_command_available(self):
        """
        Get boolean that indicates if a command has been received.

        :return: Bool indicating if new command is available
        :rtype: bool
        """
        return self._data_available


class VelocityControl:
    """
    Effort control plugin.
    """

    def __init__(self, pybullet, robot, **kargs):
        self._robot = robot
        self.subscriber = VelocityControlInterface(self._robot.get_nb_movable_joints())
        self._robot.set_joint_velocities_cmd([0] * self._robot.get_nb_movable_joints(), effort=100)

    def execute(self):
        """
        This function gets called from PyBulletROSWrapper main update loop.
        Check if user has sent a new command and apply it on the robot.
        """
        if self.subscriber.new_command_available():
            velocity_joint_command = self.subscriber.get_last_cmd()
        else:
            velocity_joint_command = [0] * self._robot.get_nb_movable_joints()
        self._robot.set_joint_velocities_cmd(velocity_joint_command, effort=100)
