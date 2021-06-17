#!/usr/bin/env python3

"""
position, velocity and effort control for all revolute joints on the robot
"""

import rospy
from std_msgs.msg import Float64MultiArray


# NOTE: 2 classes are implemented here, scroll down to the next class (Control) to see the plugin!


class pveControl:
    """helper class to receive position, velocity or effort (pve) control commands"""

    def __init__(self, controller_type, namespace):
        """constructor
        Assumes joint_name is unique, creates multiple subscribers to receive commands
        controller_type - position, velocity or effort
        namespace - namespace of the robot
        """
        assert controller_type in ['position', 'velocity', 'effort']
        rospy.Subscriber(namespace + controller_type + '_controller/command',
                         Float64MultiArray, self.pve_controlCB, queue_size=1)
        self.cmd = 0.0
        self.data_available = False

    def pve_controlCB(self, msg):
        """position, velocity or effort callback
        msg - the msg passed by the ROS network via topic publication
        """
        self.data_available = True
        self.cmd = msg.data

    def get_last_cmd(self):
        """method to fetch the last received command"""
        self.data_available = False
        return self.cmd

    def get_is_data_available(self):
        """method to retrieve flag to indicate that a command has been received"""
        return self.data_available


# plugin is implemented below
class Control:
    def __init__(self, pybullet, robot):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # lists to recall last received command (useful when controlling multiple joints)
        self.position_joint_commands = [0] * len(self.robot.get_joint_indices())
        self.velocity_joint_commands = [0] * len(self.robot.get_joint_indices())
        self.effort_joint_commands = [0] * len(self.robot.get_joint_indices())
        # this parameter will be set for all robot joints
        max_effort = rospy.get_param('~max_effort', 100.0)
        # the max force to apply to the joint, used in velocity control
        self.force_commands = [max_effort] * len(self.robot.get_joint_indices())

        # setup subscribers
        self.pc_subscriber = pveControl('position', self.robot.get_namespace())
        self.vc_subscriber = pveControl('velocity', self.robot.get_namespace())
        self.ec_subscriber = pveControl('effort', self.robot.get_namespace())

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        """check if user has commanded a joint and forward the request to pybullet"""
        # flag to indicate there are pending position control tasks
        control_params = {"bodyUniqueId": self.robot.get_id(), "jointIndices": self.robot.get_joint_indices()}
        if self.pc_subscriber.get_is_data_available():
            control_params["controlMode"] = self.pb.POSITION_CONTROL
            control_params["targetPositions"] = self.pc_subscriber.get_last_cmd()
            control_params["forces"] = self.force_commands
        if self.vc_subscriber.get_is_data_available():
            control_params["controlMode"] = self.pb.VELOCITY_CONTROL
            control_params["targetVelocities"] = self.vc_subscriber.get_last_cmd()
            control_params["forces"] = self.force_commands
        if self.ec_subscriber.get_is_data_available():
            control_params["controlMode"] = self.pb.TORQUE_CONTROL
            control_params["forces"] = self.ec_subscriber.get_last_cmd()

        if "controlMode" in control_params.keys():
            self.pb.setJointMotorControlArray(**control_params)
