#!/usr/bin/env python3

"""
position, velocity and effort control for all revolute joints on the robot
"""

import rospy
from std_msgs.msg import Float64MultiArray


# NOTE: 2 classes are implemented here, scroll down to the next class (Control) to see the plugin!


class pveControl:
    """helper class to receive position, velocity or effort (pve) control commands"""

    def __init__(self, controller_type):
        """constructor
        Assumes joint_name is unique, creates multiple subscribers to receive commands
        controller_type - position, velocity or effort
        """
        assert controller_type in ['position', 'velocity', 'effort']
        rospy.Subscriber("/" + controller_type + '_controller/command',
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
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # get joints names and store them in dictionary, combine both revolute and prismatic dic
        joint_index_name_dic = {**kargs['rev_joints'], **kargs['prism_joints']}
        self.joint_indices = [joint for joint in joint_index_name_dic]
        # lists to recall last received command (useful when controlling multiple joints)
        self.position_joint_commands = [0] * len(self.joint_indices)
        self.velocity_joint_commands = [0] * len(self.joint_indices)
        self.effort_joint_commands = [0] * len(self.joint_indices)
        # this parameter will be set for all robot joints
        if rospy.has_param('~max_effort_vel_mode'):
            rospy.logwarn('max_effort_vel_mode parameter is deprecated, please use max_effort instead')
            # kept for backwards compatibility, delete after some time
            max_effort = rospy.get_param('~max_effort_vel_mode', 100.0)
        else:
            max_effort = rospy.get_param('~max_effort', 100.0)
        # the max force to apply to the joint, used in velocity control
        self.force_commands = [max_effort] * len(self.joint_indices)
        # setup subscribers
        self.pc_subscriber = pveControl('position')
        self.vc_subscriber = pveControl('velocity')
        self.ec_subscriber = pveControl('effort')

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        """check if user has commanded a joint and forward the request to pybullet"""
        # flag to indicate there are pending position control tasks
        control_params = {"bodyUniqueId": self.robot, "jointIndices": self.joint_indices}
        command_available = False
        if self.pc_subscriber.get_is_data_available():
            control_params["controlMode"] = self.pb.POSITION_CONTROL
            control_params["targetPositions"] = self.pc_subscriber.get_last_cmd()
            control_params["forces"] = self.force_commands
            command_available = True
        if self.vc_subscriber.get_is_data_available():
            control_params["controlMode"] = self.pb.VELOCITY_CONTROL
            control_params["targetVelocities"] = self.vc_subscriber.get_last_cmd()
            control_params["forces"] = self.force_commands
            command_available = True
        if self.ec_subscriber.get_is_data_available():
            control_params["controlMode"] = self.pb.TORQUE_CONTROL
            control_params["forces"] = self.ec_subscriber.get_last_cmd()
            command_available = True

        if command_available:
            self.pb.setJointMotorControlArray(**control_params)
