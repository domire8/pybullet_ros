#!/usr/bin/env python3

"""
position, velocity and effort control for all revolute joints on the robot
"""

import rospy
from std_msgs.msg import Float64MultiArray

# NOTE: 2 classes are implemented here, scroll down to the next class (Control) to see the plugin!


class effortControl:
    """helper class to receive position, velocity or effort (pve) control commands"""
    def __init__(self, nb_joints):
        """constructor
        Assumes joint_name is unique, creates multiple subscribers to receive commands
        joint_index - stores an integer joint identifier
        joint_name - string with the name of the joint as described in urdf model
        controller_type - position, velocity or effort
        """
        rospy.Subscriber('effort_controller/command',
                         Float64MultiArray, self.effort_controlCB, queue_size=1)
        self.cmd = [0.0] * nb_joints
        self.data_available = False

    def effort_controlCB(self, msg):
        """position, velocity or effort callback
        msg - the msg passed by the ROS network via topic publication
        """
        assert(len(msg.data) == len(self.cmd))
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
class EffortControl:
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # get joints names and store them in dictionary, combine both revolute and prismatic dic
        joint_index_name_dic = {**kargs['rev_joints'], **kargs['prism_joints']}
        self.joint_indices = []
        for joint_index in joint_index_name_dic:
            self.joint_indices.append(joint_index)
        self.nb_joints = len(self.joint_indices)
        # setup subscribers
        self.ec_subscriber = effortControl(self.nb_joints)

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        """check if user has commanded a joint and forward the request to pybullet"""
        # flag to indicate there are pending position control tasks
        if self.ec_subscriber.get_is_data_available():
            effort_joint_command = self.ec_subscriber.get_last_cmd()
            self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
                                              controlMode=self.pb.VELOCITY_CONTROL, forces=effort_joint_command)
