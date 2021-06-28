#!/usr/bin/env python3

"""
Position, velocity and effort control for all joints on the robot
"""

import time

import rospy
from std_msgs.msg import Float64MultiArray


class PosVelEffControl:
    """Helper class to receive position, velocity or effort control commands"""

    def __init__(self, controller_type, namespace):
        """
        Create multiple subscribers to receive commands

        :param controller_type: position, velocity, or effort
        :param namespace: namespace of the robot
        :type controller_type: str
        :type namespace: str
        """
        assert controller_type in ["position", "velocity", "effort"]
        rospy.Subscriber(namespace + controller_type + "_controller/command",
                         Float64MultiArray, self._pve_control_cb, queue_size=1)
        self._cmd = 0.0
        self._data_available = False

    def _pve_control_cb(self, msg):
        """Callback to receive commands."""
        self._data_available = True
        self._cmd = msg.data

    def get_last_cmd(self):
        """Method to get the last received command"""
        self._data_available = False
        return self._cmd

    def get_is_data_available(self):
        """Method to get flag that indicates if a command has been received"""
        return self._data_available


class Control:
    def __init__(self, pybullet, robot):
        self._pb = pybullet
        self._robot = robot
        # lists to save last received command
        self._position_joint_commands = [0] * len(self._robot.joint_indices)
        self._velocity_joint_commands = [0] * len(self._robot.joint_indices)
        self._effort_joint_commands = [0] * len(self._robot.joint_indices)
        # the max force to apply to the joint, used in velocity control
        max_effort = rospy.get_param("~max_effort", 100.0)
        self._force_commands = [max_effort] * len(self._robot.joint_indices)
        self._last_command_type = None
        self._last_command_time = time.time()

        # setup subscribers
        self._pc_subscriber = PosVelEffControl("position", self._robot.namespace)
        self._vc_subscriber = PosVelEffControl("velocity", self._robot.namespace)
        self._ec_subscriber = PosVelEffControl("effort", self._robot.namespace)

    def execute(self):
        """
        Execute the plugin. This function is called from main update loop in the pybullet ros node.
        """
        control_params = {"bodyUniqueId": self._robot.id, "jointIndices": self._robot.joint_indices}
        if self._pc_subscriber.get_is_data_available():
            if self._last_command_type != "position":
                self._last_command_type = "position"
            control_params["controlMode"] = self._pb.POSITION_CONTROL
            control_params["targetPositions"] = self._pc_subscriber.get_last_cmd()
            control_params["forces"] = self._force_commands
        if self._vc_subscriber.get_is_data_available():
            if self._last_command_type != "velocity":
                self._last_command_type = "velocity"
            control_params["controlMode"] = self._pb.VELOCITY_CONTROL
            control_params["targetVelocities"] = self._vc_subscriber.get_last_cmd()
            control_params["forces"] = self._force_commands
        if self._ec_subscriber.get_is_data_available():
            if self._last_command_type != "effort":
                self._last_command_type = "effort"
                self._pb.setJointMotorControlArray(self._robot.id, self._robot.joint_indices, self._pb.VELOCITY_CONTROL,
                                                   forces=[0] * len(self._robot.joint_indices))
            control_params["controlMode"] = self._pb.TORQUE_CONTROL
            compensation = self._robot.get_gravity_compensation()
            control_params["forces"] = [a + b for a, b in zip(self._ec_subscriber.get_last_cmd(), compensation)]

        if "controlMode" in control_params.keys():
            self._last_command_time = time.time()
            self._pb.setJointMotorControlArray(**control_params)
            return
        if self._last_command_type == "effort" and time.time() - self._last_command_time > 0.008:
            print("helllo")
            self._last_command_type = None
            self._pb.setJointMotorControlArray(self._robot.id, self._robot.joint_indices, self._pb.VELOCITY_CONTROL,
                                               forces=self._force_commands)
