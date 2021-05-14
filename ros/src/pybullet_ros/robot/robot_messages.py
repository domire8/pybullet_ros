import numpy as np


class RobotJointState:
    def __init__(self, joint_names):
        for joint_name in joint_names:
            assert isinstance(joint_name, str), "[RobotJointState::init] Argument 'joint_states' of incorrect type."
        nb_joints = len(joint_names)
        self.joint_names = joint_names
        self.joint_positions = np.zeros((nb_joints,))
        self.joint_velocities = np.zeros((nb_joints,))
        self.joint_efforts = np.zeros((nb_joints,))

    def __str__(self):
        return "Joint names: " + str(self.joint_names) + \
               "\nJoint positions: " + str(self.joint_positions) + \
               "\nJoint velocities: " + str(self.joint_velocities) + \
               "\nJoint efforts: " + str(self.joint_efforts)


class RobotFullState:
    def __init__(self, joint_names):
        for joint_name in joint_names:
            assert isinstance(joint_name, str), "[RobotJointState::init] Argument 'joint_states' of incorrect type."
        nb_joints = len(joint_names)
        self.joint_names = joint_names
        self.joint_positions = np.zeros((nb_joints,))
        self.joint_velocities = np.zeros((nb_joints,))
        self.joint_efforts = np.zeros((nb_joints,))
        self.jacobian = np.zeros((6, nb_joints))
        self.inertia = np.zeros((nb_joints, nb_joints))
        self.ee_position = np.zeros((3,))
        self.ee_orientation = np.quaternion(1, 0, 0, 0)
        self.ee_linear_velocity = np.zeros((3,))
        self.ee_angular_velocity = np.zeros((3,))
        self.ee_force = np.zeros((3,))
        self.ee_torque = np.zeros((3,))

    def __str__(self):
        return "Joint names: " + self.joint_names + \
               "Joint positions: " + str(self.joint_positions) + \
               "\nJoint velocities: " + str(self.joint_velocities) + \
               "\nJoint efforts: " + str(self.joint_efforts) + \
               "\nEE position: " + str(self.ee_position) + \
               "\nEE orientation: " + str(self.ee_orientation) + \
               "\nEE linear velocity: " + str(self.ee_linear_velocity) + \
               "\nEE angular velocity: " + str(self.ee_angular_velocity) + \
               "\nEE force: " + str(self.ee_force) + \
               "\nEE torque: " + str(self.ee_torque) + \
               "\nJacobian matrix: " + str(self.jacobian) + \
               "\nInertia matrix: " + str(self.inertia)
