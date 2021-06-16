import os
import re

import pybullet as pb
import rospy


# import quaternion


class PyBulletRobot(object):
    """

    """

    def __init__(self, name, uid):
        """
        Constructor of the PyBulletRobot class.

        :param name: ROS namespace of the robot's parameters
        :param uid: server id of PyBullet

        :type name: str
        :type uid: int
        """
        assert isinstance(name, str), "[PyBulletRobot::init] Parameter 'name' has an incorrect type."
        assert isinstance(uid, int), "[PyBulletRobot::init] Parameter 'uid' has an incorrect type."
        self._initialized = False
        self._uid = uid
        allowed = re.compile("[a-zA-Z0-9_]*$")
        if not allowed.match(name):
            rospy.logerr(
                "[PyBulletRobot::init] Invalid name '{}' for a PyBulletRobot. Allowed characters are [a-zA-Z0-9_]. " +
                "Exiting now.".format(name))
            return
        self._namespace = "/" + name + "/"
        urdf_path = self._get_urdf_path(self._namespace)
        if urdf_path is None:
            return
        # get robot spawn pose from parameter server
        pose_x = rospy.get_param(self._namespace + "pose_x", 0.0)
        pose_y = rospy.get_param(self._namespace + "pose_y", 0.0)
        pose_z = rospy.get_param(self._namespace + "pose_z", 1.0)
        pose_yaw = rospy.get_param(self._namespace + "pose_yaw", 0.0)
        fixed_base = rospy.get_param(self._namespace + "fixed_base", False)
        # load robot from URDF model
        # user decides if inertia is computed automatically by pybullet or custom
        if rospy.get_param(self._namespace + "use_inertia_from_file", False):
            # combining several boolean flags using "or" according to pybullet documentation
            urdf_flags = pb.URDF_USE_INERTIA_FROM_FILE | pb.URDF_USE_SELF_COLLISION
        else:
            urdf_flags = pb.URDF_USE_SELF_COLLISION
        self._id = pb.loadURDF(urdf_path, basePosition=[pose_x, pose_y, pose_z],
                               baseOrientation=pb.getQuaternionFromEuler([0.0, 0.0, pose_yaw]),
                               useFixedBase=fixed_base, flags=urdf_flags, physicsClientId=self._uid)
        self._initialized = True

    def is_initialized(self):
        """
        Getter of the initialized attribute

        :rtype: bool
        """
        return self._initialized

    def get_id(self):
        """
        Getter of the robot ID

        :rtype: int
        """
        return self._id

    @staticmethod
    def _get_urdf_path(namespace):
        """
        Get robot urdf path from parameter server and create urdf file from robot_description param, if necessary. Return None if one of the operations failed.

        :param namespace: namespace of the robot parameters
        :type namespace: str

        :rtype: str
        """
        urdf_path = rospy.get_param(namespace + "urdf_path", None)
        if not urdf_path:
            rospy.logerr(
                "[PyBulletRobot::get_urdf_path] Could not get mandatory" +
                " parameter '{}urdf_path' from ROS param server, exiting now.".format(namespace))
            return None
        if not os.path.isfile(urdf_path):
            rospy.logerr(
                "[PyBulletRobot::get_urdf_path] parameter 'urdf_path' is set" +
                " but file {} does not exist, exiting now.".format(urdf_path))
            return None
        if urdf_path[-5:] == ".urdf":
            return urdf_path
        elif urdf_path[-11:] == ".urdf.xacro":
            robot_description = rospy.get_param(namespace + "robot_description", None)
            if not robot_description:
                rospy.logerr(
                    "[PyBulletRobot::get_urdf_path] Could not get mandatory" +
                    " parameter '{}robot_description' from ROS param server, exiting now.".format(namespace))
                return None
            rospy.loginfo(
                "[PyBulletRobot::get_robot_urdf_path] Generating urdf file from param '{}robot_description' at: {}".format(
                    namespace, urdf_path[:-6]))
            try:
                with open(urdf_path[:-6], 'w') as urdf_file:
                    urdf_file.write(robot_description)
                return urdf_path[:-6]
            except Exception as ex:
                rospy.logerr(
                    "[PyBulletRobot::get_robot_urdf_path] Failed to create urdf file from param '{}robot_description', " +
                    "cannot write file, exiting now: {}".format(namespace, ex))
                return None
        else:
            rospy.logerr(
                "[PyBulletRobot::get_urdf_path] Unknown file format of file {}, " +
                "expected a '.urdf' or '.urdf.xacro' file, exiting now".format(urdf_path))
            return None
