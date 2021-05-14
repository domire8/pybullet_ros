#!/usr/bin/env python3

import os
import importlib
import rospy

from .function_exec_manager import FuncExecManager
from .pybullet_sim import PyBulletSim
from .robot import Robot


class PyBulletROSWrapper(object):
    """ROS wrapper class for PyBullet simulator"""

    def __init__(self):
        pb = importlib.import_module("pybullet")
        self.pb = pb
        self.loop_rate = rospy.get_param("~sim_loop_rate", 500.0)

        start_pybullet_gui = rospy.get_param("~pybullet_gui", True)
        pause_simulation = rospy.get_param("~pause_simulation", False)
        self.simulation = PyBulletSim(gui=start_pybullet_gui, pause_simulation=pause_simulation)

        # create object of environment class for later use
        env_plugin = rospy.get_param("~environment", "environment")  # default : plugins/environment.py
        plugin_import_prefix = rospy.get_param("~plugin_import_prefix", "pybullet_ros.plugins")
        self.environment = getattr(importlib.import_module(f"{plugin_import_prefix}.{env_plugin}"), "Environment")(pb)
        # load robot URDF model, set gravity, and ground plane
        # load environment
        rospy.loginfo("loading environment")
        self.environment.load_environment()

        urdf_path = self.get_urdf_path()
        fixed_base = rospy.get_param("~fixed_base", True)
        self.robot = Robot(urdf_path, self.simulation.uid, fixed_base, flags=pb.URDF_USE_SELF_COLLISION)

        robot_pose_x = rospy.get_param("~robot_pose_x", 0.0)
        robot_pose_y = rospy.get_param("~robot_pose_y", 0.0)
        robot_pose_z = rospy.get_param("~robot_pose_z", 1.0)
        robot_pose_yaw = rospy.get_param("~robot_pose_yaw", 0.0)
        self.robot.set_base_pose([robot_pose_x, robot_pose_y, robot_pose_z], [0, 0, robot_pose_yaw])

        self.robot.set_default_joint_positions([-0.017792060227770554, -0.7601235411041661, 0.019782607023391807,
                                                -2.342050140544315, 0.029840531355804868, 1.5411935298621688,
                                                0.7534486589746342])
        self.robot.goto_default_joint_positions()

        # get all revolute joint names and PyBullet index
        rev_joint_index_name_dic, prismatic_joint_index_name_dic, fixed_joint_index_name_dic, link_names_to_ids_dic = self.get_properties(
            pb)
        # import plugins dynamically
        self.plugins = []
        plugins = rospy.get_param("~plugins", [])
        if not plugins:
            rospy.logwarn("No plugins found, forgot to set param ~plugins?")
        # return to normal shell color
        print("\033[0m")
        # load plugins
        for plugin in plugins:
            module_ = plugin.pop("module")
            class_ = plugin.pop("class")
            params_ = plugin.copy()
            rospy.loginfo("loading plugin: {} class from {}".format(class_, module_))
            # create object of the imported file class
            obj = getattr(importlib.import_module(module_), class_)(pb, self.robot,
                                                                    rev_joints=rev_joint_index_name_dic,
                                                                    prism_joints=prismatic_joint_index_name_dic,
                                                                    fixed_joints=fixed_joint_index_name_dic,
                                                                    link_ids=link_names_to_ids_dic,
                                                                    **params_)
            # store objects in member variable for future use
            self.plugins.append(obj)
        rospy.loginfo("PyBullet ROS wrapper initialized")

    def get_properties(self, pb):
        """
        construct 3 dictionaries:
        - joint index to joint name x2 (1 for revolute, 1 for fixed joints)
        - link name to link index dictionary
        """
        rev_joint_index_name_dic = {}
        fixed_joint_index_name_dic = {}
        prismatic_joint_index_name_dic = {}
        link_names_to_ids_dic = {}
        for joint_index in range(0, pb.getNumJoints(self.robot._id)):
            info = pb.getJointInfo(self.robot._id, joint_index)
            # build a dictionary of link names to ids
            link_names_to_ids_dic[info[12].decode("utf-8")] = joint_index
            # ensure we are dealing with a revolute joint
            if info[2] == pb.JOINT_REVOLUTE:
                # insert key, value in dictionary (joint index, joint name)
                rev_joint_index_name_dic[joint_index] = info[1].decode("utf-8")  # info[1] refers to joint name
            elif info[2] == pb.JOINT_FIXED:
                # insert key, value in dictionary (joint index, joint name)
                fixed_joint_index_name_dic[joint_index] = info[1].decode("utf-8")  # info[1] refers to joint name
            elif info[2] == pb.JOINT_PRISMATIC:
                prismatic_joint_index_name_dic[joint_index] = info[1].decode("utf-8")  # info[1] refers to joint name
        return rev_joint_index_name_dic, prismatic_joint_index_name_dic, fixed_joint_index_name_dic, link_names_to_ids_dic

    def get_urdf_path(self):
        urdf_path = rospy.get_param("~robot_urdf_path", None)
        if not urdf_path:
            rospy.signal_shutdown(
                "[PyBulletROSWrapper::get_urdf_path] Could not get mandatory" +
                " parameter 'robot_urdf_path' from ROS param server, exiting now.")
            del self.simulation
            return None
        if not os.path.isfile(urdf_path):
            rospy.signal_shutdown(
                "[PyBulletROSWrapper::get_urdf_path] parameter 'robot_urdf_path' is set" +
                " but file {} does not exist, exiting now.".format(urdf_path))
            del self.simulation
            return None
        if urdf_path[-5:] == ".urdf":
            return urdf_path
        elif urdf_path[-11:] == ".urdf.xacro":
            robot_description = rospy.get_param("/robot_description", None)
            if not robot_description:
                rospy.signal_shutdown(
                    "[PyBulletROSWrapper::get_urdf_path] Could not get mandatory" +
                    " parameter 'robot_description' from ROS param server, exiting now.")
                del self.simulation
            rospy.loginfo(
                "[PyBulletROSWrapper::get_urdf_path] Generating urdf file from param 'robot_description' under: {0}".format(
                    urdf_path[:-6]))
            try:
                with open(urdf_path[:-6], 'w') as urdf_file:
                    urdf_file.write(robot_description)
                return urdf_path[:-6]
            except Exception as ex:
                rospy.signal_shutdown(
                    "[PyBulletROSWrapper::get_urdf_path] Failed to create urdf file from xacro, " +
                    "cannot write file, exiting now: {}".format(ex))
                del self.simulation
        else:
            rospy.signal_shutdown(
                "[PyBulletROSWrapper::get_urdf_path] Unknown file format of file {}, " +
                "expected a '.urdf' or '.urdf.xacro' file, exiting now".format(urdf_path))
            del self.simulation
        return None

    def start_pybullet_ros_wrapper_sequential(self):
        """
        This function is deprecated, we recommend the use of parallel plugin execution
        """
        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            if not self.simulation.is_paused():
                #     run x plugins
                for task in self.plugins:
                    task.execute()
                # perform all the actions in a single forward dynamics simulation step such
                # as collision detection, constraint solving and integration
                # self.simulation.step()
                self.simulation.step()
            rate.sleep()
        rospy.logwarn("[PyBulletROSWrapper] Killing all programs now.")
        if self.simulation.is_alive():
            del self.simulation

    def start_pybullet_ros_wrapper_parallel(self):
        """
        Execute plugins in parallel, however watch their execution time and warn if exceeds the deadline (loop rate)
        """
        # create object of our parallel execution manager
        exec_manager_obj = FuncExecManager(self.plugins, rospy.is_shutdown, self.simulation.step,
                                           self.simulation.is_paused,
                                           log_info=rospy.loginfo, log_warn=rospy.logwarn, log_debug=rospy.logdebug,
                                           function_name="plugin")
        # start parallel execution of all "execute" class methods in a synchronous way
        exec_manager_obj.start_synchronous_execution(loop_rate=self.loop_rate)
        rospy.logwarn("[PyBulletROSWrapper] Killing all programs now.")
        if self.simulation.is_alive():
            del self.simulation

    def start_pybullet_ros_wrapper(self):
        if rospy.get_param("~parallel_plugin_execution", True):
            self.start_pybullet_ros_wrapper_parallel()
        else:
            self.start_pybullet_ros_wrapper_sequential()


def main():
    rospy.init_node("pybullet_ros", anonymous=False)
    pybullet_ros_wrapper = PyBulletROSWrapper()
    pybullet_ros_wrapper.start_pybullet_ros_wrapper()
