#!/usr/bin/env python3

import importlib

import rospy

from .function_exec_manager import FuncExecManager
from .pybullet_robot import PyBulletRobot
from .pybullet_sim import PyBulletSim


class PyBulletRosWrapper(object):
    """ROS wrapper class for pybullet simulator"""

    def __init__(self):
        # import pybullet
        self.pb = importlib.import_module('pybullet')
        # get from param server the frequency at which to run the simulation
        self.loop_rate = rospy.get_param('~loop_rate', 80.0)
        # query from param server if gui is needed
        start_pybullet_gui = rospy.get_param("~pybullet_gui", True)
        start_paused = rospy.get_param("~start_paused", False)
        self.simulation = PyBulletSim(gui=start_pybullet_gui, start_paused=start_paused)
        # create object of environment class for later use
        env_plugin = rospy.get_param('~environment', 'environment')  # default : plugins/environment.py
        plugin_import_prefix = rospy.get_param('~plugin_import_prefix', 'pybullet_ros.plugins')
        self.environment = getattr(importlib.import_module(f'{plugin_import_prefix}.{env_plugin}'), 'Environment')(
            self.pb)
        rospy.loginfo('loading environment')
        # set gravity and ground plane
        self.environment.load_environment()
        # load robots
        robot_names = rospy.get_param("~robots", None)
        self.robot = PyBulletRobot(name=robot_names[0], uid=self.simulation.get_uid())
        self.robot = self.robot.get_id()
        # get all revolute joint names and pybullet index
        rev_joint_index_name_dic, prismatic_joint_index_name_dic, fixed_joint_index_name_dic, link_names_to_ids_dic = self.get_properties()
        self.set_initial_joint_configuration(self.robot, rev_joint_index_name_dic)
        self.set_initial_joint_configuration(self.robot, prismatic_joint_index_name_dic)
        # import plugins dynamically
        self.plugins = []
        plugins = rospy.get_param('~plugins', [])
        if not plugins:
            rospy.logwarn('No plugins found, forgot to set param ~plugins?')
        # return to normal shell color
        print('\033[0m')
        # load plugins
        for plugin in plugins:
            module_ = plugin.pop("module")
            class_ = plugin.pop("class")
            params_ = plugin.copy()
            rospy.loginfo('loading plugin: {} class from {}'.format(class_, module_))
            # create object of the imported file class
            obj = getattr(importlib.import_module(module_), class_)(self.pb, self.robot,
                                                                    rev_joints=rev_joint_index_name_dic,
                                                                    prism_joints=prismatic_joint_index_name_dic,
                                                                    fixed_joints=fixed_joint_index_name_dic,
                                                                    link_ids=link_names_to_ids_dic,
                                                                    name=robot_names[0],
                                                                    **params_)
            # store objects in member variable for future use
            self.plugins.append(obj)
        rospy.loginfo('pybullet ROS wrapper initialized')

    def get_properties(self):
        """
        construct 3 dictionaries:
        - joint index to joint name x2 (1 for revolute, 1 for fixed joints)
        - link name to link index dictionary
        """
        rev_joint_index_name_dic = {}
        fixed_joint_index_name_dic = {}
        prismatic_joint_index_name_dic = {}
        link_names_to_ids_dic = {}
        for joint_index in range(0, self.pb.getNumJoints(self.robot)):
            info = self.pb.getJointInfo(self.robot, joint_index)
            # build a dictionary of link names to ids
            link_names_to_ids_dic[info[12].decode('utf-8')] = joint_index
            # ensure we are dealing with a revolute joint
            if info[2] == self.pb.JOINT_REVOLUTE:
                # insert key, value in dictionary (joint index, joint name)
                rev_joint_index_name_dic[joint_index] = info[1].decode('utf-8')  # info[1] refers to joint name
            elif info[2] == self.pb.JOINT_FIXED:
                # insert key, value in dictionary (joint index, joint name)
                fixed_joint_index_name_dic[joint_index] = info[1].decode('utf-8')  # info[1] refers to joint name
            elif info[2] == self.pb.JOINT_PRISMATIC:
                prismatic_joint_index_name_dic[joint_index] = info[1].decode('utf-8')  # info[1] refers to joint name
        return rev_joint_index_name_dic, prismatic_joint_index_name_dic, fixed_joint_index_name_dic, link_names_to_ids_dic

    def set_initial_joint_configuration(self, robot, joint_dict):
        """Function to set initial joint configuration to the mean of the joint position limits."""
        for joint_id, joint_name in joint_dict.items():
            lower_lim = self.pb.getJointInfo(robot, joint_id)[8]
            upper_lim = self.pb.getJointInfo(robot, joint_id)[9]
            self.pb.resetJointState(robot, joint_id, (upper_lim + lower_lim) / 2)

    def start_pybullet_ros_wrapper_sequential(self):
        """
        This function is deprecated, we recommend the use of parallel plugin execution
        """
        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            if not self.simulation.is_paused():
                # run x plugins
                for task in self.plugins:
                    task.execute()
                # perform all the actions in a single forward dynamics simulation step such
                # as collision detection, constraint solving and integration
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
                                           function_name='plugin')
        # start parallel execution of all "execute" class methods in a synchronous way
        exec_manager_obj.start_synchronous_execution(loop_rate=self.loop_rate)
        rospy.logwarn("[PyBulletROSWrapper] Killing all programs now.")
        if self.simulation.is_alive():
            del self.simulation

    def start_pybullet_ros_wrapper(self):
        if rospy.get_param('~parallel_plugin_execution', True):
            self.start_pybullet_ros_wrapper_parallel()
        else:
            self.start_pybullet_ros_wrapper_sequential()


def main():
    """function called by pybullet_ros_node script"""
    rospy.init_node('pybullet_ros', anonymous=False)  # node name gets overridden if launched by a launch file
    pybullet_ros_interface = PyBulletRosWrapper()
    pybullet_ros_interface.start_pybullet_ros_wrapper()
