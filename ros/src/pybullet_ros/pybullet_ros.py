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
        self.robots = {}
        self.plugins = []
        plugins = rospy.get_param('~plugins', [])
        if not plugins:
            rospy.logwarn('No plugins found, forgot to set param ~plugins?')
        # return to normal shell color
        print('\033[0m')
        for robot_name in robot_names:
            robot = PyBulletRobot(name=robot_name, uid=self.simulation.get_uid())
            self.robots[robot_name] = robot
            # import plugins dynamically
            for plugin in plugins:
                module_ = plugin.pop("module")
                class_ = plugin.pop("class")
                params_ = plugin.copy()
                rospy.loginfo('loading plugin: {} class from {}'.format(class_, module_))
                # create object of the imported file class
                obj = getattr(importlib.import_module(module_), class_)(self.pb, robot, **params_)
                # store objects in member variable for future use
                self.plugins.append(obj)
        rospy.loginfo('pybullet ROS wrapper initialized')

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
