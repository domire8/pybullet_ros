#!/usr/bin/env python3

import importlib

import rospy

from .function_exec_manager import FuncExecManager
from .pybullet_robot import PyBulletRobot
from .pybullet_sim import PyBulletSim


class PyBulletRosWrapper(object):
    """ROS wrapper class for pybullet simulator"""

    def __init__(self):
        self._pb = importlib.import_module("pybullet")
        self._loop_rate = rospy.get_param("~loop_rate", 80.0)
        start_pybullet_gui = rospy.get_param("~pybullet_gui", True)
        start_paused = rospy.get_param("~start_paused", False)

        print("\033[34m")  # print PyBullet stuff in blue
        self._simulation = PyBulletSim(gui=start_pybullet_gui, start_paused=start_paused)
        # create object of environment class for later use
        env_plugin = rospy.get_param("~environment", "environment")  # default : plugins/environment.py
        plugin_import_prefix = rospy.get_param("~plugin_import_prefix", "pybullet_ros.plugins")
        self._environment = getattr(importlib.import_module(f"{plugin_import_prefix}.{env_plugin}"), "Environment")(
            self._pb)
        rospy.loginfo("[PyBulletRosWrapper::init] Loading environment.")
        # set gravity and ground plane
        self._environment.load_environment()

        robot_names = rospy.get_param("~robots", None)
        self._robots = {}
        self._plugins = []
        plugins = rospy.get_param("~plugins", [])
        if not plugins:
            rospy.logwarn("[PyBulletRosWrapper::init] No plugins found, forgot to set param ~plugins?")
        for robot_name in robot_names:
            robot = PyBulletRobot(name=robot_name, uid=self._simulation.uid)
            self._robots[robot_name] = robot
            # import plugins dynamically
            for plugin in plugins:
                plugin_ = plugin.copy()
                module_ = plugin_.pop("module")
                class_ = plugin_.pop("class")
                params_ = plugin_.copy()
                rospy.loginfo(
                    "[PyBulletRosWrapper::init] Loading plugin: {} class from {} for robot {}".format(class_, module_,
                                                                                                      robot_name))
                # create object of the imported file class
                obj = getattr(importlib.import_module(module_), class_)(self._pb, robot, **params_)
                # store objects in member variable for future use
                self._plugins.append(obj)
        rospy.loginfo("[PyBulletRosWrapper::init] PyBullet ROS wrapper initialized.")
        print("\033[0m")

    def _start_pybullet_ros_wrapper_sequential(self):
        """
        This function is deprecated, we recommend the use of parallel plugin execution
        """
        rate = rospy.Rate(self._loop_rate)
        while not rospy.is_shutdown():
            if not self._simulation.is_paused():
                # run x plugins
                for task in self._plugins:
                    task.execute()
                # perform all the actions in a single forward dynamics simulation step such
                # as collision detection, constraint solving and integration
                self._simulation.step()
            rate.sleep()
        rospy.logwarn("[PyBulletROSWrapper::start_pybullet_ros_wrapper_sequential] Killing all programs now.")
        if self._simulation.is_alive():
            del self._simulation

    def _start_pybullet_ros_wrapper_parallel(self):
        """
        Execute plugins in parallel, however watch their execution time and warn if exceeds the deadline (loop rate)
        """
        exec_manager_obj = FuncExecManager(self._plugins, rospy.is_shutdown, self._simulation.step,
                                           self._simulation.is_paused,
                                           log_info=rospy.loginfo, log_warn=rospy.logwarn, log_debug=rospy.logdebug,
                                           function_name="plugin")
        # start parallel execution of all "execute" class methods in a synchronous way
        exec_manager_obj.start_synchronous_execution(loop_rate=self._loop_rate)
        rospy.logwarn("[PyBulletROSWrapper::start_pybullet_ros_wrapper_parallel] Killing all programs now.")
        if self._simulation.is_alive():
            del self._simulation

    def start_pybullet_ros_wrapper(self):
        if rospy.get_param("~parallel_plugin_execution", True):
            self._start_pybullet_ros_wrapper_parallel()
        else:
            self._start_pybullet_ros_wrapper_sequential()


def main():
    """Function called by pybullet_ros_node script"""
    rospy.init_node("pybullet_ros", anonymous=False)  # node name gets overridden if launched by a launch file
    pybullet_ros_interface = PyBulletRosWrapper()
    pybullet_ros_interface.start_pybullet_ros_wrapper()
