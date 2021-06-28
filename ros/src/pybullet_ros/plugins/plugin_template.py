#!/usr/bin/env python3

"""
TODO: briefly describe your plugin here.
"""

import rospy


class PluginTemplate:
    def __init__(self, pybullet, robot):
        self._pb = pybullet
        self._robot = robot
        # TODO: implement here...

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        rospy.loginfo('[PluginTemplate::execute] My plugin is running!')
