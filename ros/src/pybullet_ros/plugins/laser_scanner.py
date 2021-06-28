#!/usr/bin/env python3

"""
Laser scanner simulation based on pybullet rayTestBatch function.
This code does not add noise to the laser scanner readings.
"""

import math

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan


class LaserScanner:
    def __init__(self, pybullet, robot):
        self._pb = pybullet
        self._robot = robot
        # laser params
        laser_frame_name = rospy.get_param("~laser/frame_name",
                                           None)  # laser reference frame, has to be an existing link
        if not laser_frame_name:
            rospy.logerr("[LaserScanner::init] Required parameter 'laser/frame_name' not set, will exit now.")
            rospy.signal_shutdown("[LaserScanner::init] Required param 'laser/frame_name' not set.")
            return
        if laser_frame_name not in self._robot.link_names:
            rospy.logerr(
                "[LaserScanner::init] Laser reference frame '{}' not found in URDF model, cannot continue.".format(
                    laser_frame_name))
            rospy.logwarn("[LaserScanner::init] Available frames are: {}.".format(self._robot.link_names))
            rospy.signal_shutdown("[LaserScanner::init]  Required param 'laser/frame_name' not set properly.")
            return
        # get pybullet laser link id from its name
        self._laser_link_id = self._robot.get_link_index_by_name(laser_frame_name)
        self._laser_msg = LaserScan()
        # laser field of view
        angle_min = rospy.get_param("~laser/angle_min", -1.5707963)
        angle_max = rospy.get_param("~laser/angle_max", 1.5707963)
        assert (angle_max > angle_min)
        self._numRays = rospy.get_param("~laser/num_beams", 50)  # should be 512 beams but simulation becomes slow
        self._laser_msg.range_min = rospy.get_param("~laser/range_min", 0.03)
        self._laser_msg.range_max = rospy.get_param("~laser/range_max", 5.6)
        self.beam_visualisation = rospy.get_param("~laser/beam_visualisation", False)
        self._laser_msg.angle_min = angle_min
        self._laser_msg.angle_max = angle_max
        self._laser_msg.angle_increment = (angle_max - angle_min) / self._numRays

        self._publisher = rospy.Publisher("scan", LaserScan, queue_size=1)
        self._laser_msg.header.frame_id = laser_frame_name
        self._laser_msg.time_increment = 0.01
        self._laser_msg.scan_time = 0.1
        self._rayHitColor = [1, 0, 0]  # red color
        self._rayMissColor = [0, 1, 0]  # green color
        # compute rays end beam position
        self._rayFrom, self._rayTo = self._prepare_rays()
        # variable used to run this plugin at a lower frequency, HACK
        self._count = 0

    def _prepare_rays(self):
        """Assume laser is in the origin and compute its x, y beam end position"""
        rayFrom = []
        rayTo = []
        for n in range(0, self._numRays):
            alpha = self._laser_msg.angle_min + n * self._laser_msg.angle_increment
            rayFrom.append([self._laser_msg.range_min * math.cos(alpha),
                            self._laser_msg.range_min * math.sin(alpha), 0.0])
            rayTo.append([self._laser_msg.range_max * math.cos(alpha),
                          self._laser_msg.range_max * math.sin(alpha), 0.0])
        return rayFrom, rayTo

    def _transform_rays(self, laser_position, laser_orientation):
        """Transform rays from reference frame"""
        laser_position = [laser_position[0], laser_position[1], laser_position[2]]
        TFrayFrom = []
        TFrayTo = []
        rm = self._pb.getMatrixFromQuaternion(laser_orientation)
        rotation_matrix = [[rm[0], rm[1], rm[2]], [rm[3], rm[4], rm[5]], [rm[6], rm[7], rm[8]]]
        for ray in self._rayFrom:
            position = np.dot(rotation_matrix, [ray[0], ray[1], ray[2]]) + laser_position
            TFrayFrom.append([position[0], position[1], position[2]])
        for ray in self._rayTo:
            position = np.dot(rotation_matrix, [ray[0], ray[1], ray[2]]) + laser_position
            TFrayTo.append([position[0], position[1], position[2]])
        return TFrayFrom, TFrayTo

    def execute(self):
        """
        Execute the plugin. This function is called from main update loop in the pybullet ros node.
        """
        # run at lower frequency, laser computations are expensive
        self._count += 1
        if self._count < 100:
            return
        self._count = 0  # reset count
        # remove any previous laser data if any
        self._laser_msg.ranges = []
        # remove previous beam lines from screen
        if self.beam_visualisation:
            self._pb.removeAllUserDebugItems()
        # get laser link position
        laser_position, laser_orientation = self._robot.get_link_state(self._laser_link_id)[0:2]
        # transform start and end position of the rays which were generated considering laser at the origin
        rayFrom, rayTo = self._transform_rays(laser_position, laser_orientation)  # position + orientation
        # raycast using 4 threads
        results = self._pb.rayTestBatch(rayFrom, rayTo, 4)
        for i in range(self._numRays):
            if self.beam_visualisation:
                hitObjectUid = results[i][0]
                if hitObjectUid < 0:
                    # draw a line on pybullet gui for debug purposes in green because it did not hit any obstacle
                    self._pb.addUserDebugLine(rayFrom[i], rayTo[i], self._rayMissColor)
                else:
                    # draw a line on pybullet gui for debug purposes in red because it hited obstacle, results[i][3] -> hitPosition
                    self._pb.addUserDebugLine(rayFrom[i], results[i][3], self._rayHitColor)
            # compute laser ranges from hitFraction -> results[i][2]
            self._laser_msg.ranges.append(results[i][2] * self._laser_msg.range_max)
        # update laser time stamp with current time
        self._laser_msg.header.stamp = rospy.Time.now()
        # publish scan
        self._publisher.publish(self._laser_msg)
