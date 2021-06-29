#!/usr/bin/env python3

"""
RGBD camera sensor simulation for pybullet_ros based on pybullet.getCameraImage().
"""

import math

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class RGBDCamera:
    def __init__(self, pybullet, robot):
        self._pb = pybullet
        self._robot = robot
        # create image msg placeholder for publication
        self._image_msg = Image()
        # get RGBD camera parameters from ROS param server
        self._image_msg.width = rospy.get_param('~rgbd_camera/resolution/width', 640)
        self._image_msg.height = rospy.get_param('~rgbd_camera/resolution/height', 480)
        assert (self._image_msg.width > 5)
        assert (self._image_msg.height > 5)

        cam_frame_name = rospy.get_param('~rgbd_camera/frame_name', None)
        if not cam_frame_name:
            rospy.logerr("[RGBDCamera::init] Required parameter 'rgbd_camera/frame_name' not set, will exit now.")
            rospy.signal_shutdown("[RGBDCamera::init] Required param 'rgbd_camera/frame_name' not set.")
            return
        # get pybullet camera link id from its name
        if cam_frame_name not in self._robot.link_names:
            rospy.logerr(
                "[RGBDCamera::init] RGBD camera reference frame '{}' not found in URDF model, cannot continue.".format(
                    cam_frame_name))
            rospy.logwarn("[RGBDCamera::init] Available frames are: {}.".format(self._robot.link_names))
            rospy.signal_shutdown("[RGBDCamera::init]  Required param 'rgbd_camera/frame_name' not set properly.")
            return
        self._camera_link_id = self._robot.get_link_index_by_name(cam_frame_name)
        self._image_msg.header.frame_id = cam_frame_name
        # create publisher
        self._publisher = rospy.Publisher('rgb_image', Image, queue_size=1)
        self._image_msg.encoding = rospy.get_param('~rgbd_camera/resolution/encoding', 'rgb8')
        self._image_msg.is_bigendian = rospy.get_param('~rgbd_camera/resolution/encoding', 0)
        self._image_msg.step = rospy.get_param('~rgbd_camera/resolution/encoding', 1920)
        # projection matrix
        hfov = rospy.get_param('~rgbd_camera/hfov', 56.3)
        vfov = rospy.get_param('~rgbd_camera/vfov', 43.7)
        near_plane = rospy.get_param('~rgbd_camera/near_plane', 0.4)
        far_plane = rospy.get_param('~rgbd_camera/far_plane', 8)
        self._projection_matrix = self._compute_projection_matrix(hfov, vfov, near_plane, far_plane)
        # use cv_bridge ros to convert cv matrix to ros format
        self._image_bridge = CvBridge()
        # variable used to run this plugin at a lower frequency, HACK
        self._count = 0

    def _compute_projection_matrix(self, hfov, vfov, near_plane, far_plane):
        return self._pb.computeProjectionMatrix(
            left=-math.tan(math.pi * hfov / 360.0) * near_plane,
            right=math.tan(math.pi * hfov / 360.0) * near_plane,
            bottom=-math.tan(math.pi * vfov / 360.0) * near_plane,
            top=math.tan(math.pi * vfov / 360.0) * near_plane,
            nearVal=near_plane,
            farVal=far_plane)

    def _extract_frame(self, camera_image):
        bgr_image = np.zeros((self._image_msg.height, self._image_msg.width, 3))
        camera_image = np.reshape(camera_image[2], (camera_image[1], camera_image[0], 4))

        bgr_image[:, :, 2] = \
            (1 - camera_image[:, :, 3]) * camera_image[:, :, 2] + \
            camera_image[:, :, 3] * camera_image[:, :, 2]

        bgr_image[:, :, 1] = \
            (1 - camera_image[:, :, 3]) * camera_image[:, :, 1] + \
            camera_image[:, :, 3] * camera_image[:, :, 1]

        bgr_image[:, :, 0] = \
            (1 - camera_image[:, :, 3]) * camera_image[:, :, 0] + \
            camera_image[:, :, 3] * camera_image[:, :, 0]

        # return frame
        return bgr_image.astype(np.uint8)

    def _compute_camera_target(self, camera_position, camera_orientation):
        """
        Camera target is a point 5m in front of the robot camera.
        This method is used to transform it to the world reference frame.
        NOTE: this method uses pybullet functions and not tf.
        """
        target_point = [5.0, 0, 0]  # expressed w.r.t camera reference frame
        camera_position = [camera_position[0], camera_position[1], camera_position[2]]
        rm = self._pb.getMatrixFromQuaternion(camera_orientation)
        rotation_matrix = [[rm[0], rm[1], rm[2]], [rm[3], rm[4], rm[5]], [rm[6], rm[7], rm[8]]]
        return np.dot(rotation_matrix, target_point) + camera_position

    def execute(self):
        """
        Execute the plugin. This function is called from main update loop in the pybullet ros node.
        """
        # run at lower frequency, camera computations are expensive
        self._count += 1
        if self._count < 100:
            return
        self._count = 0  # reset count
        # get camera pose
        cam_position, cam_orientation = self._robot.get_link_state(self._camera_link_id)[0:2]
        # target is a point 5m ahead of the robot camera expressed w.r.t world reference frame
        target = self._compute_camera_target(cam_position, cam_orientation)
        view_matrix = self._pb.computeViewMatrix(cam_position, target, [0, 0, 1])
        # get camera image from pybullet
        pybullet_cam_resp = self._pb.getCameraImage(self._image_msg.width,
                                                    self._image_msg.height,
                                                    view_matrix,
                                                    self._projection_matrix,
                                                    renderer=self._pb.ER_BULLET_HARDWARE_OPENGL,
                                                    flags=self._pb.ER_NO_SEGMENTATION_MASK)
        # frame extraction function from pybullet
        frame = self._extract_frame(pybullet_cam_resp)
        # fill pixel data array
        self._image_msg.data = self._image_bridge.cv2_to_imgmsg(frame).data
        self._image_msg.header.stamp = rospy.Time.now()
        self._publisher.publish(self._image_msg)
