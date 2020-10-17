import os

import cv2
import numpy as np
import yaml
import copy
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import CameraInfo, CompressedImage


class ROSAgent:
    def __init__(self):
        # Get the vehicle name, which comes in as HOSTNAME
        self.vehicle = os.getenv("HOSTNAME")
        topic = "/{}/wheels_driver_node/wheels_cmd".format(self.vehicle)
        self.ik_action_sub = rospy.Subscriber(topic, WheelsCmdStamped, self._ik_action_cb)
        # Place holder for the action, which will be read by the agent in solution.py
        self.action = np.array([0.0, 0.0])
        self.updated = True
        self.initialized = False

        # Publishes onto the corrected image topic
        # since image out of simulator is currently rectified
        #topic = "/{}/image_topic".format(self.vehicle)
        topic = "/{}/camera_node/image/compressed".format(self.vehicle)
        self.cam_pub = rospy.Publisher(topic, CompressedImage, queue_size=10)

        # Publisher for camera info - needed for the ground_projection
        #topic = "/{}/camera_info_topic".format(self.vehicle)
        topic = "/{}/camera_node/camera_info".format(self.vehicle)
        self.cam_info_pub = rospy.Publisher(topic, CameraInfo, queue_size=1)

        # copied from camera driver:


        # For intrinsic calibration
        self.cali_file_folder = '/data/config/calibrations/camera_intrinsic/'
        self.frame_id = rospy.get_namespace().strip('/') + '/camera_optical_frame'
        self.cali_file = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(self.cali_file):
            rospy.logwarn("Calibration not found: %s.\n Using default instead." % self.cali_file)
            self.cali_file = (self.cali_file_folder + "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Load the calibration file
        self.original_camera_info = self.load_camera_info(self.cali_file)
        self.original_camera_info.header.frame_id = self.frame_id
        self.current_camera_info = copy.deepcopy(self.original_camera_info)
        rospy.loginfo("Using calibration file: %s" % self.cali_file)


        # Initializes the node
        rospy.init_node("ROSTemplate")

        # 15Hz ROS Cycle - TODO: What is this number?
        self.r = rospy.Rate(15)

    def _ik_action_cb(self, msg):
        """
        Callback to listen to last outputted action from inverse_kinematics node
        Stores it and sustains same action until new message published on topic
        """
        self.initialized = True
        vl = msg.vel_left
        vr = msg.vel_right
        self.action = np.array([vl, vr])
        self.updated = True

    def _publish_info(self):
        """
        Publishes a default CameraInfo - TODO: Fix after distortion applied in simulator
        """
        # Publish the CameraInfo message
        stamp = rospy.Time.now()
        self.current_camera_info.header.stamp = stamp
        self.cam_info_pub.publish(self.current_camera_info)

    def _publish_img(self, obs):
        """
        Publishes the image to the compressed_image topic, which triggers the lane following loop
        """

        # XXX: make this into a function (there were a few of these conversions around...)
        img_msg = CompressedImage()

        time = rospy.get_rostime()
        img_msg.header.stamp.secs = time.secs
        img_msg.header.stamp.nsecs = time.nsecs

        img_msg.format = "jpeg"
        contig = cv2.cvtColor(np.ascontiguousarray(obs), cv2.COLOR_BGR2RGB)
        img_msg.data = np.array(cv2.imencode(".jpg", contig)[1]).tostring()

        self.cam_pub.publish(img_msg)

    @staticmethod
    def load_camera_info(filename):
        """Loads the camera calibration files.

        Loads the intrinsic and extrinsic camera matrices.

        Args:
            filename (:obj:`str`): filename of calibration files.

        Returns:
            :obj:`CameraInfo`: a CameraInfo message object

        """
        with open(filename, 'r') as stream:
            calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info