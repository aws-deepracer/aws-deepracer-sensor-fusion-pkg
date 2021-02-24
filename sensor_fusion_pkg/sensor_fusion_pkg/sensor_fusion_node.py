#!/usr/bin/env python

#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

"""
sensor_fusion_node.py

This module creates the sensor_fusion_node which is responsible for collecting the
messages from all the sensors and publish the combined sensor message. It provides
services and functions to subscribe to the LiDAR and camera messages, configure LiDAR,
publish a combined sensor message, publish an overlay message with the lidar data
overlayed on top of the camera frame.

The node defines:
    A subscriber to the /rplidar_ros/scan topic published
    by the rplidar_ros with LiDAR data.
    A subscriber to the /camera_pkg/video_mjpeg topic published
    by the camera_pkg with the camera data.
    A subscriber to the /camera_pkg/display_mjpeg topic published
    by the camera_pkg with the display image data.
    sensor_message_publisher: A publisher to publish the message that combines
                              the camera data published from the camera package and
                              the LiDAR data published from the rplidar_ros package
                              (preprocessed in this node as per the LiDAR configuration).
    overlay_image_publisher: A publisher to publish the Image message that has the LiDAR
                             overaly data on top of the camera frame to highlight the sectors
                             with an obstacle present behind the DeepRacer device.
    status_check_service: A service to find out the data status of the cameras and the LiDAR
                          sensors. Based on whether the single camera/stereo camera/LiDAR data
                          is being read by the node, the corresponding sensor status are set.
    lidar_config_service: A service to configure LiDAR processing information for each model.
                          It allows to set the LiDAR preprocessing information like the maximum
                          distance, minimum distance, number of sectors etc., which will be used
                          to create the sensor message being published for running the inference
                          for the particular model selected.
"""

import os
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from deepracer_interfaces_pkg.msg import (EvoSensorMsg,
                                          CameraMsg)
from sensor_msgs.msg import LaserScan
from deepracer_interfaces_pkg.srv import (LidarConfigSrv,
                                          SensorStatusCheckSrv)
from sensor_fusion_pkg import (constants,
                               utils,
                               lidar_preprocess,
                               lidar_overlay)


class SensorFusionNode(Node):
    """Node responsible for collecting the camera and LiDAR messages and publishing them
       at the rate of the camera sensor.
    """

    def __init__(self):
        """Create a SensorFusionNode.
        """
        super().__init__("sensor_fusion_node")
        self.get_logger().info("sensor_fusion_node started.")

        # Buffer objects to hold the sensor and images data.
        self.lidar_buffer = utils.DoubleBuffer(clear_data_on_get=False)
        self.camera_buffer = utils.DoubleBuffer(clear_data_on_get=False)
        self.overlay_lidar_buffer = utils.DoubleBuffer(clear_data_on_get=False)

        # CvBridge object used to convert the sensor_msgs.Image type to OpenCV Image
        # and vice versa.
        self.bridge = CvBridge()

        # Flag to enable publishing the overlay image.
        self.enable_overlay_publish = True

        self.lidar_configuration, self.lidar_overlay_configuration = \
            self.load_lidar_configurations()

        # Max LiDAR distance required to replace the incorrect/erroneous LiDAR data point
        # that is read as 'inf' in the LaseScan message.
        self.max_lidar_dist = min(self.lidar_configuration
                                  [constants.
                                   LidarConfigurationKeys.
                                   MAX_LIDAR_DIST],
                                  self.lidar_configuration
                                  [constants.
                                   LidarConfigurationKeys.
                                   LIDAR_CLIPPING_DIST])

        # LiDAR preprocessing object to be set based on LiDAR configuration for the model.
        self.lidar_preprocessing_obj = lidar_preprocess.DefaultPreprocessing()

        # Define the LiDAR overlay configuration. Default to use SectorPreprocessing and
        # 8 sectors.
        self.lidar_overlay_obj = lidar_overlay.LidarOverlay(self.lidar_overlay_configuration)
        self.lidar_overlay_preprocessing_obj = \
            lidar_preprocess.SectorPreprocessing(self.lidar_overlay_configuration[
                                                    constants.
                                                    LidarOverlayConfigurationKeys.
                                                    NUM_LIDAR_SECTORS],
                                                 self.lidar_overlay_configuration[
                                                    constants.
                                                    LidarOverlayConfigurationKeys.
                                                    MAX_LIDAR_DIST])

        # Publisher that sends combined sensor messages with camera data and LiDAR data
        # to inference node.
        self.sensor_message_pub_cb_grp = ReentrantCallbackGroup()
        self.sensor_message_publisher = self.create_publisher(EvoSensorMsg,
                                                              constants.SENSOR_MSG_TOPIC,
                                                              1,
                                                              callback_group=self.sensor_message_pub_cb_grp)

        # Publisher to publish the overlay message with sector LiDAR information
        # overlayed over the camera image frame.
        self.overlay_image_pub_cb_grp = ReentrantCallbackGroup()
        # Queue size of overlay_image_publisher has been increased to 10 to
        # enable web_video_server streaming seamlessly.
        self.overlay_image_publisher = \
            self.create_publisher(Image,
                                  constants.OVERLAY_MSG_TOPIC,
                                  10,
                                  callback_group=self.overlay_image_pub_cb_grp)

        # Service to find out what sensors are connected and publishing messages.
        self.status_check_service_cb_group = ReentrantCallbackGroup()
        self.status_check_service = \
            self.create_service(SensorStatusCheckSrv,
                                constants.SENSOR_DATA_STATUS_SERVICE_NAME,
                                self.status_check_callback,
                                callback_group=self.status_check_service_cb_group)

        # Service to configure LiDAR processing information for each model.
        # Needed to differentiate SECTOR_LIDAR and LIDAR models.
        self.lidar_config_service_cb_group = ReentrantCallbackGroup()
        self.lidar_config_service = \
            self.create_service(LidarConfigSrv,
                                constants.CONFIGURE_LIDAR_SERVICE_NAME,
                                self.set_lidar_config_callback,
                                callback_group=self.lidar_config_service_cb_group)

        # Initialize the listener for LiDAR data.
        self.lidar_listener()
        # Initialize the listener for camera data.
        self.camera_listener()

        # Heartbeat timer.
        self.timer_count = 0
        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        """Heartbeat function to keep the node alive.
        """
        self.get_logger().debug(f"Timer heartbeat {self.timer_count}")
        self.timer_count += 1

    def create_default_sensor_config_file(self, sensor_config_file_path):
        """Create a sensor configuration file with default LiDAR configuration.

        Args:
            sensor_config_file_path (str): Location where the sensor configuration
                                           file should be saved.
        """
        data = dict()
        data[constants.SensorConfigurationKeys.LIDAR] = \
            constants.DEFAULT_LIDAR_CONFIGURATION
        data[constants.SensorConfigurationKeys.LIDAR_OVERLAY] = \
            constants.DEFAULT_LIDAR_OVERLAY_CONFIGURATION
        with open(sensor_config_file_path, "w") as outfile:
            json.dump(data, outfile, indent=4,)

    def load_lidar_configurations(self):
        """Load the LiDAR and LiDAR overlay configuration details from the sensor
           configuration file.

        Returns:
            (dict, dict): A tuple of dictionary containing the LiDAR and LiDAR overlay
                          configuration read from sensor configuration file.
        """
        sensor_config_file_path = constants.SENSOR_CONFIGURATION_FILE_PATH
        try:
            if(not os.path.isfile(sensor_config_file_path)):
                self.get_logger().warning("No sensor configuration file. "
                                          "Creating sensor configuration file.")
                self.create_default_sensor_config_file(sensor_config_file_path)
            with open(sensor_config_file_path) as json_file:
                data = json.load(json_file)
                try:
                    constants.LidarConfigurationKeys.validate_dict(
                        data[constants.SensorConfigurationKeys.LIDAR])
                    lidar_config = data[constants.SensorConfigurationKeys.LIDAR]
                except Exception as ex:
                    self.get_logger().warning("Lidar configuration data is not valid.")
                    lidar_config = constants.DEFAULT_LIDAR_CONFIGURATION
                try:
                    constants.LidarOverlayConfigurationKeys.validate_dict(
                        data[constants.SensorConfigurationKeys.LIDAR_OVERLAY])
                    lidar_overlay_config = \
                        data[constants.SensorConfigurationKeys.LIDAR_OVERLAY]
                except Exception as ex:
                    self.get_logger().warning("Lidar overlay configuration data is not valid.")
                    lidar_overlay_config = constants.DEFAULT_LIDAR_OVERLAY_CONFIGURATION
            return (lidar_config, lidar_overlay_config)
        except Exception as ex:
            self.get_logger().warning(f"Error while loading lidar configurations: {ex}")
            return (constants.DEFAULT_LIDAR_CONFIGURATION,
                    constants.DEFAULT_LIDAR_OVERLAY_CONFIGURATION)

    def status_check_callback(self, req, res):
        """Callback method for the sensor_data_status service to return the status of the
           sensors based on the presence of data in the data buffer at the sensor fusion node.

        Args:
            req (SensorStatusCheckSrv.Request): No request data passed.
            res (SensorStatusCheckSrv.Response): Response object with the each sensor data
                                                 status indicating 0: connected or 1: not
                                                 connected.

        Returns:
            SensorStatusCheckSrv.Response: Response object with the each sensor data status
                                           indicating 0: connected or 1: not connected.
        """
        try:
            res.single_camera_status = 1
            res.stereo_camera_status = 1
            res.lidar_status = 1
            if self.camera_buffer.read_buffer is not None \
               and isinstance(self.camera_buffer.read_buffer, list):
                if len(self.camera_buffer.read_buffer) == 2:
                    res.stereo_camera_status = 0
                elif len(self.camera_buffer.read_buffer) == 1:
                    res.single_camera_status = 0
            if self.lidar_buffer.read_buffer is not None:
                res.lidar_status = 0
            return res
        except Exception as ex:
            self.get_logger().error(f"Failed to get sensor data status: {ex}")

    def set_lidar_config_callback(self, req, res):
        """Callback method for the configure_lidar service. Allows clients to dynamically
           set the LiDAR configuration: use_lidar, min_angle, max_angle, min_distance,
           max_distance, num_values, clipping_distance, num_sectors, preprocess_type.

        Args:
            req (LidarConfigSrv.Request): Request object with the LiDAR configuration
                                          values sent.
            res (LidarConfigSrv.Response): Response object with error(int) flag
                                           indicating successful LiDAR configuration
                                           update.

        Returns:
            LidarConfigSrv.Response: Response object with error(int) flag indicating
                                     successful LiDAR configuration update.
        """
        try:
            if req.min_angle > req.max_angle \
               or req.min_distance > req.max_distance \
               or req.min_distance > req.clipping_distance \
               or req.num_values < 1:
                self.get_logger().error(f"Incorrect lidar configuration values {req}")
                res.error = 1
                return res
            # Check if the preprocessing is valid
            if req.preprocess_type not in set(constants.LidarPreprocessingTypes._value2member_map_):
                self.get_logger().error(f"Incorrect lidar preprocess type {req}")
                res.error = 1
                return res
            new_lidar_config = dict()
            new_lidar_config[constants.LidarConfigurationKeys.USE_LIDAR] = req.use_lidar
            new_lidar_config[constants.LidarConfigurationKeys.MIN_LIDAR_ANGLE] = req.min_angle
            new_lidar_config[constants.LidarConfigurationKeys.MAX_LIDAR_ANGLE] = req.max_angle
            new_lidar_config[constants.LidarConfigurationKeys.MIN_LIDAR_DIST] = req.min_distance
            new_lidar_config[constants.LidarConfigurationKeys.MAX_LIDAR_DIST] = req.max_distance
            new_lidar_config[constants.LidarConfigurationKeys.NUM_LIDAR_VALUES] = req.num_values
            new_lidar_config[constants.LidarConfigurationKeys.LIDAR_CLIPPING_DIST] = req.clipping_distance
            new_lidar_config[constants.LidarConfigurationKeys.NUM_LIDAR_SECTORS] = req.num_sectors
            new_lidar_config[constants.LidarConfigurationKeys.PREPROCESS_TYPE] = req.preprocess_type
            self.lidar_configuration = new_lidar_config
            # Select the minimum of the clipping distance and max_lidar_distance as upper bound of the range
            self.max_lidar_dist = min(self.lidar_configuration
                                      [constants.LidarConfigurationKeys.MAX_LIDAR_DIST],
                                      self.lidar_configuration
                                      [constants.LidarConfigurationKeys.LIDAR_CLIPPING_DIST])
            if new_lidar_config[constants.LidarConfigurationKeys.PREPROCESS_TYPE] \
               == constants.LidarPreprocessingTypes.DEFAULT.value:
                self.lidar_preprocessing_obj = lidar_preprocess.DefaultPreprocessing()
            elif new_lidar_config[constants.LidarConfigurationKeys.PREPROCESS_TYPE] \
                    == constants.LidarPreprocessingTypes.SECTOR.value:
                self.lidar_preprocessing_obj = \
                    lidar_preprocess.SectorPreprocessing(
                        new_lidar_config[constants.LidarConfigurationKeys.NUM_LIDAR_SECTORS],
                        self.max_lidar_dist)

            self.get_logger().info(f"Setting the lidar configuration values for the model: {req}")
            res.error = 0
            return res
        except Exception as ex:
            self.lidar_configuration = constants.DEFAULT_LIDAR_CONFIGURATION
            self.lidar_preprocessing_obj = lidar_preprocess.DefaultPreprocessing()
            self.get_logger().error(f"Failed set lidar configuration: {ex}, "
                                    "reloading default lidar configuration "
                                    f"{self.lidar_configuration}")
            res.error = 1
            return res

    def lidar_listener(self):
        """Registers the node to listen to LiDAR(scan) messages.
        """
        lidar_sub_cb_grp = ReentrantCallbackGroup()
        self.create_subscription(LaserScan,
                                 constants.LIDAR_MSG_TOPIC,
                                 self.lidar_callback,
                                 10,
                                 callback_group=lidar_sub_cb_grp)

    def camera_listener(self):
        """Registers the node to listen to CameraMsg(video_mjpeg) and
           Image(display_mjpeg) messages.
        """
        camera_sub_cb_grp = ReentrantCallbackGroup()
        self.create_subscription(CameraMsg,
                                 constants.CAMERA_MSG_TOPIC,
                                 self.camera_callback,
                                 10,
                                 callback_group=camera_sub_cb_grp)
        display_img_sub_cb_grp = ReentrantCallbackGroup()
        self.create_subscription(Image,
                                 constants.DISPLAY_MSG_TOPIC,
                                 self.display_callback,
                                 10,
                                 callback_group=display_img_sub_cb_grp)

    def display_callback(self, data):
        """Callback method for camera message subscription.

        Args:
            data (Image): Message object with images from DeepRacer cameras.
        """
        if self.enable_overlay_publish:
            self.publish_lidar_overlay_image(data)

    def camera_callback(self, data):
        """Callback method for camera message subscription.

        Args:
            data (camera_msg): Message object with images from DeepRacer cameras.
        """
        try:
            self.camera_buffer.put(data.images)
            self.publish_sensor_message()
        except Exception as ex:
            self.get_logger().error(f"Error in camera callback: {ex}")

    def lidar_callback(self, data):
        """Callback method for scan message subscription.

        Args:
            data (LaserScan): Message object with raw data from LiDAR.
        """
        try:
            lidar_data = list()
            lidar_degrees = list()
            overlay_lidar_data = list()
            overlay_lidar_degrees = list()

            for i in range(len(data.ranges)):
                degree = utils.rad2deg(data.angle_min + data.angle_increment * i)
                # Select the distance for the angle from min_lidar_angle to max_lidar_angle set as per the
                # lidar configuration of the model selected
                if degree >= self.lidar_configuration[constants.LidarConfigurationKeys.MIN_LIDAR_ANGLE] and \
                   degree <= self.lidar_configuration[constants.LidarConfigurationKeys.MAX_LIDAR_ANGLE]:
                    # If the distance value if "inf", ignore and set it to be the max lidar dist
                    if data.ranges[i] == float("inf"):
                        lidar_data.append(self.max_lidar_dist)
                    else:
                        # Select the distance values in the min to max range of distance
                        # as per the lidar configuration of the model selected
                        lidar_data.append(min(max(data.ranges[i],
                                                  self.lidar_configuration[
                                                        constants.
                                                        LidarConfigurationKeys.
                                                        MIN_LIDAR_DIST]),
                                              self.max_lidar_dist))
                    lidar_degrees.append(degree)
                if data.ranges[i] == float("inf"):
                    overlay_lidar_data.append(self.lidar_overlay_configuration[
                                                    constants.
                                                    LidarOverlayConfigurationKeys.
                                                    MAX_LIDAR_DIST])
                else:
                    # Select the distance values in the min to max range of distance
                    # as per the lidar configuration of the model selected
                    overlay_lidar_data.append(min(max(data.ranges[i],
                                                      self.lidar_overlay_configuration[
                                                            constants.
                                                            LidarOverlayConfigurationKeys.
                                                            MIN_LIDAR_DIST]),
                                                  self.lidar_overlay_configuration[
                                                        constants.
                                                        LidarOverlayConfigurationKeys.
                                                        MAX_LIDAR_DIST]))
                overlay_lidar_degrees.append(degree)

            # interpolate the lidar data
            desired_lidar_degrees = np.linspace(self.lidar_configuration[
                                                    constants.
                                                    LidarConfigurationKeys.
                                                    MIN_LIDAR_ANGLE],
                                                self.lidar_configuration[
                                                    constants.
                                                    LidarConfigurationKeys.
                                                    MAX_LIDAR_ANGLE],
                                                num=self.lidar_configuration[
                                                    constants.
                                                    LidarConfigurationKeys.
                                                    NUM_LIDAR_VALUES])

            # numpy array
            lidar_data_interp = np.interp(desired_lidar_degrees, lidar_degrees, lidar_data)

            self.lidar_buffer.put(self.lidar_preprocessing_obj.preprocess_data(lidar_data_interp).tolist())

            # interpolate lidar data for overlay
            desired_overlay_lidar_degrees = np.linspace(constants.
                                                        DEFAULT_LIDAR_OVERLAY_CONFIGURATION[
                                                            constants.
                                                            LidarConfigurationKeys.
                                                            MIN_LIDAR_ANGLE],
                                                        constants.
                                                        DEFAULT_LIDAR_OVERLAY_CONFIGURATION[
                                                            constants.
                                                            LidarConfigurationKeys.
                                                            MAX_LIDAR_ANGLE],
                                                        num=constants.
                                                        DEFAULT_LIDAR_OVERLAY_CONFIGURATION[
                                                            constants.
                                                            LidarConfigurationKeys.
                                                            NUM_LIDAR_VALUES])

            # numpy array
            overlay_lidar_data_interp = np.interp(desired_overlay_lidar_degrees,
                                                  overlay_lidar_degrees,
                                                  overlay_lidar_data)
            self.overlay_lidar_buffer.put(
                self.lidar_overlay_preprocessing_obj.preprocess_data(overlay_lidar_data_interp))
        except Exception as ex:
            self.get_logger().error(f"Error in LiDAR callback: {ex}")

    def publish_sensor_message(self):
        """Publish the sensor message when we get new data for the slowest sensor(LiDAR).
        """
        try:
            sensor_msg = EvoSensorMsg()
            if self.lidar_configuration[constants.LidarConfigurationKeys.USE_LIDAR]:
                self.lidar_buffer.get()
            self.camera_buffer.get()
            if self.lidar_configuration[constants.LidarConfigurationKeys.USE_LIDAR]:
                try:
                    lidar_data = self.lidar_buffer.get_nowait()
                except Exception as ex:
                    self.get_logger().error(f"Error while reading lidar data: {ex}")
            try:
                camera_images = self.camera_buffer.get_nowait()
            except Exception as ex:
                self.get_logger().error(f"Error while reading camera data: {ex}")

            sensor_msg.images = camera_images
            if self.lidar_configuration[constants.LidarConfigurationKeys.USE_LIDAR]:
                sensor_msg.lidar_data = lidar_data
            else:
                sensor_msg.lidar_data = \
                    [self.max_lidar_dist] * \
                    self.lidar_configuration[constants.LidarConfigurationKeys.NUM_LIDAR_VALUES]
            self.sensor_message_publisher.publish(sensor_msg)
        except Exception as ex:
            self.get_logger().error(f"Error in publishing sensor message: {ex}")

    def publish_lidar_overlay_image(self, image):
        """Publish the overlay image combining camera and LiDAR data.

        Args:
            image (Image): Display image data.
        """
        overlay_lidar_data = list()
        try:
            overlay_lidar_data = self.overlay_lidar_buffer.get_nowait()
        except utils.DoubleBuffer.Empty:
            overlay_lidar_data = []
        except Exception as ex:
            self.get_logger().error(f"Error while reading overlay lidar data: {ex}")

        overlay_image = self.lidar_overlay_obj.overlay_lidar_data_on_image(
                            self.bridge.imgmsg_to_cv2(image),
                            overlay_lidar_data)
        try:
            self.overlay_image_publisher.publish(
                self.bridge.cv2_to_imgmsg(overlay_image,
                                          encoding="bgr8")
                )
        except Exception as e:
            self.get_logger().error(e)


def main(args=None):
    rclpy.init(args=args)
    sensor_fusion_node = SensorFusionNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(sensor_fusion_node, executor)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_fusion_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
