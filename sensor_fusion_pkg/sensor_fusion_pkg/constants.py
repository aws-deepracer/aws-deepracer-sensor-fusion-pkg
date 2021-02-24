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

from enum import Enum

SENSOR_MSG_TOPIC = "sensor_msg"
OVERLAY_MSG_TOPIC = "overlay_msg"
SENSOR_DATA_STATUS_SERVICE_NAME = "sensor_data_status"
CONFIGURE_LIDAR_SERVICE_NAME = "configure_lidar"

CAMERA_PKG_NS = "/camera_pkg"
CAMERA_MSG_TOPIC = f"{CAMERA_PKG_NS}/video_mjpeg"
DISPLAY_MSG_TOPIC = f"{CAMERA_PKG_NS}/display_mjpeg"

RPLIDAR_ROS_PKG_NS = "/rplidar_ros"
LIDAR_MSG_TOPIC = f"{RPLIDAR_ROS_PKG_NS}/scan"

SENSOR_CONFIGURATION_FILE_PATH = "/opt/aws/deepracer/sensor_configuration.json"


class LidarPreprocessingTypes(Enum):
    """Enum with lidar preprocessing keys.
    """
    # No processing
    DEFAULT = 0
    # Preprocess the lidar values by splitting them to
    # sectors and converting to binary values
    SECTOR = 1


class SensorConfigurationKeys():
    """Class with keys in the sensor configuration file.
    """
    LIDAR = "lidar"
    LIDAR_OVERLAY = "lidar_overlay"


class LidarConfigurationKeys():
    """Class with keys in the sensor fusion node lidar configuration dictionary.
    """
    USE_LIDAR = "use_lidar"
    NUM_LIDAR_VALUES = "num_values"
    MIN_LIDAR_ANGLE = "min_angle"
    MAX_LIDAR_ANGLE = "max_angle"
    MIN_LIDAR_DIST = "min_distance"
    MAX_LIDAR_DIST = "max_distance"
    LIDAR_CLIPPING_DIST = "clipping_distance"
    NUM_LIDAR_SECTORS = "num_sectors"
    PREPROCESS_TYPE = "preprocess_type"

    @classmethod
    def validate_dict(cls, input_dict):
        """Helper function to validate the input dictionary. Will raise an exception
           if input dict does not contain all the keys.

        Args:
            input_dict (dict): Input dictionary to be verified.
        """
        for key in dir(cls):
            attr = getattr(cls, key)
            if not key.startswith('__') and not callable(attr):
                _ = input_dict[attr]


class LidarOverlayConfigurationKeys():
    """Class with keys in the sensor fusion node lidar overlay configuration dictionary.
    """
    NUM_LIDAR_VALUES = "num_values"
    MIN_LIDAR_ANGLE = "min_angle"
    MAX_LIDAR_ANGLE = "max_angle"
    MIN_LIDAR_DIST = "min_distance"
    MAX_LIDAR_DIST = "max_distance"
    NUM_LIDAR_SECTORS = "num_sectors"
    LIDAR_OVERLAY_ALPHA = "lidar_overlay_alpha"
    LIDAR_OVERLAY_FILL_COLOR = "lidar_overlay_fill_color"
    LIDAR_OVERLAY_LINE_COLOR = "lidar_overlay_line_color"
    LIDAR_OVERLAY_BACKGROUND_COLOR = "lidar_overlay_background_color"
    LIDAR_OVERLAY_LINE_WIDTH = "lidar_overlay_line_width"
    LIDAR_OVERLAY_NUM_SECTORS = "lidar_overlay_num_sectors"

    @classmethod
    def validate_dict(cls, input_dict):
        """Helper function to validate the input dictionary. Will raise an exception
           if input dict does not contain all the keys.

        Args:
            input_dict (dict): Input dictionary to be verified.
        """
        for key in dir(cls):
            attr = getattr(cls, key)
            if not key.startswith('__') and not callable(attr):
                _ = input_dict[attr]


# Default lidar configuration
DEFAULT_LIDAR_CONFIGURATION = {
    LidarConfigurationKeys.USE_LIDAR: False,
    LidarConfigurationKeys.MIN_LIDAR_ANGLE: -60.0,
    LidarConfigurationKeys.MAX_LIDAR_ANGLE: 60.0,
    LidarConfigurationKeys.MIN_LIDAR_DIST: 0.15,
    LidarConfigurationKeys.MAX_LIDAR_DIST: 1.0,
    LidarConfigurationKeys.LIDAR_CLIPPING_DIST: 1.0,
    LidarConfigurationKeys.NUM_LIDAR_VALUES: 64,
    LidarConfigurationKeys.NUM_LIDAR_SECTORS: 64,
    # Default value set to 0 - No preprocessing required
    LidarConfigurationKeys.PREPROCESS_TYPE: LidarPreprocessingTypes.DEFAULT.value
}

# Default lidar overlay configuration
DEFAULT_LIDAR_OVERLAY_CONFIGURATION = {
    LidarOverlayConfigurationKeys.MIN_LIDAR_ANGLE: -150.0,
    LidarOverlayConfigurationKeys.MAX_LIDAR_ANGLE: 150.0,
    LidarOverlayConfigurationKeys.MIN_LIDAR_DIST: 0.15,
    LidarOverlayConfigurationKeys.MAX_LIDAR_DIST: 0.5,
    LidarOverlayConfigurationKeys.NUM_LIDAR_VALUES: 64,
    LidarOverlayConfigurationKeys.NUM_LIDAR_SECTORS: 8,
    # Lidar overlay constants
    # Transparency factor.
    LidarOverlayConfigurationKeys.LIDAR_OVERLAY_ALPHA: 0.8,
    # BGR - Orange - #FA9A50
    LidarOverlayConfigurationKeys.LIDAR_OVERLAY_FILL_COLOR: (80, 154, 250),
    # BGR - White
    LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_COLOR: (255, 255, 255),
    # BGR - Gray - #C0C0C0
    LidarOverlayConfigurationKeys.LIDAR_OVERLAY_BACKGROUND_COLOR: (192, 192, 192),
    # px
    LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_WIDTH: 1,
    # number of sectors
    LidarOverlayConfigurationKeys.LIDAR_OVERLAY_NUM_SECTORS: 8
}
