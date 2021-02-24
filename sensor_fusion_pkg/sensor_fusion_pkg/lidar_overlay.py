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
lidar_overlay.py

This module defines the LidarOverlay class that manages the functions to create an
8 sector overlay image based on the image passed. The sectors are currently hardcoded
to start from 150 degree to -150 degree with 0 degree marker at the rear of the car.
"""

import numpy as np
import cv2
import math

from sensor_fusion_pkg import constants


class LidarOverlay():
    """ Class used for managing the LiDAR overlay creation on the camera images.
    """

    def __init__(self, lidar_overlay_configuration):
        """Create the LidarOverlay object.

        Args:
            lidar_overlay_configuration (dict): LiDAR overlay configuration object
                                                with details about max_dist,
                                                min_dist, num_sectors, etc.
        """
        self.lidar_overlay_cache = dict()
        self.lidar_overlay_configuration = lidar_overlay_configuration

    def load_lidar_overlay_cache(self, lidar_data):
        """Create an overlay image for the particular lidar_data array passed and store it
           as part of the cache.

        Args:
               lidar_data (np.array): Array of shape [num_sectors] with values 1/0 indicating
                                      if there is an obstacle in that sector.
        """
        image = np.zeros((120, 160, 3), np.uint8)
        # Fill image with background color
        image[:] = self.lidar_overlay_configuration[
                            constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_BACKGROUND_COLOR]
        if tuple(lidar_data) not in self.lidar_overlay_cache:
            overlay = image.copy()
            for idx, value in enumerate(list(lidar_data)):
                if value:
                    if idx < self.lidar_overlay_configuration[
                            constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_NUM_SECTORS]:
                        cv2.fillPoly(overlay,
                                     self.get_fill_coordinates(overlay.shape[1], overlay.shape[0], idx),
                                     self.lidar_overlay_configuration[
                                            constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_FILL_COLOR])
            self.draw_sector_seperator_lines(overlay)
            self.lidar_overlay_cache[tuple(lidar_data)] = overlay

    def get_fill_coordinates(self, image_width, image_height, sector_idx):
        """Helper function to create and return the fill coordinates on the image for
           the sector index passed for an 8 sector image.
           The sectors are from 150 degree to -150 degree with 0 degree marker at the rear
           of the car.

        Args:
               image_width (int): Width of the image.
               image_height (int): Height of the image.
               sector_idx (int): Sector index.

        Returns:
               np.array: Set of points in clockwise direction for the sector forming the
                         fill section on the image.
        """
        # Each sector is a set of points in clockwise direction starting from center
        lidar_mapping = {
            # Sector 0 - -150 degrees to -112.5 degrees
            0: np.array([[[int(image_width/2), int(image_height/2)],
                        [int(image_width/2-math.tan(math.radians(30))*int(image_height/2)), 0],
                        [0, 0],
                        [0, int(math.tan(math.radians(22.5))*image_width/2)]]],
                        dtype='int32'),
            # Sector 1 - -112.5 degrees to -75 degrees
            1: np.array([[[int(image_width/2), int(image_height/2)],
                        [0, int(math.tan(math.radians(22.5))*image_width/2)],
                        [0, int(image_height/2 + math.tan(math.radians(15))*image_width/2)]]],
                        dtype='int32'),
            # Sector 2 - -75 degrees to -37.5 degrees
            2: np.array([[[int(image_width/2), int(image_height/2)],
                        [0, int(image_height/2 + math.tan(math.radians(15))*image_width/2)],
                        [0, image_height],
                        [int(image_width/2 - math.tan(math.radians(37.5))*image_height/2), int(image_height)]]],
                        dtype='int32'),
            # Sector 3 - -37.5 degrees to 0 degrees
            3: np.array([[[int(image_width/2), int(image_height/2)],
                        [int(image_width/2 - math.tan(math.radians(37.5))*image_height/2), int(image_height)],
                        [int(image_width/2), int(image_height)]]],
                        dtype='int32'),
            # Sector 4 - 0 degrees to 37.5 degrees
            4: np.array([[[int(image_width/2), int(image_height/2)],
                        [int(image_width/2 + math.tan(math.radians(37.5))*image_height/2), int(image_height)],
                        [int(image_width/2), int(image_height)]]],
                        dtype='int32'),
            # Sector 5 - 37.5 degrees to 75 degrees
            5: np.array([[[int(image_width/2), int(image_height/2)],
                        [int(image_width), int(image_height/2 + math.tan(math.radians(15))*image_width/2)],
                        [int(image_width), image_height],
                        [int(image_width/2 + math.tan(math.radians(37.5))*image_height/2), int(image_height)]]],
                        dtype='int32'),
            # Sector 6 - 75 degrees to 112.5 degrees
            6: np.array([[[int(image_width/2), int(image_height/2)],
                        [int(image_width), int(math.tan(math.radians(22.5))*image_width/2)],
                        [int(image_width), int(image_height/2 + math.tan(math.radians(15))*image_width/2)]]],
                        dtype='int32'),
            # Sector 6 - 112.5 degrees to 150 degrees
            7: np.array([[[int(image_width/2), int(image_height/2)],
                        [int(image_width/2+math.tan(math.radians(30))*int(image_height/2)), 0],
                        [int(image_width), 0],
                        [int(image_width), int(math.tan(math.radians(22.5))*image_width/2)]]],
                        dtype='int32')
        }
        return lidar_mapping[sector_idx]

    def draw_sector_seperator_lines(self, image):
        """Helper function to draw the sector seperator lines on the overlay image.

        Args:
               image (np.array): Overlay image with the sector filled in.
        """
        image_width = image.shape[1]
        image_height = image.shape[0]
        # Counter clockwise from the top left
        center_vertex = (int(image_width/2), int(image_height/2))
        # -150 degrees
        cv2.line(image, center_vertex, (int(image_width/2-math.tan(math.radians(30))*int(image_height/2)), 0),
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_COLOR],
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_WIDTH])
        # -112.5 degrees
        cv2.line(image, center_vertex, (0, int(math.tan(math.radians(22.5))*image_width/2)),
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_COLOR],
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_WIDTH])
        # -75 degrees
        cv2.line(image, center_vertex, (0, int(image_height/2 + math.tan(math.radians(15))*image_width/2)),
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_COLOR],
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_WIDTH])
        # -37.5 degrees
        cv2.line(image, center_vertex, (int(image_width/2 - math.tan(math.radians(37.5))*image_height/2),
                 int(image_height)),
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_COLOR],
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_WIDTH])
        # 0 degrees
        cv2.line(image, center_vertex, (int(image_width/2), int(image_height)),
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_COLOR],
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_WIDTH])
        # 37.5 degrees
        cv2.line(image, center_vertex, (int(image_width/2 + math.tan(math.radians(37.5))*image_height/2),
                 int(image_height)),
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_COLOR],
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_WIDTH])
        # 75 degrees
        cv2.line(image, center_vertex, (int(image_width),
                 int(image_height/2 + math.tan(math.radians(15))*image_width/2)),
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_COLOR],
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_WIDTH])
        # 112.5 degrees
        cv2.line(image, center_vertex, (int(image_width), int(math.tan(math.radians(22.5))*image_width/2)),
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_COLOR],
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_WIDTH])
        # 150 degrees
        cv2.line(image, center_vertex, (int(image_width/2+math.tan(math.radians(30))*int(image_height/2)), 0),
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_COLOR],
                 self.lidar_overlay_configuration[
                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_LINE_WIDTH])

    def overlay_lidar_data_on_image(self, image, lidar_data):
        """Main function to combine the overlay image on the camera image based on the
           lidar_data passed as a parameter.

            Args:
                image (list): Camera frame.
                lidar_data (np.array): LiDAR sector data with binary 1/0 marker indicating
                                       obstacles.

            Returns:
                list: Image with the LiDAR sector data overlayed on the camera frame.
        """
        if tuple(lidar_data) not in self.lidar_overlay_cache:
            self.load_lidar_overlay_cache(lidar_data)
        # Following line overlays transparent rectangle over the image
        image_new = cv2.addWeighted(image,
                                    self.lidar_overlay_configuration[
                                        constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_ALPHA],
                                    cv2.resize(self.lidar_overlay_cache[tuple(lidar_data)],
                                               (image.shape[1], image.shape[0])),
                                    1 - self.lidar_overlay_configuration[
                                            constants.LidarOverlayConfigurationKeys.LIDAR_OVERLAY_ALPHA],
                                    0)
        return image_new
