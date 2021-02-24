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
lidar_preprocess.py

This module defines the interface for the lidar preprocessing and different
preprocessing methods.
"""

import abc
import numpy as np


class LidarPreprocessingInterface():
    """ This class defines an interface for LiDAR preprocessing, it defines
        the basic functionality required by all preprocess types.
    """
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def preprocess_data(self, lidar_data):
        """Abstract method that has to be implemented in the classes that implement
           this interface with the details of preprocessing the interpolated LiDAR
           data passed as input.

        Args:
            lidar_data (np.array): This function expects a interpolated np.array
                                   with the distance information.

        Raises:
            NotImplementedError: Abstract method to force override this method when
                                 implementing thisinterface.
        """
        raise NotImplementedError("Lidar preprocessing object must implement a way "
                                  "to preprocess data")


class DefaultPreprocessing(LidarPreprocessingInterface):
    """Default preprocessing method that returns the interpolated LiDAR data as is.

    Args:
        LidarPreprocessingInterface: Implements the LidarPreprocessingInterface.
    """
    def preprocess_data(self, lidar_data):
        """Return the interpolated LiDAR data as is without any modification.

        Args:
            lidar_data (np.array): LiDAR distance data.

        Returns:
            np.array: LiDAR distance data.
        """
        return lidar_data


class SectorPreprocessing(LidarPreprocessingInterface):
    """Preprocessing method that is used for SECTOR_LIDAR models where the interpolated LiDAR
       distance information is further preprocessed into binary sector values to indicate there
       was any obstacle present in that sector.

    Args:
        LidarPreprocessingInterface: Implements the LidarPreprocessingInterface.
    """
    def __init__(self, num_sectors, max_lidar_dist):
        """Create the SectorPreprocessing object.

        Args:
            num_sectors (int): Number of sectors to split the LiDAR data.
            max_lidar_dist (float): Maximum distance until which we look for an
                                    obstacle in the sector.
        """
        self.num_sectors = num_sectors
        self.max_lidar_dist = max_lidar_dist

    def preprocess_data(self, lidar_data):
        """Split the LiDAR distance array into sectors and flag 1 if there is an distance
           value less than the maximum distance value.

        Args:
            lidar_data (np.array): Interpolated LiDAR distance data.

        Returns:
            np.array: Array of shape [num_sectors] with values 1/0 indicating if there is
                      an obstacle in that sector.
        """
        lidar_data = np.min(lidar_data.reshape(-1, self.num_sectors), axis=1)
        return (lidar_data < self.max_lidar_dist).astype(float)
