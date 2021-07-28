///////////////////////////////////////////////////////////////////////////////////
//   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          //
//                                                                               //
//   Licensed under the Apache License, Version 2.0 (the "License").             //
//   You may not use this file except in compliance with the License.            //
//   You may obtain a copy of the License at                                     //
//                                                                               //
//       http://www.apache.org/licenses/LICENSE-2.0                              //
//                                                                               //
//   Unless required by applicable law or agreed to in writing, software         //
//   distributed under the License is distributed on an "AS IS" BASIS,           //
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    //
//   See the License for the specific language governing permissions and         //
//   limitations under the License.                                              //
///////////////////////////////////////////////////////////////////////////////////

#ifndef LIDAR_OVERLAY_HPP
#define LIDAR_OVERLAY_HPP

#include <math.h>
#include "opencv2/opencv.hpp"
#include <bitset>

namespace SensorFusion {
    // Keys in the sensor fusion node lidar overlay configuration dictionary
    const std::string LIDAR_OVERLAY_CONFIG_NUM_LIDAR_VALUES_KEY = "num_values";
    const std::string LIDAR_OVERLAY_CONFIG_MIN_LIDAR_ANGLE_KEY = "min_angle";
    const std::string LIDAR_OVERLAY_CONFIG_MAX_LIDAR_ANGLE_KEY = "max_angle";
    const std::string LIDAR_OVERLAY_CONFIG_MIN_LIDAR_DIST_KEY = "min_distance";
    const std::string LIDAR_OVERLAY_CONFIG_MAX_LIDAR_DIST_KEY = "max_distance";
    const std::string LIDAR_OVERLAY_CONFIG_NUM_LIDAR_SECTORS_KEY = "num_sectors";
    const std::string LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_ALPHA_KEY = "lidar_overlay_alpha";
    const std::string LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_LINE_WIDTH_KEY = "lidar_overlay_line_width";
    const std::string LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_NUM_SECTORS_KEY = "lidar_overlay_num_sectors";

    // BGR - White
    const cv::Scalar LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_LINE_COLOR = cv::Scalar(255, 255, 255);
    // BGR - Orange - #FA9A50
    const cv::Scalar LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_FILL_COLOR = cv::Scalar(80, 154, 250);
    // BGR - Gray - #C0C0C0
    const cv::Scalar LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_BACKGROUND_COLOR = cv::Scalar(192, 192, 192);
    class LidarOverlay{
       public:
            LidarOverlay() = default;
            ~LidarOverlay();
            /// Initialize the LiDAR overlay configuration.
            /// @param lidarOverlayConfiguration Map with the overlay configuration
            /// @param imageWidth Width of the overlay image that will be combined with the camera image
            /// @param imageHeight Height of the overlay image that will be combined with the camera image
            void init(std::unordered_map<std::string, float> lidarOverlayConfiguration,
                      int imageWidth,
                      int imageHeight);
            /// Overlay the binary sector LiDAR data on the camera image and return overlayed image.
            /// @param image Camera image on which the LiDAR data will be added
            /// @param sectorLidarData Binary values for sectorized LiDAR data
            cv::Mat overlayLidarDataOnImage(const cv::Mat& image, const std::bitset<8>& sectorLidarData);
        private:
            /// Load the overlay cache with an image corresponding to the binary sectorized LiDAR data passed
            /// @param sectorLidarData Binary values for sectorized LiDAR data
            void loadLidarOverlayCache(const std::bitset<8>& sectorLidarData);
            /// Create fill coordinates based on the overlay image height and width
            void createFillCoordinates();
            /// Draw sector seperator lines on the image passed
            /// @param image Overlay image on which the sector separator lines will be drawn
            void drawSectorSeparatorLinesOnImage(const cv::Mat& image);
            /// Hash map that stores the LiDAR overlay configuration data
            std::unordered_map<std::string, float> lidarOverlayConfiguration_;
            /// Hash map that caches the overlay images for different LiDAR values
            std::unordered_map<int, cv::Mat> lidarOverlayCache_;
            /// Vector of sector end points on the overlay image
            std::vector<cv::Point> sectorEndPoints_;
            /// Coordinates of the center point of the image
            cv::Point centerPoint_;
            /// Map of coordinates for each sector
            std::unordered_map<int, std::vector<std::vector<cv::Point>>> sectorCoordinatesMap_;
            /// Width of the overlay image that will be combined with the camera image
            int imageWidth_;
            /// Height of the overlay image that will be combined with the camera image
            int imageHeight_;
    };
}
#endif
