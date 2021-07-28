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

/*
This module defines the LidarOverlay class that manages the functions to create an
8 sector overlay image based on the image passed. The sectors are currently hardcoded
to start from 150 degree to -150 degree with 0 degree marker at the rear of the car.
*/

#include "sensor_fusion_pkg/lidar_overlay.hpp"

#define DEG2RAD(x) ((x)*M_PI/180.)

namespace SensorFusion {

    LidarOverlay::~LidarOverlay() = default;

    /// Initialize the LiDAR overlay configuration.
    /// @param lidarOverlayConfiguration Map with the overlay configuration
    /// @param imageWidth Width of the overlay image that will be combined with the camera image
    /// @param imageHeight Height of the overlay image that will be combined with the camera image
    void LidarOverlay::init(std::unordered_map<std::string, float> lidarOverlayConfiguration,
                       int imageWidth,
                       int imageHeight)
    {
        lidarOverlayConfiguration_ = lidarOverlayConfiguration;
        imageWidth_ = imageWidth;
        imageHeight_ = imageHeight;
        createFillCoordinates();
    }

    /// Overlay the binary sector LiDAR data on the camera image and return overlayed image.
    /// @param image Camera image on which the LiDAR data will be added
    /// @param sectorLidarData Binary values for sectorized LiDAR data
    cv::Mat LidarOverlay::overlayLidarDataOnImage(const cv::Mat& image, const std::bitset<8>& sectorLidarData){
        int cacheKey = (int)(sectorLidarData.to_ulong());
        if(lidarOverlayCache_.find(cacheKey) == lidarOverlayCache_.end()){
            loadLidarOverlayCache(sectorLidarData);
        }
        cv::Mat finalImage;
        cv::addWeighted(image,
                        lidarOverlayConfiguration_[LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_ALPHA_KEY],
                        lidarOverlayCache_[cacheKey],
                        1.0 - lidarOverlayConfiguration_[LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_ALPHA_KEY],
                        0.0,
                        finalImage);
        return finalImage;
    }

    /// Load the overlay cache with an image corresponding to the binary sectorized LiDAR data passed
    /// @param sectorLidarData Binary values for sectorized LiDAR data
    void LidarOverlay::loadLidarOverlayCache(const std::bitset<8>& sectorLidarData){
        auto overlay = cv::Mat(imageHeight_,
                               imageWidth_,
                               CV_8UC3,
                               LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_BACKGROUND_COLOR);
        
        int cacheKey = (int)(sectorLidarData.to_ulong());
        if(lidarOverlayCache_.find(cacheKey) == lidarOverlayCache_.end()){
            for(size_t sector_idx = 0; sector_idx < sectorLidarData.size(); sector_idx++){
                if(sectorLidarData[sector_idx] && sector_idx < lidarOverlayConfiguration_[LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_NUM_SECTORS_KEY]){
                    cv::fillPoly(overlay,
                                 sectorCoordinatesMap_[sector_idx],
                                 LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_FILL_COLOR);
                }
            }
            drawSectorSeparatorLinesOnImage(overlay);
            lidarOverlayCache_[cacheKey] = overlay;
        }
    }

    /// Create fill coordinates based on the overlay image height and width
    void LidarOverlay::createFillCoordinates() {
    /*
        topLeftCorner       topRightCorner
                    p0   p12
                 _______________       
            p2  |  0 \     /  7,|  p10
                | `   \   / ,   |
            p3  |__1_`_\ /____6_|  p9
                |      /|\      |
                |  2 /  |  \  5 |
                |__/__3_|_4__\__|
                    p5         p7     
        bottomLeftCorner    bottomRightCorner
                    bottomCenter
                                    (p3 and p9 are not at center)
                                    (image not to scale)
    */  
        // Create end points required for the fill coordinates        
        centerPoint_ = cv::Point(imageWidth_/2, imageHeight_/2);
        cv::Point bottomcenterPoint_ = cv::Point(imageWidth_/2, imageHeight_);
        cv::Point topLeftCornerPoint = cv::Point(0, 0);
        cv::Point topRightCornerPoint = cv::Point(imageWidth_, 0);
        cv::Point bottomLeftCornerPoint = cv::Point(0, imageHeight_);
        cv::Point bottomRightCornerPoint = cv::Point(imageWidth_, imageHeight_);
        // Left half
        cv::Point p0 = cv::Point((imageWidth_/2) - tan(DEG2RAD(30)) * (imageHeight_/2), 0);
        cv::Point p2 = cv::Point(0, tan(DEG2RAD(22.5))*(imageWidth_/2));
        cv::Point p3 = cv::Point(0, (imageHeight_/2) + tan(DEG2RAD(15))*(imageWidth_/2));
        cv::Point p5 = cv::Point((imageWidth_/2) - tan(DEG2RAD(37.5)) * (imageHeight_/2), imageHeight_);
        // Right half
        cv::Point p7 = cv::Point((imageWidth_/2) + tan(DEG2RAD(37.5)) * (imageHeight_/2), imageHeight_);
        cv::Point p9 = cv::Point(imageWidth_, (imageHeight_/2) + tan(DEG2RAD(15)) * (imageWidth_/2));
        cv::Point p10 = cv::Point(imageWidth_, tan(DEG2RAD(22.5)) * (imageWidth_/2));
        cv::Point p12 = cv::Point((imageWidth_/2) + tan(DEG2RAD(30)) * (imageHeight_/2), 0);

        // Sector 0 - -150 degrees to -112.5 degrees
        std::vector<std::vector<cv::Point>> sector0 = {{{centerPoint_},
                     {p0},
                     {topLeftCornerPoint},
                     {p2}}};
        // Sector 1 - -112.5 degrees to -75 degrees
        std::vector<std::vector<cv::Point>> sector1 = {{{centerPoint_},
                     {p2},
                     {p3}}};
        // Sector 2 - -75 degrees to -37.5 degrees
        std::vector<std::vector<cv::Point>> sector2 = {{{centerPoint_},
                     {p3},
                     {bottomLeftCornerPoint},
                     {p5}}};
        // Sector 3 - -37.5 degrees to 0 degrees
        std::vector<std::vector<cv::Point>> sector3 = {{{centerPoint_},
                     {p5},
                     {bottomcenterPoint_}}};
        // Sector 4 - 0 degrees to 37.5 degrees
        std::vector<std::vector<cv::Point>> sector4 = {{{centerPoint_},
                     {p7},
                     {bottomcenterPoint_}}};
        // Sector 5 - 37.5 degrees to 75 degrees
        std::vector<std::vector<cv::Point>> sector5 = {{{centerPoint_},
                     {p9},
                     {bottomRightCornerPoint},
                     {p7}}};
        // Sector 6 - 75 degrees to 112.5 degrees
        std::vector<std::vector<cv::Point>> sector6 = {{{centerPoint_},
                     {p10},
                     {p9}}};
        // Sector 7 - 112.5 degrees to 150 degrees
        std::vector<std::vector<cv::Point>> sector7 = {{{centerPoint_},
                     {p12},
                     {topRightCornerPoint},
                     {p10}}};
        sectorCoordinatesMap_[0] = sector0;
        sectorCoordinatesMap_[1] = sector1;
        sectorCoordinatesMap_[2] = sector2;
        sectorCoordinatesMap_[3] = sector3;
        sectorCoordinatesMap_[4] = sector4;
        sectorCoordinatesMap_[5] = sector5;
        sectorCoordinatesMap_[6] = sector6;
        sectorCoordinatesMap_[7] = sector7;
        // Save sector end points to draw sector seperator lines
        sectorEndPoints_.push_back(p0);
        sectorEndPoints_.push_back(p2);
        sectorEndPoints_.push_back(p3);
        sectorEndPoints_.push_back(p5);
        sectorEndPoints_.push_back(bottomcenterPoint_);
        sectorEndPoints_.push_back(p7);
        sectorEndPoints_.push_back(p9);
        sectorEndPoints_.push_back(p10);
        sectorEndPoints_.push_back(p12);                                                    
    }
    /// Draw sector seperator lines on the image passed
    /// @param image Overlay image on which the sector separator lines will be drawn
    void LidarOverlay::drawSectorSeparatorLinesOnImage(const cv::Mat& image) {
        for(auto endPoint : sectorEndPoints_) {
            cv::line(image,
                     centerPoint_,
                     endPoint,
                     LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_LINE_COLOR,
                     lidarOverlayConfiguration_[LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_LINE_WIDTH_KEY]);
        }
    }
}