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
    sensorMsgPub_: A publisher to publish the message that combines
                   the camera data published from the camera package and
                   the LiDAR data published from the rplidar_ros package
                   (preprocessed in this node as per the LiDAR configuration).
    overlayImagePub_: A publisher to publish the Image message that has the LiDAR
                      overaly data on top of the camera frame to highlight the sectors
                      with an obstacle present behind the DeepRacer device.
    statusCheckService_: A service to find out the data status of the cameras and the LiDAR
                         sensors. Based on whether the single camera/stereo camera/LiDAR data
                         is being read by the node, the corresponding sensor status are set.
    lidarConfigService_: A service to configure LiDAR processing information for each model.
                         It allows to set the LiDAR preprocessing information like the maximum
                         distance, minimum distance, number of sectors etc., which will be used
                         to create the sensor message being published for running the inference
                         for the particular model selected.
*/

#include "rclcpp/rclcpp.hpp"
#include <sys/stat.h>
#include <thread>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <vector>
#include <bitset>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.hpp"

#include "deepracer_interfaces_pkg/msg/evo_sensor_msg.hpp"
#include "deepracer_interfaces_pkg/msg/camera_msg.hpp"
#include "deepracer_interfaces_pkg/srv/lidar_config_srv.hpp"
#include "deepracer_interfaces_pkg/srv/sensor_status_check_srv.hpp"

#include "sensor_fusion_pkg/lidar_overlay.hpp"
#include "sensor_fusion_pkg/utility.hpp"

// Convert radian values to degrees
#define RAD2DEG(x) ((x)*180./M_PI)

namespace SensorFusion {
    #define DEFAULT_IMAGE_WIDTH 160
    #define DEFAULT_IMAGE_HEIGHT 120

    // Default lidar configuration
    #define DEFAULT_LIDAR_CONFIGURATION_MIN_LIDAR_ANGLE -60.0
    #define DEFAULT_LIDAR_CONFIGURATION_MAX_LIDAR_ANGLE 60.0
    #define DEFAULT_LIDAR_CONFIGURATION_MIN_LIDAR_DIST 0.15
    #define DEFAULT_LIDAR_CONFIGURATION_MAX_LIDAR_DIST 1.0
    #define DEFAULT_LIDAR_CONFIGURATION_LIDAR_CLIPPING_DIST 1.0
    #define DEFAULT_LIDAR_CONFIGURATION_NUM_LIDAR_VALUES 64
    #define DEFAULT_LIDAR_CONFIGURATION_NUM_LIDAR_SECTORS 64
    // Default value set to 0 - No preprocessing required
    #define DEFAULT_LIDAR_CONFIGURATION_PREPROCESS_TYPE 0

    // Default lidar overlay configuration
    #define DEFAULT_LIDAR_OVERLAY_CONFIGURATION_MIN_LIDAR_ANGLE -150.0
    #define DEFAULT_LIDAR_OVERLAY_CONFIGURATION_MAX_LIDAR_ANGLE 150.0
    #define DEFAULT_LIDAR_OVERLAY_CONFIGURATION_MIN_LIDAR_DIST 0.15
    #define DEFAULT_LIDAR_OVERLAY_CONFIGURATION_MAX_LIDAR_DIST 0.5
    #define DEFAULT_LIDAR_OVERLAY_CONFIGURATION_NUM_LIDAR_VALUES 64
    #define DEFAULT_LIDAR_OVERLAY_CONFIGURATION_NUM_LIDAR_SECTORS 8
    // Lidar overlay constants
    // Transparency factor.
    #define DEFAULT_LIDAR_OVERLAY_CONFIGURATION_LIDAR_OVERLAY_ALPHA 0.8
    // px
    #define DEFAULT_LIDAR_OVERLAY_CONFIGURATION_LIDAR_OVERLAY_LINE_WIDTH 1
    // number of sectors
    #define DEFAULT_LIDAR_OVERLAY_CONFIGURATION_LIDAR_OVERLAY_NUM_SECTORS 8
    /// Enum that references the lidar preprocessing keys.
    enum LidarPreprocessingType {
        // No processing
        defaultProcessing,
        // Preprocess the lidar values by splitting them to
        // sectors and converting to binary values
        sector,
        numLidarPreprocessingType
    };

    // Message Topics to publish to.
    const char* SENSOR_MSG_TOPIC = "sensor_msg";
    const char* OVERLAY_MSG_TOPIC = "overlay_msg";
    const char* SENSOR_DATA_STATUS_SERVICE_NAME = "sensor_data_status";
    const char* CONFIGURE_LIDAR_SERVICE_NAME = "configure_lidar";

    // Message topics to subscribe to.
    const char* CAMERA_MSG_TOPIC = "/camera_pkg/video_mjpeg";
    const char* DISPLAY_MSG_TOPIC = "/camera_pkg/display_mjpeg";
    const char* LIDAR_MSG_TOPIC = "/rplidar_ros/scan";

    // Sensor configuration file path.
    const char* SENSOR_CONFIGURATION_FILE_PATH = "/opt/aws/deepracer/sensor_configuration.json";

    // Keys for the sensor configuration file
    const std::string LIDAR_KEY = "lidar";
    const std::string LIDAR_OVERLAY_KEY = "lidar_overlay";

    // Keys in the sensor fusion node lidar configuration dictionary
    const std::string LIDAR_CONFIG_NUM_LIDAR_VALUES_KEY = "num_values";
    const std::string LIDAR_CONFIG_MIN_LIDAR_ANGLE_KEY = "min_angle";
    const std::string LIDAR_CONFIG_MAX_LIDAR_ANGLE_KEY = "max_angle";
    const std::string LIDAR_CONFIG_MIN_LIDAR_DIST_KEY = "min_distance";
    const std::string LIDAR_CONFIG_MAX_LIDAR_DIST_KEY = "max_distance";
    const std::string LIDAR_CONFIG_NUM_LIDAR_SECTORS_KEY = "num_sectors";

    // Keys in the sensor fusion node lidar configuration dictionary
    const std::string LIDAR_CONFIG_LIDAR_CLIPPING_DIST_KEY = "clipping_distance";
    const std::string LIDAR_CONFIG_PREPROCESS_TYPE_KEY = "preprocess_type";

    class SensorFusionNode : public rclcpp::Node
    {
    /// This class creates the sensor_fusion_node responsible to collect data from the cameras and LiDAR
    /// and publish it as sensor message.

    public:

        /// Initialize the SensorFusionNode.
        /// @param node_name Name of the node to be created.
        SensorFusionNode(const std::string & node_name)
        : Node(node_name),
          lidarOverlayProcessingObj_(LidarOverlay()),
          enableOverlayPublish_(true),
          imageWidth_(DEFAULT_IMAGE_WIDTH),
          imageHeight_(DEFAULT_IMAGE_HEIGHT),
          node_handle_(std::shared_ptr<SensorFusionNode>(this, [](auto *) {})),
          image_transport_(node_handle_)
        {

            RCLCPP_INFO(this->get_logger(), "%s started", node_name.c_str());
            createDefaultSensorConfiguration();
            if (checkFile(SENSOR_CONFIGURATION_FILE_PATH)) {
                setSensorConfigurationFromFile(sensorConfiguration_,
                                              SENSOR_CONFIGURATION_FILE_PATH);
            }
            else {
                writeSensorConfigJSON(sensorConfiguration_,
                                     SENSOR_CONFIGURATION_FILE_PATH);
            }
            // Update the maximum LiDAR distance based on the sensor configuration.
            maxLiDARDist_ = std::min(sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_MAX_LIDAR_DIST_KEY],
                                     sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_LIDAR_CLIPPING_DIST_KEY]);

            // Define the LiDAR overlay configuration. Default to use SectorPreprocessing and
            // 8 sectors.
            lidarOverlayProcessingObj_.init(sensorConfiguration_[LIDAR_OVERLAY_KEY], imageWidth_, imageHeight_);

            // Publisher that sends combined sensor messages with camera data and LiDAR data
            // to inference node.
            sensorMsgPub_ = this->create_publisher<deepracer_interfaces_pkg::msg::EvoSensorMsg>(SENSOR_MSG_TOPIC, 1);

            // Publisher to publish the overlay message with sector LiDAR information
            // overlayed over the camera image frame.
            overlayImagePub_ = image_transport_.advertise(OVERLAY_MSG_TOPIC, 1);
            
            statusCheckService_ = this->create_service<deepracer_interfaces_pkg::srv::SensorStatusCheckSrv>(
                                                                                SENSOR_DATA_STATUS_SERVICE_NAME,
                                                                                std::bind(&SensorFusionNode::statusCheckCallback,
                                                                                          this,
                                                                                          std::placeholders::_1,
                                                                                          std::placeholders::_2,
                                                                                          std::placeholders::_3));

            lidarConfigService_ = this->create_service<deepracer_interfaces_pkg::srv::LidarConfigSrv>(
                                                                                CONFIGURE_LIDAR_SERVICE_NAME,
                                                                                std::bind(&SensorFusionNode::setLidarConfigCallback,
                                                                                          this,
                                                                                          std::placeholders::_1,
                                                                                          std::placeholders::_2,
                                                                                          std::placeholders::_3));
                                                                                         
            auto sensorMsgQOS = rclcpp::QoS(rclcpp::KeepLast(1));
            sensorMsgQOS.best_effort();
            // Subscriber to subscribe to the lidar scan messages published by the LiDAR.
            lidarSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(LIDAR_MSG_TOPIC,
                                                                               sensorMsgQOS,
                                                                               std::bind(&SensorFusionNode::lidarCB,
                                                                                         this,
                                                                                         std::placeholders::_1));
            // Subscriber to subscribe to the camera sensor messages published by the camera_pkg.
            cameraSub_ = this->create_subscription<deepracer_interfaces_pkg::msg::CameraMsg>(CAMERA_MSG_TOPIC,
                                                                                             sensorMsgQOS,
                                                                                             std::bind(&SensorFusionNode::cameraCB,
                                                                                                       this,
                                                                                                       std::placeholders::_1));

            // Subscriber to subscribe to the camera display messages published by the camera_pkg.
            displaySub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(DISPLAY_MSG_TOPIC,
                                                                             sensorMsgQOS,
                                                                             std::bind(&SensorFusionNode::displayCB,
                                                                                       this,
                                                                                       std::placeholders::_1));


            cameraImageCount_ = 0;
        }
        ~SensorFusionNode() = default;
    private:
        /// Create sensor configuration with default LiDAR configuration.
        void createDefaultSensorConfiguration(){
            sensorConfiguration_ = {
                {LIDAR_KEY, {
                    {LIDAR_CONFIG_MIN_LIDAR_ANGLE_KEY, DEFAULT_LIDAR_CONFIGURATION_MIN_LIDAR_ANGLE},
                    {LIDAR_CONFIG_MAX_LIDAR_ANGLE_KEY, DEFAULT_LIDAR_CONFIGURATION_MAX_LIDAR_ANGLE},
                    {LIDAR_CONFIG_MIN_LIDAR_DIST_KEY, DEFAULT_LIDAR_CONFIGURATION_MIN_LIDAR_DIST},
                    {LIDAR_CONFIG_MAX_LIDAR_DIST_KEY, DEFAULT_LIDAR_CONFIGURATION_MAX_LIDAR_DIST},
                    {LIDAR_CONFIG_LIDAR_CLIPPING_DIST_KEY, DEFAULT_LIDAR_CONFIGURATION_LIDAR_CLIPPING_DIST},
                    {LIDAR_CONFIG_NUM_LIDAR_VALUES_KEY, DEFAULT_LIDAR_CONFIGURATION_NUM_LIDAR_VALUES},
                    {LIDAR_CONFIG_NUM_LIDAR_SECTORS_KEY, DEFAULT_LIDAR_CONFIGURATION_NUM_LIDAR_SECTORS},
                    {LIDAR_CONFIG_PREPROCESS_TYPE_KEY, DEFAULT_LIDAR_CONFIGURATION_PREPROCESS_TYPE}}},
                {LIDAR_OVERLAY_KEY, {
                    {LIDAR_OVERLAY_CONFIG_MIN_LIDAR_ANGLE_KEY, DEFAULT_LIDAR_OVERLAY_CONFIGURATION_MIN_LIDAR_ANGLE},
                    {LIDAR_OVERLAY_CONFIG_MAX_LIDAR_ANGLE_KEY, DEFAULT_LIDAR_OVERLAY_CONFIGURATION_MAX_LIDAR_ANGLE},
                    {LIDAR_OVERLAY_CONFIG_MIN_LIDAR_DIST_KEY, DEFAULT_LIDAR_OVERLAY_CONFIGURATION_MIN_LIDAR_DIST},
                    {LIDAR_OVERLAY_CONFIG_MAX_LIDAR_DIST_KEY, DEFAULT_LIDAR_OVERLAY_CONFIGURATION_MAX_LIDAR_DIST},
                    {LIDAR_OVERLAY_CONFIG_NUM_LIDAR_VALUES_KEY, DEFAULT_LIDAR_OVERLAY_CONFIGURATION_NUM_LIDAR_VALUES},
                    {LIDAR_OVERLAY_CONFIG_NUM_LIDAR_SECTORS_KEY, DEFAULT_LIDAR_OVERLAY_CONFIGURATION_NUM_LIDAR_SECTORS},
                    {LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_ALPHA_KEY, DEFAULT_LIDAR_OVERLAY_CONFIGURATION_LIDAR_OVERLAY_ALPHA},
                    {LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_LINE_WIDTH_KEY, DEFAULT_LIDAR_OVERLAY_CONFIGURATION_LIDAR_OVERLAY_LINE_WIDTH},
                    {LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_NUM_SECTORS_KEY, DEFAULT_LIDAR_OVERLAY_CONFIGURATION_LIDAR_OVERLAY_NUM_SECTORS}}}};
        }

        /// Callback function for camera message subscription.
        /// @param msg Message with images from DeepRacer cameras.
        void cameraCB(const deepracer_interfaces_pkg::msg::CameraMsg::SharedPtr msg) {
            try {
                lastCameraMsgRecievedTime = std::chrono::steady_clock::now();
                cameraImageCount_ = msg->images.size();
                auto sensorMsg = deepracer_interfaces_pkg::msg::EvoSensorMsg();
                sensorMsg.images = msg->images;
                // Update the LiDAR data with sector LiDAR data based on preprocessing configuration
                if(sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_PREPROCESS_TYPE_KEY] == sector){
                    std::lock_guard<std::mutex> guard(lidarMutex_);
                    size_t blockSize = lidarData_.size()/sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_NUM_LIDAR_SECTORS_KEY];
                    auto sectorLidarData = binarySectorizeLidarData(lidarData_, blockSize, maxLiDARDist_);
                    sensorMsg.lidar_data = sectorLidarData;
                } else {
                    std::lock_guard<std::mutex> guard(lidarMutex_);
                    // Set LiDAR data by default
                    sensorMsg.lidar_data = lidarData_; 
                }
                this->sensorMsgPub_->publish(sensorMsg);  // Publish it along.
            }
            catch (const std::exception &ex) {
                RCLCPP_ERROR(this->get_logger(), "Camera callback failed: %s", ex.what());
            }
        }

        /// Callback function for camera message subscription.
        /// @param msg Message with images from DeepRacer cameras.
        void displayCB(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            try {
                std::bitset<8> sectorOverlayValues;
                auto displayMsg = sensor_msgs::msg::CompressedImage();
                {
                    std::lock_guard<std::mutex> guard(lidarMutex_);
                    size_t blockSize = overlayLidarData_.size()/sensorConfiguration_[LIDAR_OVERLAY_KEY][LIDAR_OVERLAY_CONFIG_LIDAR_OVERLAY_NUM_SECTORS_KEY];
                    if(blockSize == 8){
                        auto overlaySectorLidarData = binarySectorizeLidarData(overlayLidarData_,
                                                                               blockSize,
                                                                               sensorConfiguration_[LIDAR_OVERLAY_KEY][LIDAR_OVERLAY_CONFIG_MAX_LIDAR_DIST_KEY]);
                        for(size_t sector_idx = 0; sector_idx < overlaySectorLidarData.size(); sector_idx++){
                            sectorOverlayValues[sector_idx] = (int)overlaySectorLidarData[sector_idx];
                        }
                    }
                }
                cv::Mat resizedImg;
                cv::resize(cv_bridge::toCvCopy(msg, "bgr8")->image, resizedImg, cv::Size(imageWidth_, imageHeight_));
                cv::Mat overlayCVImage = lidarOverlayProcessingObj_.overlayLidarDataOnImage(resizedImg, sectorOverlayValues);
                if(enableOverlayPublish_) {
                    // Publish Lidar Overlay
                    overlayImagePub_.publish(*(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", overlayCVImage).toImageMsg().get()));
                }
            }
            catch (const std::exception &ex) {
                RCLCPP_ERROR(this->get_logger(), "Display callback failed: %s", ex.what());
            }
        }

        /// Callback function for LiDAR scan message subscription.
        /// @param msg Message with raw data from LiDAR.
        void lidarCB(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            try {
                lastLidarMsgRecievedTime = std::chrono::steady_clock::now();
                std::vector<float> overlayRanges, overlayDegrees, lidarRanges, lidarDegrees;
                for (size_t i = 0; i < msg->ranges.size(); i++) {
                    float degree = RAD2DEG(msg->angle_min + msg->angle_increment * i);
                    if(degree >= sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_MIN_LIDAR_ANGLE_KEY] &&
                       degree <= sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_MAX_LIDAR_ANGLE_KEY]){
                        if(std::isinf(msg->ranges[i])) {
                            lidarRanges.push_back(maxLiDARDist_);
                        }
                        else {
                            lidarRanges.push_back(std::min(std::max(static_cast<float>(msg->ranges[i]),
                            sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_MIN_LIDAR_DIST_KEY]),
                            maxLiDARDist_));
                        }
                        lidarDegrees.push_back(degree);
                    }
                    overlayRanges.push_back(std::min(std::max(static_cast<float>(msg->ranges[i]),
                                            sensorConfiguration_[LIDAR_OVERLAY_KEY][
                                             LIDAR_OVERLAY_CONFIG_MIN_LIDAR_DIST_KEY]),
                                            sensorConfiguration_[LIDAR_OVERLAY_KEY][
                                             LIDAR_OVERLAY_CONFIG_MAX_LIDAR_DIST_KEY]));
                    overlayDegrees.push_back(degree);
                }

                auto desiredLidarDegrees = linspace(sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_MIN_LIDAR_ANGLE_KEY],
                                                    sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_MAX_LIDAR_ANGLE_KEY],
                                                    sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_NUM_LIDAR_VALUES_KEY]);

                auto desiredOverlayLidarDegrees = linspace(sensorConfiguration_[LIDAR_OVERLAY_KEY][LIDAR_OVERLAY_CONFIG_MIN_LIDAR_ANGLE_KEY],
                                                           sensorConfiguration_[LIDAR_OVERLAY_KEY][LIDAR_OVERLAY_CONFIG_MAX_LIDAR_ANGLE_KEY],
                                                           sensorConfiguration_[LIDAR_OVERLAY_KEY][LIDAR_OVERLAY_CONFIG_NUM_LIDAR_VALUES_KEY]);

                // Add the min/max lidarDegrees and ranges for interpolation in case not present.
                if(!std::count(lidarDegrees.begin(), lidarDegrees.end(), sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_MIN_LIDAR_ANGLE_KEY])) {
                    lidarDegrees.insert(lidarDegrees.begin(), sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_MIN_LIDAR_ANGLE_KEY]);
                    lidarDegrees.push_back(sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_MAX_LIDAR_ANGLE_KEY]);
                    lidarRanges.insert(lidarRanges.begin(), maxLiDARDist_);
                    lidarRanges.push_back(maxLiDARDist_);
                }
                std::lock_guard<std::mutex> guard(lidarMutex_);
                lidarData_ = interp(desiredLidarDegrees, lidarDegrees, lidarRanges);
                overlayLidarData_ = interp(desiredOverlayLidarDegrees, overlayDegrees, overlayRanges);
            }
            catch (const std::exception &ex) {
                RCLCPP_ERROR(this->get_logger(), "LiDAR callback failed: %s", ex.what());
            }
        }

        /// Function to sectorize the lidarData to represent sectors where an object is detected.
        std::vector<float> binarySectorizeLidarData(const std::vector<float>& lidarData, size_t blockSize, float maxDist){
            std::vector<float> sectorLidarData;
            for(size_t i = 0; i < lidarData.size(); i += blockSize) {
                bool setBit = false;
                for(size_t b = 0; b < blockSize; b++) {
                    setBit = setBit || (lidarData[i+b] < maxDist);
                }
                sectorLidarData.push_back((setBit) ? 1 : 0);
            }
            return sectorLidarData;
        }
 
        /// Callback function for the statusCheckService_ service to return the status of the
        /// sensors based on the presence of data in the data buffer at the sensor fusion node.
        void statusCheckCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                 std::shared_ptr<deepracer_interfaces_pkg::srv::SensorStatusCheckSrv::Request> req,
                                 std::shared_ptr<deepracer_interfaces_pkg::srv::SensorStatusCheckSrv::Response> res) {
            try {
                (void)request_header; 
                (void)req;

                res->single_camera_status = 0;
                res->stereo_camera_status = 0;
                res->lidar_status = 0;
                std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
                if(std::chrono::duration_cast<std::chrono::seconds>(now - lastLidarMsgRecievedTime).count() > 1) {
                    res->lidar_status = 1;
                }
                if(cameraImageCount_ != 2){
                    res->stereo_camera_status = 1;
                }
                if (cameraImageCount_ != 1) {
                    res->single_camera_status = 1;
                }
                if(std::chrono::duration_cast<std::chrono::seconds>(now - lastCameraMsgRecievedTime).count() > 1) {
                    res->stereo_camera_status = 1;
                    res->single_camera_status = 1;
                }
            }
            catch (const std::exception &ex) {
                RCLCPP_ERROR(this->get_logger(), "statusCheckCallback failed: %s", ex.what());
            }

         }

        /// Callback method for the lidarConfigService_ service. Allows clients to dynamically
        /// set the LiDAR configuration: min_angle, max_angle, min_distance,
        /// max_distance, num_values, clipping_distance, num_sectors, preprocess_type.
        void setLidarConfigCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                     std::shared_ptr<deepracer_interfaces_pkg::srv::LidarConfigSrv::Request> req,
                                     std::shared_ptr<deepracer_interfaces_pkg::srv::LidarConfigSrv::Response> res) {
            try {
                (void)request_header; 
                if(req->min_angle > req->max_angle ||
                   req->min_distance > req->max_distance ||
                   req->min_distance > req->clipping_distance ||
                   req->num_values < 1){
                    RCLCPP_ERROR(this->get_logger(), "Incorrect lidar configuration values.");
                    res->error = 1;
                    return;
                }
                // Check if the preprocessing is valid
                if(req->preprocess_type < 0 || req->preprocess_type >= numLidarPreprocessingType){
                    RCLCPP_ERROR(this->get_logger(), "Incorrect lidar preprocess type");
                    res->error = 1;
                    return;
                }
                sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_MIN_LIDAR_ANGLE_KEY] = req->min_angle;
                sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_MAX_LIDAR_ANGLE_KEY] = req->max_angle;
                sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_MIN_LIDAR_DIST_KEY] = req->min_distance;
                sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_MAX_LIDAR_DIST_KEY] = req->max_distance;
                sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_NUM_LIDAR_VALUES_KEY] = req->num_values;
                sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_LIDAR_CLIPPING_DIST_KEY] = req->clipping_distance;
                sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_NUM_LIDAR_SECTORS_KEY] = req->num_sectors;
                sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_PREPROCESS_TYPE_KEY] = req->preprocess_type;
                
                // Select the minimum of the clipping distance and max_lidar_distance as upper bound of the range
                maxLiDARDist_ = std::min(sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_MAX_LIDAR_DIST_KEY],
                                         sensorConfiguration_[LIDAR_KEY][LIDAR_CONFIG_LIDAR_CLIPPING_DIST_KEY]);
                RCLCPP_INFO(this->get_logger(), "Setting the lidar configuration values for the model");
                res->error = 0;
                return;
            }
            catch (const std::exception &ex) {
                RCLCPP_ERROR(this->get_logger(), "setLidarConfigCallback failed: %s", ex.what());
            }

         }
        
        /// Writes the given sensor configuration map to file
        /// @param sensorConfiguration Sensor configuration file to write to file
        /// @param filePath Full path to the desired file for which to write the given sensor configuration map.
        void writeSensorConfigJSON(const std::unordered_map<std::string, std::unordered_map<std::string, float>> &sensorConfiguration,
                                   const std::string &filePath) {
            try {
                Json::Value sensorConfigJsonValue;
                auto itLidarConfig = sensorConfiguration.find(LIDAR_KEY);
                auto itLidarOverlayConfig = sensorConfiguration.find(LIDAR_OVERLAY_KEY);

                if (itLidarConfig != sensorConfiguration.end() && itLidarOverlayConfig != sensorConfiguration.end()) {
                    for (auto lidarConfigMap : itLidarConfig->second) {
                        sensorConfigJsonValue[LIDAR_KEY][lidarConfigMap.first] = lidarConfigMap.second;
                    }
                    for (auto lidarOverlayConfigMap : itLidarOverlayConfig->second) {
                        sensorConfigJsonValue[LIDAR_OVERLAY_KEY][lidarOverlayConfigMap.first] = lidarOverlayConfigMap.second;
                    }
                }
                else {
                    RCLCPP_ERROR(this->get_logger(), "Invalid sensor configuration map");
                    return;
                }
                writeJSONToFile(sensorConfigJsonValue, filePath);
            }
            catch (const std::exception &ex) {
                RCLCPP_ERROR(this->get_logger(), "writeSensorConfigJSON failed: %s", ex.what());
            }

        }

        /// Reads the sensor configuration data from the desired file and stores the values in memory
        /// @param sensorConfiguration Map to store the sensor configuration read from file
        /// @param filePath Full file path of the file containing the sensor configuration details, note the client verifies existence
        void setSensorConfigurationFromFile(std::unordered_map<std::string, std::unordered_map<std::string, float>> &sensorConfiguration,
                                           const std::string &filePath) {
            try {
                Json::Value sensorConfigJsonValue;
                Json::Reader reader;
                std::ifstream ifs(filePath);
        
                if (!reader.parse(ifs, sensorConfigJsonValue)) {
                    RCLCPP_ERROR(this->get_logger(), "Error parsing sensor_configuration.json");
                    return;
                }
                if (!sensorConfigJsonValue.isMember(LIDAR_KEY) || !sensorConfigJsonValue.isMember(LIDAR_OVERLAY_KEY)) {
                    RCLCPP_ERROR(this->get_logger(), "Sensor Configuration file error: Missing servo type");
                    return;
                }

                auto populateMap = [&](auto &map, const auto &configType) {
                    for (auto &sensorConfigMap : map) {
                        if (sensorConfigJsonValue[configType].isMember(sensorConfigMap.first)) {
                            sensorConfigMap.second = sensorConfigJsonValue[configType][sensorConfigMap.first].asFloat();
                        }
                        else {
                            RCLCPP_ERROR(this->get_logger(), "Sensor Configuration file error:%s missing: %s", configType.c_str(), sensorConfigMap.first.c_str());
                            return false;
                        }
                    }
                    return true;
                };
        
                auto tmpMap = sensorConfiguration;
                auto itLidarConfig = tmpMap.find(LIDAR_KEY);
                auto itLidarOverlayConfig = tmpMap.find(LIDAR_OVERLAY_KEY);
        
                if (itLidarConfig != tmpMap.end() && itLidarOverlayConfig != tmpMap.end()) {
                    if (!populateMap(itLidarConfig->second, LIDAR_KEY) || !populateMap(itLidarOverlayConfig->second, LIDAR_OVERLAY_KEY)) {
                        return;
                    }
                }
                else {
                    RCLCPP_ERROR(this->get_logger(), "Invalid sensor configuration map");
                    return;
                }
                sensorConfiguration = tmpMap;
            }
            catch (const std::exception &ex) {
                RCLCPP_ERROR(this->get_logger(), "setSensorConfigurationFromFile failed: %s", ex.what());
            }

        }
        

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidarSub_;
        rclcpp::Subscription<deepracer_interfaces_pkg::msg::CameraMsg>::SharedPtr cameraSub_;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr displaySub_;

        rclcpp::Service<deepracer_interfaces_pkg::srv::LidarConfigSrv>::SharedPtr lidarConfigService_;
        rclcpp::Service<deepracer_interfaces_pkg::srv::SensorStatusCheckSrv>::SharedPtr statusCheckService_;

        rclcpp::Publisher<deepracer_interfaces_pkg::msg::EvoSensorMsg>::SharedPtr sensorMsgPub_;

        image_transport::ImageTransport image_transport_;       
        image_transport::Publisher overlayImagePub_;

        // Max LiDAR distance required to replace the incorrect/erroneous LiDAR data point
        // that is read as 'inf' in the LaserScan message.
        float maxLiDARDist_;
        std::vector<float> lidarData_;
        std::vector<float> overlayLidarData_;
        LidarOverlay lidarOverlayProcessingObj_;
        /// Hash map that stores the min/max of each servo.
        std::unordered_map<std::string, std::unordered_map<std::string, float>> sensorConfiguration_;
        /// Flag to enable publishing the overlay image.
        std::atomic<bool> enableOverlayPublish_;
        std::chrono::steady_clock::time_point lastLidarMsgRecievedTime;
        std::chrono::steady_clock::time_point lastCameraMsgRecievedTime;
        size_t cameraImageCount_;
        int imageWidth_;
        int imageHeight_;
        std::mutex lidarMutex_;

        rclcpp::Node::SharedPtr node_handle_;

    };
}


/// Node entry point
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<SensorFusion::SensorFusionNode>("sensor_fusion_node");
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
