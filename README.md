# DeepRacer Sensor Fusion Package

## Overview

The DeepRacer Sensor Fusion ROS package creates the *sensor_fusion_node* which is part of the core AWS DeepRacer application and will be launched from the deepracer_launcher. More details about the application and the components can be found [here](https://github.com/awsdeepracer/aws-deepracer-launcher).

This node is responsible for collecting the messages from all the sensors and publish the combined sensor message. It provides services and functions to subscribe to the LiDAR and camera messages, configure LiDAR, publish a combined sensor message, publish an overlay message with the LiDAR data overlaid on top of the camera image.

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The DeepRacer device comes with all the pre-requisite packages and libraries installed to run the sensor_fusion_pkg. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The sensor_fusion_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

1. *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application.
1. *cv_bridge* - This contains CvBridge, which converts between ROS Image messages and OpenCV images.
1. *image_transport* - It provides transparent support for transporting images in low-bandwidth compressed formats.
1. *sensor_msgs* - This package defines messages for commonly used sensors, including cameras and scanning laser rangefinders.

## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the sensor_fusion_pkg on the DeepRacer device:

        git clone https://github.com/awsdeepracer/aws-deepracer-sensor-fusion-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-sensor-fusion-pkg
        rosws update

1. Resolve the sensor_fusion_pkg dependencies:

        cd ~/deepracer_ws/aws-deepracer-sensor-fusion-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the sensor_fusion_pkg and deepracer_interfaces_pkg:

        cd ~/deepracer_ws/aws-deepracer-sensor-fusion-pkg && colcon build --packages-select sensor_fusion_pkg deepracer_interfaces_pkg

## Usage

The sensor_fusion_node provides the core functionality to combine the sensor data from various sensors connected to the DeepRacer vehicle. Although the nodes is built to work with the AWS DeepRacer application, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built sensor_fusion_node as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-sensor-fusion-pkg/install/setup.bash

1. Launch the sensor_fusion_node using the launch script:

        ros2 launch sensor_fusion_pkg sensor_fusion_pkg_launch.py

## Launch Files

The  sensor_fusion_pkg_launch.py is also included in this package that gives an example of how to launch the nodes independently from the core application.

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='sensor_fusion_pkg',
                namespace='sensor_fusion_pkg',
                executable='sensor_fusion_node',
                name='sensor_fusion_node'
            )
        ])

## Node Details

### sensor_fusion_node

#### Subscribed Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|/rplidar_ros/scan|LaserScan|This message holds the LiDAR data published by the rplidar_ros package.|
|/camera_pkg/video_mjpeg|CameraMsg|This message holds the single camera or two camera images from the connected cameras published by the camera_pkg.|
|/camera_pkg/display_mjpeg|Image|This message holds one camera image used for display in the UI published by the camera_pkg.|

#### Published Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|/sensor_fusion_pkg/overlay_msg|Image|Publisher to publish the overlay message with sector LiDAR information highlighting nearby obstacles overlayed over the camera image frame.|
|/sensor_fusion_pkg/sensor_msg|EvoSensorMsg|Publisher that publishes combined sensor messages with camera data and LiDAR data.|

#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|sensor_data_status|SensorStatusCheckSrv|Service that is called to find out the data status of the cameras and the LiDAR sensors. Based on whether the single camera/stereo camera/LiDAR data is being read by the node, the corresponding sensor status are set.|
|configure_lidar|LidarConfigSrv|Service called to dynamically configure LiDAR processing information for each model. It allows to set the LiDAR preprocessing information like the maximum distance, minimum distance, number of sectors etc., which will be used to create the sensor message being published for running the inference for the particular model selected.|

## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md)
