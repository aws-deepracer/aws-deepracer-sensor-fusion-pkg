# AWS DeepRacer sensor fusion package

## Overview

The AWS DeepRacer sensor fusion ROS package creates the `sensor_fusion_node`, which is part of the core AWS DeepRacer application and launches from the `deepracer_launcher`. For more information about the application and the components, see the [aws-deepracer-launcher repository](https://github.com/aws-deepracer/aws-deepracer-launcher).

This node is responsible for collecting the messages from all the sensors and publishing the combined sensor message. It provides services and functions to subscribe to the LiDAR and camera messages, configure LiDAR, publish a combined sensor message, and publish an overlay message with the LiDAR data overlaid on top of the camera image.

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation
Follow these steps to install the AWS DeepRacer sensor fusion package.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the `sensor_fusion_pkg`. For more information about the preinstalled set of packages and libraries on the AWS DeepRacer, and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The `sensor_fusion_pkg` specifically depends on the following ROS 2 packages as build and run dependencies:

1. `deepracer_interfaces_pkg`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application.
1. `cv_bridge`: This package contains CvBridge, which converts between ROS image messages and OpenCV images.
1. `image_transport`: This package provides transparent support for transporting images in low-bandwidth compressed formats.
1. `sensor_msgs`: This package defines messages for commonly used sensors, including cameras and scanning laser rangefinders.

## Downloading and building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the `sensor_fusion_pkg` on the AWS DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-sensor-fusion-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-sensor-fusion-pkg
        rosws update

1. Resolve the `sensor_fusion_pkg` dependencies:

        cd ~/deepracer_ws/aws-deepracer-sensor-fusion-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `sensor_fusion_pkg` and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-sensor-fusion-pkg && colcon build --packages-select sensor_fusion_pkg deepracer_interfaces_pkg

## Usage

The `sensor_fusion_node` provides the core functionality to combine the sensor data from various sensors connected to the AWS DeepRacer vehicle. Although the node is built to work with the AWS DeepRacer application, you can run it independently for development, testing, and debugging purposes.

### Run the node

To launch the built `sensor_fusion_node` as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user:

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-sensor-fusion-pkg/install/setup.bash

1. Launch the `sensor_fusion_node` using the launch script:

        ros2 launch sensor_fusion_pkg sensor_fusion_launch.py

## Launch files

The `sensor_fusion_launch.py`, included in this package, provides an example demonstrating how to launch the nodes independently from the core application.

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

## Node details

### `sensor_fusion_node`

#### Subscribed topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|/`rplidar_ros`/`scan`|`LaserScan`|This message holds the LiDAR data published by the `rplidar_ros` package.|
|/`camera_pkg`/`video_mjpeg`|`CameraMsg`|This message holds the single-camera or two-camera images from the connected cameras published by the `camera_pkg`.|
|/`camera_pkg`/`display_mjpeg`|Image|This message holds the one-camera image used for display in the UI published by the `camera_pkg`.|

#### Published topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|/`sensor_fusion_pkg`/`overlay_msg`|Image|Publisher that publishes the overlay message with sector LiDAR information highlighting nearby obstacles overlayed over the camera image frame.|
|/`sensor_fusion_pkg`/`sensor_msg`|`EvoSensorMsg`|Publisher that publishes combined sensor messages with camera data and LiDAR data.|

#### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|`sensor_data_status`|`SensorStatusCheckSrv`|Service that is called to detect the data status of the cameras and the LiDAR sensors. Based on whether the single camera, stereo camera, or LiDAR data is being read by the node, the corresponding sensor status are set.|
|`configure_lidar`|`LidarConfigSrv`|Service called to dynamically configure LiDAR processing information for each model. It sets the LiDAR preprocessing information like the maximum distance, minimum distance, and number of sectors, which create the sensor message being published for running the inference for the particular model selected.|

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
