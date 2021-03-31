# DeepRacer Device Info Package

## Overview

The DeepRacer Device Info ROS package creates the *device_info_node* which is part of the core AWS DeepRacer application and will be launched from the deepracer_launcher. More details about the application and the components can be found [here](https://github.com/aws-deepracer/aws-deepracer-launcher).

This node is responsible for providing the hardware version of the DeepRacer device and software version of *aws-deepracer-core* debian package. It provides services and functions to find out the current hardware and software version and log the secure boot information. 

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

The DeepRacer device comes with all the pre-requisite packages and libraries installed to run the device_info_pkg. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The device_info_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

1. *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application.

## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the device_info_pkg on the DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-device-info-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-device-info-pkg
        rosws update

1. Resolve the device_info_pkg dependencies:

        cd ~/deepracer_ws/aws-deepracer-device-info-pkg && apt-get update
        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the device_info_pkg and deepracer_interfaces_pkg:

        cd ~/deepracer_ws/aws-deepracer-device-info-pkg && colcon build --packages-select device_info_pkg deepracer_interfaces_pkg

## Usage

Although the *device_info_node* is built to work with the AWS DeepRacer application, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built device_info_node as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-device-info-pkg/install/setup.bash 

1. Launch the device_info_node using the launch script:

        ros2 launch device_info_pkg device_info_pkg_launch.py

## Launch Files

The  device_info_pkg_launch.py is also included in this package that gives an example of how to launch the deepracer_navigation_node.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='device_info_pkg',
                    namespace='device_info_pkg',
                    executable='device_info_node',
                    name='device_info_node'
                )
            ])


## Node Details

### device_info_node

#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|get_device_info|GetDeviceInfoSrv|A service that is called to get the DeepRacer hardware and software packages version information.|

## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)