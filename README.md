# AWS DeepRacer device info package

## Overview

The AWS DeepRacer device info ROS package creates the `device_info_node`, which is part of the core AWS DeepRacer application and launches from the `deepracer_launcher`. For more details about the application and the components, see the  [aws-deepracer-launcher repository](https://github.com/aws-deepracer/aws-deepracer-launcher).

This node is responsible for providing the hardware version of the AWS DeepRacer device and the software version of the `aws-deepracer-core` Debian package. It provides services and functions to detect the current hardware and software version and log the secure boot information. 

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the `device_info_pkg`. For more information about the preinstalled set of packages and libraries on the AWS DeepRacer, and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The `device_info_pkg` specifically depends on the following ROS 2 packages as build and run dependencies:

1. `deepracer_interfaces_pkg`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application.

## Downloading and building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the `device_info_pkg` on the AWS DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-device-info-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-device-info-pkg
        rosws update

1. Resolve the `device_info_pkg` dependencies:

        cd ~/deepracer_ws/aws-deepracer-device-info-pkg && apt-get update
        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `device_info_pkg` and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-device-info-pkg && colcon build --packages-select device_info_pkg deepracer_interfaces_pkg

## Usage

Although the `device_info_node` is built to work with the AWS DeepRacer application, you can run it independently for development, testing, and debugging purposes.

### Run the node

To launch the built `device_info_node` as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user:

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-device-info-pkg/install/setup.bash 

1. Launch the `device_info_node` using the launch script:

        ros2 launch device_info_pkg device_info_pkg_launch.py

## Launch files

The `device_info_pkg_launch.py` included in this package provides an example demonstrating how to launch the `deepracer_navigation_node`.

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


## Node details

### `device_info_node`

#### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|`get_device_info`|`GetDeviceInfoSrv`|A service that is called to get the AWS DeepRacer hardware and software packages version information.|

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)