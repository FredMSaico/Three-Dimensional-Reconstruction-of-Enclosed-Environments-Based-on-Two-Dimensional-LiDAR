# 3D Mapping ROS Package

This repository contains a ROS package for 3D mapping using a 2D LIDAR, specifically the RPLIDAR A1M8 model. The package has been tested on ROS Melodic and Noetic versions and requires any ROS-based robot that publishes odometry data. The system subscribes to the `/scan` (sensor_msgs/LaserScan) and `/odom` (nav_msgs/Odometry) topics to receive the necessary data for map construction.

## Requirements

- ROS Melodic or Noetic
- RPLIDAR A1M8 /to install: https://github.com/Slamtec/rplidar_ros
- A ROS-based robot that publishes odometry data on the `/odom` topic

## Installation

1. Clone this repository into your ROS workspace:
    ```sh
    cd ~/catkin_ws/src
    git clone <REPOSITORY_URL>
    ```

2. Compile the package:
    ```sh
    cd ~/catkin_ws
    catkin_make
    ```

3. Source the environment:
    ```sh
    source devel/setup.bash
    ```

## Usage

### Step 1: Launch the LIDAR Node

To launch the LIDAR node, run the following command:
```sh
roslaunch rplidar_ros rplidar.launch
```
### Step 2: Launch the Odometry Node

This step depends on the robot you are using. Make sure you have the node that publishes odometry data on /odom running.

### Step 3: Launch the Mapping Package

To launch the 3D mapping package, run:

```sh
roslaunch mapping3d mapping_demo.launch
```sh
