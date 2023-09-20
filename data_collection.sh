#!/bin/bash

# Source ROS setup script
source /opt/ros/melodic/setup.bash  # This is typical for ROS Melodic, adjust for your ROS version if necessary.

# Source your workspace's setup script if you have one. Assuming your workspace is named "/Desktop/UAV_data_collection":
source ~/Desktop/UAV_data_collection/devel/setup.bash

# Launch your ROS node
roslaunch datacollection camera_and_vio_data.launch
