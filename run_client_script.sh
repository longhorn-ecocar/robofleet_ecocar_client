#!/bin/sh

source /opt/ros/noetic/setup.bash

export ROS_PACKAGE_PATH=/home/autera-admin/robofleet_ecocar/robofleet_client:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/autera-admin/infrastructure/ros1_utils/amrl_msgs:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/autera-admin/datachecker:$ROS_PACKAGE_PATH

export ROS_IP=192.168.131.1
export ROS_HOSTNAME=192.168.131.1
export ROS_MASTER_URI=http://$ROS_IP:11311

export ROBOFLEET_SERVER_PORT=8084
export REACT_APP_ROBOFLEET_SERVER_PORT=$ROBOFLEET_SERVER_PORT

cd /home/autera-admin/robofleet_ecocar/robofleet_client
ROS_NAMESPACE="leva" make run

