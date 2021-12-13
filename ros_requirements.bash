#!/bin/bash

sudo apt install ros-$ROS_DISTRO-turtlebot3
sudo apt install ros-$ROS_DISTRO-turtlebot3-bringup
sudo apt install ros-$ROS_DISTRO-turtlebot3-description
sudo apt install ros-$ROS_DISTRO-turtlebot3-gazebo
sudo apt install ros-$ROS_DISTRO-turtlebot3-msgs
sudo apt install ros-$ROS_DISTRO-turtlebot3-navigation
sudo apt install ros-$ROS_DISTRO-turtlebot3-simulations
sudo apt install ros-$ROS_DISTRO-turtlebot3-slam
sudo apt install ros-$ROS_DISTRO-move-base
sudo apt install ros-$ROS_DISTRO-move-base-msgs
sudo apt install ros-$ROS_DISTRO-tf
sudo apt install ros-$ROS_DISTRO-actionlib
sudo apt install ros-$ROS_DISTRO-actionlib-msgs
sudo apt install ros-$ROS_DISTRO-amcl
sudo apt install ros-$ROS_DISTRO-map-server
sudo apt install ros-$ROS_DISTRO-navigation

rospack profile
