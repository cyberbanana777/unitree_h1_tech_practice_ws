#!/bin/bash

sudo apt install \
    python3-pip \
    ros-$ROS_DISTRO-compressed-image-transport \
    ros-$ROS_DISTRO-rviz-common

pip install \
    numpy==1.26.4 \
    opencv-contrib-python==4.9.0.80