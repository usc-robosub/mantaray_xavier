#!/usr/bin/env bash

source ~/.bashrc
source ~/catkin_ws/devel/setup.bash
export ROS_IP=192.168.23.101
export ROS_MASTER_URI=http://lab_computer:11311
#export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=mantaraylab1
export PYTHONPATH=/home/mantaray/catkin_ws/devel/lib/python2.7/dist-packages:$PYTHONPATH
