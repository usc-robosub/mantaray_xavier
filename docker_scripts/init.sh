#!/usr/bin/env bash
chmod o+rw /dev/i2c-0
chmod o+rw /dev/i2c-1
source ~/.bashrc
source ~/catkin_ws/devel/setup.bash
export ROS_IP=192.168.23.105
export ROS_MASTER_URI=http://lab_computer:11311
#export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=mantaray
#export ROS_HOSTNAME=localhost
export PYTHONPATH=/home/mantaray/catkin_ws/devel/lib/python2.7/dist-packages:$PYTHONPATH
