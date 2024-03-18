#!/usr/bin/env bash
# TODO: Execute the command by doing . ./build.sh
cd ~/catkin_ws

catkin build -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3

sudo chmod 666 /dev/ttyACM0
source devel/setup.bash
cd -

