# mantaray_xavier
ROS package that handles control on the Jetson Xavier.

## Installation

```
cd ~/catkin_ws/src
git clone git@github.com:USCAUVTeam/mantaray_xavier.git
cd ..
catkin_make
```

## Launching

```
roslaunch mantaray_xavier mantaray.launch
```

## Dependencies:
* [zed-api](https://www.stereolabs.com/docs/app-development/cpp/linux/) - An API for the Zed-M Stereo Camera
* [darknet_ros](https://github.com/leggedrobotics/darknet_ros) - ROS Package that contains implementaion for YOLO v4