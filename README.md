# mantaray_xavier
ROS package for the mantaray sub on the Jetson Xavier.

### Please use the master branch. The repository is intended to be a single ROS Package that puts together many packages and connects to the RPI roscore. The dependencies of this project should be cloned as packages into the catkin workspace or installed onto the machine path.

## Installation

```
cd ~/catkin_ws/src
git clone git@github.com:USCAUVTeam/mantaray_xavier.git
git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git
git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
cd ../
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Launching

```
source ./devel/setup.bash
roslaunch mantaray_xavier mantaray.launch
```

## Dependencies:
* [zed-api](https://www.stereolabs.com/docs/app-development/cpp/linux/) - An API for the Zed-M Stereo Camera
* [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper) - ROS wrapper for the Zed-M Stereo Camera
* [darknet_ros](https://github.com/leggedrobotics/darknet_ros) - ROS Package that contains implementaion for YOLO v4