#pragma once

#include "tf/tf.h"
#include "ros/ros.h"
#include <ros/console.h>
#include "thruster_msgs/ThrustOutput.h"
#include "std_msgs/Float64.h"
#include <cmath>
#include <thread>
#include "eigen3/Eigen/Dense"
#include <Eigen/Geometry>
#include "nav_msgs/Odometry.h"
#include <cstdint>


Eigen::Matrix <double, 3, 1> getAngularSetpoint(Eigen::Quaternion<double> sp, Eigen::Quaternion<double> meas);

void odometryFilteredListenerCallback(nav_msgs::Odometry msg);

void thruster_mixer();
