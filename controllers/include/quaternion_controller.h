#pragma once

#include "ros/ros.h"
#include <ros/console.h>
#include "thruster_msgs/ThrustOutput.h"
#include "std_msgs/Float64.h"
#include <cmath>
#include "eigen3/Eigen/Dense"

Eigen::Matrix <double, 3, 1> getAngularSetpoint(Eigen::Quaternion<double> sp, Eigen::Quaternion<double> meas);
