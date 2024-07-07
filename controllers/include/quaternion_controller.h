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


ros::Publisher quaternion_pub;

ros::Subscriber odometryFilteredListener;

tf::Quaternion robotQuat;
Eigen::Matrix<double, 3, 1> sp;

uint16_t quaternion_size = 1; //size of linear velocity messages queue
uint16_t thrusters_size = 1; //size of thruster messages queue
uint8_t* thruster_mapping;
double* thruster_values;

bool update = false;
double goal_x = 0;
double goal_y = 0;
double goal_z = 0;

Eigen::Matrix <double, 3, 1> getAngularSetpoint(Eigen::Quaternion<double> sp, Eigen::Quaternion<double> meas);

void odometryFilteredListenerCallback(nav_msgs::Odometry msg);

void thruster_mixer();
