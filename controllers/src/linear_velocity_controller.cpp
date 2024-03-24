/*
Linear Velocity Controller Node
Controls AUV movement in x, y, z direction
*/

#include "ros/ros.h"
#include <ros/console.h>
#include "thruster_msgs/ThrustOutput.h"
#include "std_msgs/Float64.h"
#include <cmath>

ros::Publisher linear_vel_pub;

uint16_t linear_vel_size = 1; //size of linear velocity messages queue
uint16_t thrusters_size = 1; //size of thruster messages queue
uint8_t* thruster_mapping;
double* thruster_values;

bool update = false;
int x_vel = 0;
int y_vel = 0;
int z_vel = 0;

void lin_vel_x_callback(const std_msgs::Float64::ConstPtr& data) {
    update = true;
    x_vel = data->data;
}

void lin_vel_y_callback(const std_msgs::Float64::ConstPtr& data) {
    update = true;
    y_vel = data->data;
}

void lin_vel_z_callback(const std_msgs::Float64::ConstPtr& data) {
    update = true;
    z_vel = data->data;
}

void thruster_mixer() {
    int max = fmax(1, fabs(x_vel) + fabs(y_vel));
    // Normalize
    // Max is always between 1 and 2. z_vel doesn't contribute to it
    thruster_values[0] = (x_vel - y_vel)/max;
    thruster_values[1] = (x_vel + y_vel)/max;
    thruster_values[2] = (-x_vel + y_vel)/max;
    thruster_values[3] = (-x_vel - y_vel)/max;
    thruster_values[4] = z_vel;
    thruster_values[5] = z_vel;
    thruster_values[6] = z_vel;
    thruster_values[7] = z_vel;
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "linear_velocity_controller");
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle linear_vel_node;

    ros::Subscriber linear_vel_x = linear_vel_node.subscribe("/linear_vel/x", linear_vel_size, lin_vel_x_callback);
    ros::Subscriber linear_vel_y = linear_vel_node.subscribe("/linear_vel/y", linear_vel_size, lin_vel_y_callback);
    ros::Subscriber linear_vel_z = linear_vel_node.subscribe("/linear_vel/z", linear_vel_size, lin_vel_z_callback);
    
    linear_vel_pub = linear_vel_node.advertise<thruster_msgs::ThrustOutput>("/thrusters/lv_control_input", thrusters_size);

    ros::Rate loop_rate(10); //todo fix magic number

    // Initialize thruster mapping values and thruster values
    thruster_mapping = new uint8_t[8];
    thruster_values = new double[8];
    for (uint8_t i = 0; i<8; i++) {
        // ros::param::param<uint8_t>("thruster_"+('0'+i), thruster_mapping[i], i);
        thruster_mapping[i] = i;
        thruster_values[i] = 0;
    }

    thruster_msgs::ThrustOutput msg;

    while(ros::ok()) {
        //TODO:SOMONE if no new inputs sent after 5s, turn off motors

        if (update) {
            thruster_mixer();
            for (int i = 0; i<8; i++) {
                msg.num = thruster_mapping[i];
                msg.val = thruster_values[i];
                linear_vel_pub.publish(msg);
            }
            ROS_DEBUG("Published to %i thrust val %f", msg.num, msg.val);
            update = false;
        }
        ros::spinOnce();
    }

    return 0;
}
