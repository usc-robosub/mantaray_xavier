/*
Linear Velocity Controller Node
Controls AUV movement in x, y, z direction
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

uint16_t linear_vel_size = 1; //size of linear velocity messages queue
uint16_t thrusters_size = 1; //size of thruster messages queue

void linear_vel_x_callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void linear_vel_y_callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void linear_vel_z_callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "linear_velocity_controller");

    ros::NodeHandle linear_vel_node;

    ros::Subscriber linear_vel_x = linear_vel_node.subscribe("/linear_vel/x", linear_vel_size, lin_vel_x_callback);
    ros::Subscriber linear_vel_y = linear_vel_node.subscribe("/linear_vel/y", linear_vel_size, lin_vel_y_callback);
    ros::Subscriber linear_vel_z = linear_vel_node.subscribe("/linear_vel/z", linear_vel_size, lin_vel_z_callback);

    ros::spin(); // ???

    //TODO do stuff with the data acquired
    //allocate the thrusters... the thrust
    // all normalized to -1 to 1
    // thrusters at 30 degree angles
    
    ros::Publisher linear_vel_pub = linear_vel_node.advertise<std_msgs::Float64>("/thrusters/lv_control_input", thrusters_size);

    ros::Rate loop_rate(10); //todo fix magic number

    while(ros::ok()) {
        std_msgs::Float64 linear_vel_msg;

        // acquire data from linear vel numbers

        linear_vel_pub.publish(linear_vel_msg);

        //if no new inputs sent after 5s, turn off motors

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
