/*
Attitude (Quaternion) Controller Node
Controls AUV attitude/orientation (roll, pitch, yaw)
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

uint16_t orientation_size = 1; //size of orientation messages queue
uint16_t thrusters_size = 1; //size of thruster message queue

/*
pitch = x
yaw = y
roll = z
change these if u need idk what your orientaiton x y z is
*/
void pitch_callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void yaw_callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void roll_callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "attitude_controller");

    ros::NodeHandle attitude_node;

    ros::Subscriber pitch = attitude_node.subscribe("/orientation/x", orientation_size, pitch_callback);
    ros::Subscriber yaw = attitude_node.subscribe("/orientation/y", orientation_size, yaw_callback);
    ros::Subscriber roll = attitude_node.subscribe("/orientation/z", orientation_size, roll_callback);

    ros::spin(); // ???
    
    //TODO do stuff with the data acquired
    
    ros::Publisher attitude_pub = attitude_node.advertise<std_msgs::Float64>("/thrusters/q_control_input", thrusters_size);

    ros::Rate loop_rate(10); //todo fix magic number

    while(ros::ok()) {
        std_msgs::Float64 orientation_msg;

        // acquire data to publish

        attitude_pub.publish(orientation_msg);

        //if no new inputs sent after 5s, turn off motors

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
