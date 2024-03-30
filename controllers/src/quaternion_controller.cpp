/*
Attitude (Quaternion) Controller Node
Controls AUV attitude/orientation (roll, pitch, yaw)

Need to connect robotQuat to RobotLocalization
*/
#include "quaternion_controller.h"

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

Eigen::Matrix <double, 3, 1> getAngularSetpoint(Eigen::Quaternion<double> sp, Eigen::Quaternion<double> meas) {
  
    Eigen::Quaternion<double> err = sp * meas.conjugate(); // error around each axis of rotation
    Eigen::Quaternion<double> errShort;

    if (err.w() < 0) {
        Eigen::Quaternion<double> flipped(-err.w(), -err.x(), -err.y(), -err.z());
        errShort = flipped;
    } else {
        errShort = err;
    }

    ROS_INFO("Error: (%f, %f, %f)", errShort.x(), errShort.y(), errShort.z());
    Eigen::Matrix <double, 3, 1> angularSetpoint;
    angularSetpoint << errShort.x(), errShort.y(), errShort.z();
    angularSetpoint *= 1;
    return angularSetpoint;
} 

void run(const ros::TimerEvent&){
    Eigen::Quaternion<double> meas(robotQuat.getW(), robotQuat.getX(), robotQuat.getY(), robotQuat.getZ());
    ROS_INFO("Measured Quaternion: (W=%f, X=%f, Y=%f, Z=%f)", meas.w(), meas.x(), meas.y(), meas.z());

    Eigen::Quaternion<double> qsp;
    
    qsp = Eigen::AngleAxisd(goal_x, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(goal_y, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(goal_z, Eigen::Vector3d::UnitZ());
    
    ROS_INFO("Desired Setpoint Quaternion: (W=%f, X=%f, Y=%f, Z=%f)", qsp.w(), qsp.x(), qsp.y(), qsp.z());
    ROS_INFO("Desired Setpoint Degrees: (X=%f, Y=%f, Z=%f)", goal_x, goal_y, qsp.z());
    
    sp = getAngularSetpoint(qsp, meas);
    
    // Debugging code start
    // Convert the quaternion to Euler angles
    double roll, pitch, yaw;
    tf::Matrix3x3(robotQuat).getRPY(roll, pitch, yaw);

    // Convert radians to degrees
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;

    // Print out the Euler angles in degrees
    ROS_INFO("Roll: %.2f degrees, Pitch: %.2f degrees, Yaw: %.2f degrees", roll, pitch, yaw);
    // Debugging code end
    
    ROS_INFO("_____________");

    thruster_mixer();
 }

void odometryFilteredListenerCallback(nav_msgs::Odometry msg) {
    robotQuat[0] = msg.pose.pose.orientation.x;
    robotQuat[1] = msg.pose.pose.orientation.y;
    robotQuat[2] = msg.pose.pose.orientation.z;
    robotQuat[3] = msg.pose.pose.orientation.w;

    // Eventually create a cascading PID loop using the angular velocity of the vehicle
    // robotState[9] = msg.twist.twist.angular.x;
    // robotState[10] = msg.twist.twist.angular.y;
    // robotState[11] = msg.twist.twist.angular.z;
    // ROS_INFO("Angular Velocities: X=%f, Y=%f, Z=%f", robotState[9], robotState[10], robotState[11]);
}

void quat_x_callback(const std_msgs::Float64::ConstPtr& data) {
    goal_x = data->data;
}

void quat_y_callback(const std_msgs::Float64::ConstPtr& data) {
    goal_y = data->data;
}

void quat_z_callback(const std_msgs::Float64::ConstPtr& data) {
    goal_z = data->data;
}

void thruster_mixer() {
    // int max = fmax(1, fabs(sp(0,0)) + fabs(sp(1,0)));
    int max = 1;
    // Normalize
    // Max is always between 1 and 2. z_vel doesn't contribute to it

    // thruster_values[0] = sp(2,0) * max;
    // thruster_values[1] = -sp(2,0) * max;
    // thruster_values[2] = sp(2,0) * max;
    // thruster_values[3] = -sp(2,0) * max;

    // thruster_values[0] = -joy.LeftJoystickY * 25 - joy.LeftJoystickX * 25
    // thruster_values[1] = -joy.LeftJoystickY * 25 + joy.LeftJoystickX * 25
    // thruster_values[2] = joy.LeftJoystickY * 25 + joy.LeftJoystickX * 25
    // thruster_values[3] = joy.LeftJoystickY * 25 - joy.LeftJoystickX * 25
    thruster_values[4] = -sp(0,0) - sp(1,0);
    thruster_values[5] = -sp(0,0) + sp(1,0);
    thruster_values[6] = sp(0,0) - sp(1,0);
    thruster_values[7] = sp(0,0) + sp(1,0);

    // thruster_values[4] = -sp(1,0);
    // thruster_values[5] = sp(1,0);
    // thruster_values[6] = -sp(1,0);
    // thruster_values[7] = sp(1,0);

    // thruster_values[4] = 0;
    // thruster_values[5] = 0;
    // thruster_values[6] = 0;
    // thruster_values[7] = 0;

    // thruster_values[4] = (-sp(0,0) - sp(1,0))*max;
    // thruster_values[5] = (-sp(0,0) - sp(1,0))*max;
    // thruster_values[6] = (sp(0,0) + sp(1,0))*max;
    // thruster_values[7] = (sp(0,0) + sp(1,0))*max;

    for (int i = 0; i<8; i++) {
        thruster_msgs::ThrustOutput msg;
        msg.num = thruster_mapping[i];
        msg.val = thruster_values[i];
        quaternion_pub.publish(msg);
        // if (i == 7)
        //     ROS_DEBUG("Published to %i thrust val %f", msg.num, msg.val);
    }
}

int main (int argc, char **argv) {

    ros::init(argc, argv, "quaternion_controller");
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle quaternion_node;

    ros::Subscriber quaternion_x = quaternion_node.subscribe("orientation/x", 1, quat_x_callback);
    ros::Subscriber quaternion_y = quaternion_node.subscribe("orientation/y", 1, quat_y_callback);
    ros::Subscriber quaternion_z = quaternion_node.subscribe("orientation/z", 1, quat_z_callback);
    
    quaternion_pub = quaternion_node.advertise<thruster_msgs::ThrustOutput>("thrusters/q_control_input", thrusters_size);

    ros::Rate loop_rate(10); //todo fix magic number

    // Initialize thruster mapping values and thruster values
    thruster_mapping = new uint8_t[8];
    thruster_values = new double[8];
    for (uint8_t i = 0; i<8; i++) {
        // ros::param::param<uint8_t>("thruster_"+('0'+i), thruster_mapping[i], i);
        thruster_mapping[i] = i;
        thruster_values[i] = 0;
    }

    odometryFilteredListener = quaternion_node.subscribe<nav_msgs::Odometry>("odometry/filtered", 1, odometryFilteredListenerCallback);
    ros::Timer timer = quaternion_node.createTimer(ros::Duration(0.1), run);
    ros::spin();
    delete [] thruster_mapping;
    delete [] thruster_values;
    return 0;
}
