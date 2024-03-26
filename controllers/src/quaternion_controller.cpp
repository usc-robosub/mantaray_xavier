/*
Attitude (Quaternion) Controller Node
Controls AUV attitude/orientation (roll, pitch, yaw)
*/
#include "quaternion_controller.h"

ros::Publisher linear_vel_pub;

tf::Quaternion robotQuat;

uint16_t linear_vel_size = 1; //size of linear velocity messages queue
uint16_t thrusters_size = 1; //size of thruster messages queue
uint8_t* thruster_mapping;
double* thruster_values;

bool update = false;
double x_vel = 0;
double y_vel = 0;
double z_vel = 0;

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

void run(int dt){
    Eigen::Quaternion<double> meas(robotQuat.getW(), robotQuat.getX(), robotQuat.getY(), robotQuat.getZ());
    ROS_INFO("Measured Quaternion: (W=%f, X=%f, Y=%f, Z=%f)", meas.w(), meas.x(), meas.y(), meas.z());

    Eigen::Quaternion<double> qsp;
    
    qsp = Eigen::AngleAxisd(this->setpoint(3,0), Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(this->setpoint(4,0), Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(this->setpoint(5,0), Eigen::Vector3d::UnitZ());
    
    ROS_INFO("Desired Setpoint Quaternion: (W=%f, X=%f, Y=%f, Z=%f)", qsp.w(), qsp.x(), qsp.y(), qsp.z());


    Eigen::Matrix<double, 3, 1> sp = getAngularSetpoint(qsp, meas);
    
    thruster_values[0] = sp(2,0) * 100;
    thruster_values[1] = -sp(2,0) * 100;
    thruster_values[2] = sp(2,0) * 100;
    thruster_values[3] = -sp(2,0) * 100;
    thruster_values[4] = sp(0,0) * 100 - sp(1,0) * 100;
    thruster_values[5] = sp(0,0) * 100 - sp(1,0) * 100;
    thruster_values[6] = sp(0,0) * 100 + sp(1,0) * 100;
    thruster_values[7] = sp(0,0) * 100 + sp(1,0) * 100;
    ROS_INFO("Robot State: %f %f %f", sp(0,0), sp(1,0), sp(2,0));  
    ROS_INFO("_____________");
 }

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

    ros::Subscriber linear_vel_x = linear_vel_node.subscribe("linear_vel/x", linear_vel_size, lin_vel_x_callback);
    ros::Subscriber linear_vel_y = linear_vel_node.subscribe("linear_vel/y", linear_vel_size, lin_vel_y_callback);
    ros::Subscriber linear_vel_z = linear_vel_node.subscribe("linear_vel/z", linear_vel_size, lin_vel_z_callback);
    
    linear_vel_pub = linear_vel_node.advertise<thruster_msgs::ThrustOutput>("thrusters/lv_control_input", thrusters_size);

    ros::Rate loop_rate(10); //todo fix magic number

    // Initialize thruster mapping values and thruster values
    thruster_mapping = new uint8_t[8];
    thruster_values = new double[8];
    for (uint8_t i = 0; i<8; i++) {
        // ros::param::param<uint8_t>("thruster_"+('0'+i), thruster_mapping[i], i);
        thruster_mapping[i] = i;
        thruster_values[i] = 0;
    }


    while(ros::ok()) {
        //TODO:SOMONE if no new inputs sent after 5s, turn off motors

        if (update) {
            thruster_mixer();
            for (int i = 0; i<8; i++) {
                thruster_msgs::ThrustOutput msg;
                msg.num = thruster_mapping[i];
                msg.val = thruster_values[i];
                linear_vel_pub.publish(msg);
                if (i == 7)
                    ROS_DEBUG("Published to %i thrust val %f", msg.num, msg.val);
            }
            update = false;
        }
        ros::spinOnce();
    }
    delete [] thruster_mapping;
    delete [] thruster_values;
    return 0;
}
