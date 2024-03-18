#include "controllers/quaternion_controller.h"

extern double robotState[15];
extern tf::Quaternion robotQuat;

QP_Controller::QP_Controller(){
    this->thruster_values = new double[8];
}

QP_Controller::~QP_Controller(){
    delete [] thruster_values;
}

void QP_Controller::enter(){
    ROS_INFO("Entering QPController");
}

void QP_Controller::exit(){
    ROS_INFO("Exiting QPController");
}

Eigen::Matrix <double, 3, 1> QP_Controller::getAngularSetpoint(Eigen::Quaternion<double> sp, Eigen::Quaternion<double> meas) {
  
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

void QP_Controller::run(int dt){
    Eigen::Quaternion<double> meas(robotQuat.getW(), robotQuat.getX(), robotQuat.getY(), robotQuat.getZ());
    ROS_INFO("Measured Quaternion: (W=%f, X=%f, Y=%f, Z=%f)", meas.w(), meas.x(), meas.y(), meas.z());

    Eigen::Quaternion<double> qsp;
    
    qsp = Eigen::AngleAxisd(this->setpoint(3,0), Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(this->setpoint(4,0), Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(this->setpoint(5,0), Eigen::Vector3d::UnitZ());
    
    ROS_INFO("Desired Setpoint Quaternion: (W=%f, X=%f, Y=%f, Z=%f)", qsp.w(), qsp.x(), qsp.y(), qsp.z());

    Eigen::Matrix<double, 3, 1> sp = this->getAngularSetpoint(qsp, meas);
    
    this->thruster_values[0] = sp(2,0) * 100;
    this->thruster_values[1] = -sp(2,0) * 100;
    this->thruster_values[2] = sp(2,0) * 100;
    this->thruster_values[3] = -sp(2,0) * 100;
    this->thruster_values[4] = sp(0,0) * 100 - sp(1,0) * 100;
    this->thruster_values[5] = sp(0,0) * 100 - sp(1,0) * 100;
    this->thruster_values[6] = sp(0,0) * 100 + sp(1,0) * 100;
    this->thruster_values[7] = sp(0,0) * 100 + sp(1,0) * 100;
    ROS_INFO("Robot State: %f %f %f", sp(0,0), sp(1,0), sp(2,0));  
    ROS_INFO("_____________");
 }