#include "include.h"
#include "base_controller.h"

std::string Base_Controller::getName(){
    return this->name_;
}

double* Base_Controller::getThrusterValues(){
    return this->thruster_values;
}