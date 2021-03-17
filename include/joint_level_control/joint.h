#ifndef JOINT_CONNECTOR_LEG_H
#define JOINT_CONNECTOR_LEG_H


#include <string>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "joint_level_control/second_order_system.h"


class Joint : public hardware_interface::RobotHW
{
public:
    Joint(const std::string& joint_name);
    ~Joint();
    
    void set_disturbance(double disturbance);
    void read();
    void write();
    
private:
    SecondOrderSystem _second_order_system;
    double _disturbance;

    hardware_interface::JointStateInterface _joint_state_interface;
    hardware_interface::EffortJointInterface _effort_joint_interface;

    double _joint_position;   
    double _joint_velocity;
    double _joint_effort;
    double _joint_command_position;
};


#endif
