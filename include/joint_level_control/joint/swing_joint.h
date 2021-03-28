#ifndef JOINT_CONNECTOR_LEG_H
#define JOINT_CONNECTOR_LEG_H


#include <string>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "joint_level_control/motor/motor.h"


class SwingJoint : public hardware_interface::RobotHW
{
public:
    SwingJoint(const std::string& joint_name, const std::string& can_name, uint8_t can_id);
    ~SwingJoint();

    void read();
    void write();
    
private:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;

    double position_;   
    double velocity_;
    double effort_;
    double command_position_;
    
    Motor motor_;
};


#endif
