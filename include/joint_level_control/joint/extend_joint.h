#ifndef EXTEND_JOINT_H
#define EXTEND_JOINT_H


#include <string>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "joint_level_control/motor/motor.h"
#include "joint_level_control/rotary_encoder/rotary_encoder.h"


class ExtendJoint : public hardware_interface::RobotHW
{
public:
    ExtendJoint(const std::string& joint_name, const std::string& can_name, uint8_t can_id);
    ~ExtendJoint();

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
    RotaryEncoder rotary_encoder_;
};


#endif
