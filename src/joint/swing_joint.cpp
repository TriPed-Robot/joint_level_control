#include "joint_level_control/joint/swing_joint.h"


SwingJoint::SwingJoint(const std::string& joint_name, const std::string& can_name, uint8_t can_id)
    : position_(0.0), velocity_(0.0), effort_(0.0), command_position_(0.0), motor_(can_name, can_id)
{
    // Setup hardware interface:
    hardware_interface::JointStateHandle state_handle(joint_name, &position_, &velocity_, &effort_);
    joint_state_interface_.registerHandle(state_handle);
    registerInterface(&joint_state_interface_);

    hardware_interface::JointHandle effort_handle(joint_state_interface_.getHandle(joint_name), &command_position_);
    effort_joint_interface_.registerHandle(effort_handle);
    registerInterface(&effort_joint_interface_);
}


SwingJoint::~SwingJoint()
{
}

  
void SwingJoint::read()
{
    position_ = command_position_; // TODO: read sensor data. Assigning the last command positions leads to 0 error in control loop.
}


void SwingJoint::write()
{
    motor_.setCurrent(command_position_);
}
