#include "joint_level_control/joint/extend_joint.h"


ExtendJoint::ExtendJoint(const std::string& joint_name, const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint8_t spi_speed, uint8_t spi_delay, const std::string& can_name, uint8_t can_id)
    : position_(0.0), velocity_(0.0), effort_(0.0), command_position_(0.0), motor_(can_name, can_id), rotary_encoder_(spi_device, spi_cs_id, spi_mode, spi_bits, spi_speed, spi_delay)
{
    // Setup hardware interface:
    hardware_interface::JointStateHandle state_handle(joint_name, &position_, &velocity_, &effort_);
    joint_state_interface_.registerHandle(state_handle);
    registerInterface(&joint_state_interface_);

    hardware_interface::JointHandle effort_handle(joint_state_interface_.getHandle(joint_name), &command_position_);
    effort_joint_interface_.registerHandle(effort_handle);
    registerInterface(&effort_joint_interface_);
}


ExtendJoint::~ExtendJoint()
{
}

  
void ExtendJoint::read()
{
    double position = rotary_encoder_.getValue();
    position_ = command_position_; // TODO: read sensor data. Assigning the last command positions leads to 0 error in control loop.
}


void ExtendJoint::write()
{
    motor_.setCurrent(command_position_);
}
