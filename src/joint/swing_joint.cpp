#include "joint_level_control/joint/swing_joint.h"


SwingJoint::SwingJoint(const std::string& joint_name, const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay, const std::string& can_name, uint8_t can_id, double zero_point)
    : position_(0.0), velocity_(0.0), effort_(0.0), command_position_(0.0), motor_(can_name, can_id), hall_sensor_(spi_device, spi_cs_id, spi_mode, spi_bits, spi_speed, spi_delay,zero_point)
{
    // Setup hardware interface:
    hardware_interface::JointStateHandle state_handle(joint_name, &position_, &velocity_, &effort_);
    joint_state_interface_.registerHandle(state_handle);
    registerInterface(&joint_state_interface_);

    hardware_interface::JointHandle effort_handle(joint_state_interface_.getHandle(joint_name), &command_position_);
    effort_joint_interface_.registerHandle(effort_handle);
    registerInterface(&effort_joint_interface_);

    joint_limits_interface::getJointLimits(joint_name,nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle(effort_handle, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandle);
	    
}


SwingJoint::~SwingJoint()
{
}

  
void SwingJoint::read()
{
    position_ = hall_sensor_.getValue();
    //position_ = command_position_; // TODO: read sensor data. Assigning the last command positions leads to 0 error in control loop.
}


void SwingJoint::write()
{
    motor_.setCurrent(command_position_);
}

void SwingJoint::calibrate()
{
	hall_sensor_.setZeroPoint();
}
