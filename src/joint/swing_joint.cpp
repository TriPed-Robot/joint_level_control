#include "joint_level_control/joint/swing_joint.h"


SwingJoint::SwingJoint(
    const std::string& joint_name, 
    const std::string& spi_device, 
    uint8_t spi_cs_id, 
    uint8_t spi_mode, 
    uint8_t spi_bits, 
    uint32_t spi_speed, 
    uint16_t spi_delay, 
    const std::string& can_name, 
    uint8_t can_id, 
    double zero_point, 
    double error_command_position, 
    uint16_t mux_sel_pin_1, 
    uint16_t mux_sel_pin_2)
    : position_(0.0), 
    velocity_(0.0), 
    effort_(0.0), 
    command_position_(0.0), 
    error_command_position_(error_command_position), 
    error_state_(0), 
    motor_(can_name, can_id), 
    hall_sensor_(spi_device, spi_cs_id, spi_mode, spi_bits, spi_speed, spi_delay,zero_point, mux_sel_pin_1, mux_sel_pin_2)
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
    
    //TODO: check for invalid position:
    uint8_t error = hall_sensor_.getErrors(); // get last errors
    if (error){
        // errors in last read!
        error_state++; // increment error state counter
    }else{
        error_state = 0; // No error -> reset error state
    }
}


void SwingJoint::write()
{
    if(error_state_){ // error occurred, alternatively use error_state < max #Errors
        motor_.setCurrent(error_command_position_); // send default value to motor
    }else{
        motor_.setCurrent(command_position_);
    }
    
}

void SwingJoint::calibrate()
{
	hall_sensor_.setZeroPoint();
}

uint SwingJoint::getErrorState(){
    return error_state_;
}