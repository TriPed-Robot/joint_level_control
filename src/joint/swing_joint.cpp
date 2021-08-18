#include "joint_level_control/joint/swing_joint.h"
#include <iostream> // std cout debug

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
    uint spi_error_treshold,
    uint16_t mux_sel_pin_1, 
    uint16_t mux_sel_pin_2)
    : position_(0.0), 
    velocity_(0.0), 
    effort_(0.0), 
    command_position_(0.0), 
    error_command_position_(error_command_position), 
    error_state_(0),
    error_limit_(spi_error_treshold),
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
    // TODO: maybe set position_ only if NOT in ERROR state
    position_ = hall_sensor_.getValue();
    
    //TODO: check for invalid position:
    uint8_t error = hall_sensor_.getErrors(); // get last errors
    if (error){
        // errors in last read!
        // use weights to differentiate error criticalness
        if (error & SPI_CMD_ERROR)
        {
            /* last command frame was invalid */
            error_state_+= 5; // increment error state counter
	    std::cout << "Error: CMD" << std::endl;
        }
        if (error & SPI_PARITY_ERROR)
        {
            /* last parity was wrong */
            error_state_+= 5; // increment error state counter
	    std::cout << "Error: PARITY" << std::endl;
        }
        if (error & SPI_DEVICE_ERROR)
        {
            /* spi device couldn't e opened */
            error_state_+= 50; // increment error state counter
	    std::cout << "Error: DEVICE" << std::endl;
        }
        if (error & SPI_MODE_ERROR)
        {
            /* spi mode couldn't be set / read */
            //error_state_+= 1; // increment error state counter
	   //std::cout << "Error: MODE"<< std::endl;
        }
        if (error & ANGLE_OUT_OF_BOUNDS_ERROR)
        {
            /* last returned value was unfeasible / sensor NOT connected */
           error_state_+= error_limit_; // increment error state counter
	   std::cout << "Error: OOB"<< std::endl;
        }
    }else if(error_state_ < error_limit_){  
        // No error -> have error state decay over time
        // Also ERROR state is not yet reached
        uint decay_rate = 40;
        if (error_state_ - decay_rate >= 0)
        {
            error_state_ -= decay_rate;
        }else{
            error_state_ = 0;
        }
    }
}


void SwingJoint::write()
{
    if(error_state_ >= error_limit_){ // ERROR STATE
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
    return error_state_ >= error_limit_; // true if in ERROR state
}
