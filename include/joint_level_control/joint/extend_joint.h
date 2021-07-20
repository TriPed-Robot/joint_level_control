#ifndef EXTEND_JOINT_H
#define EXTEND_JOINT_H


#include <string>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
//#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include "joint_level_control/motor/motor.h"
#include "joint_level_control/rotary_encoder/rotary_encoder.h"

/**
 * \brief This Packages the RotaryEncoder class and Motor class together into a Hardware Interface
 *
 * The class offers two types of interfaces, a joint state interface which reads the joint position from the RotaryEncoder class and a effort interface with which feedback controllers can controll the torque of the robot. 
 * More information on hardware interfaces can be found [here](http://wiki.ros.org/ros_control#Hardware_Interfaces).
 * */
class ExtendJoint : public hardware_interface::RobotHW
{
public:
    ExtendJoint(const std::string& joint_name, const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay, const std::string& can_name, uint8_t can_id);
    ~ExtendJoint();

    void read();
    void write();
    
private:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;

    joint_limits_interface::JointLimits limits;
    joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
    joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;

    double position_;   
    double velocity_;
    double effort_;
    double command_position_;
    
    Motor motor_;
    RotaryEncoder rotary_encoder_;

    ros::NodeHandle nh_;
};


#endif
