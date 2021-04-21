#ifndef SWING_JOINT_H
#define SWING_JOINT_H


#include <string>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "joint_level_control/hall_sensor/hall_sensor.h"
#include "joint_level_control/motor/motor.h"


/**
 * \brief This Packages the HallSensor class and Motor class together into a Hardware Interface
 *
 * The class offers two types of interfaces, a joint state interface which reads the joint position from the HallSensor class and a effort interface with which feedback controllers can controll the torque of the robot. 
 * More information on hardware interfaces can be found [here](http://wiki.ros.org/ros_control#Hardware_Interfaces).
 * It also offers a calibration function which calls the setZero method of the HallSensor. This is done to allow calibration via ROS services.
 * */
class SwingJoint : public hardware_interface::RobotHW
{
public:
    SwingJoint(const std::string& joint_name, const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay, const std::string& can_name, uint8_t can_id,double zero_point);
    ~SwingJoint();

    void read();
    void write();
    void calibrate();
    
private:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;

    double position_;   
    double velocity_;
    double effort_;
    double command_position_;
    
    Motor motor_;
    HallSensor hall_sensor_;
};


#endif
