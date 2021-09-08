#ifndef SWING_JOINT_H
#define SWING_JOINT_H


#include <string>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
//#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include "joint_level_control/hall_sensor/hall_sensor.h"
#include "joint_level_control/motor/motor.h"


/**
 * \brief This Packages the HallSensor class and Motor class together into a Hardware Interface
 *
 * The class offers two types of interfaces, a joint state interface which reads the joint position from the HallSensor class and a effort interface with which feedback controllers can controll the torque of the robot. 
 * More information on hardware interfaces can be found [here](http://wiki.ros.org/ros_control#Hardware_Interfaces).
 * It also offers a calibration function which calls the setZero method of the HallSensor. This is done to allow calibration via ROS services.
 * Lastly it contains a function for reading out how many errors in a row the last read accesses had.
 * */
class SwingJoint : public hardware_interface::RobotHW
{
public:
    /**
     * \brief Constructor of the SwingJoint class
     */
    SwingJoint(
        const std::string& joint_name,  //!< string joint name
        const std::string& spi_device,  //!< string file descriptor of spi device
        uint8_t spi_cs_id,              //!< uint8_t id of sensor on multiplexer
        uint8_t spi_mode,               //!< uint8_t mode of spi transfer: clock phase & polarity
        uint8_t spi_bits,               //!< uint8_t spi bits per word
        uint32_t spi_speed,             //!< uint32_t spi transfer speed in Hz
        uint16_t spi_delay,             //!< uint16_t delay in usec after last bit transfer, before device deselect. 
        const std::string& can_name,    //!< string can name
        uint8_t can_id,                 //!< uint8_t can id
        double zero_point,              //!< double zero angle offset for sensor
        double error_command_position,  //!< double default motor position (current) if errors occur
        uint spi_error_treshold,        //!< uint upper limit for accumulated errors, till ERROR state is reached
        uint16_t mux_sel_pin_1,         //!< uint16_t 1st selector pin for multiplexer
        uint16_t mux_sel_pin_2,         //!< uint16_t 2nd selector pin for multiplexer
        double joint_min_pos,           //!< double lower joint limit
        double joint_max_pos);          //!< double upper joint limit

    ~SwingJoint(); //!< destructor of SwingJoint class

    /**
     * \brief Reads the Angle of the HallSensor and weighs errors of read
     * 
     * This function sets internal position variable for controller.
     * It also reads the errors of the read access, and calculates a weighted sum of them. 
     * The sum dictates, whether the joint is in ERROR state, which is reached if: sum > spi_error_treshold
     **/
    void read();
    /**
     * \brief Writes a command position to the motors 
     **/
    void write();
    /**
     * \brief Sets the zero angle of the hallsensor using an internal function  
     **/
    void calibrate();
    /**
     * \brief Returns true if the sensor is in ERROR state, false otherwise. 
     *
     *  Checks if the weighted sum of errors is greater than the error threshold.
     **/
    uint getErrorState();
    
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
    double error_command_position_; // default value for cmd_pos if errors occur
    uint error_state_; // current error state of the joint. 0: OK, otherwise #error_state errors occured in a row
    uint error_limit_; // max #errors in a row until ERROR state is reached
    double joint_min_pos_; // joint position limits
    double joint_max_pos_;
    
    Motor motor_;
    HallSensor hall_sensor_;

    ros::NodeHandle nh_;
};


#endif
