#include "ros/package.h"
#include "ros/ros.h"
#include <iostream>

#include <string>
#include <ros/console.h> // debug

#include "controller_manager/controller_manager.h"

#include "joint_level_control/joint/swing_joint.h"




int main(int argc, char** argv)
{    
    ros::init(argc, argv, "swing/joint");
    ros::NodeHandle node;
    
    // set debug level in ROS ------
    // TODO: remove this later!
  
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    // ------

    std::string joint_name;    
    node.getParam("joint_name", joint_name);

    //spi params
    std::string spi_device; 
    int spi_cs_id_int, spi_mode_int, spi_bits_int, spi_speed_int, spi_delay_int, 
        spi_mux_sel_pin_1_int, spi_mux_sel_pin_2_int;
    double zero_point, spi_error_motor_default_value;
    
    node.getParam("hall_sensor/spi_device", spi_device);
    node.getParam("hall_sensor/spi_cs_id", spi_cs_id_int);
    node.getParam("hall_sensor/spi_mode", spi_mode_int);
    node.getParam("hall_sensor/spi_bits", spi_bits_int);
    node.getParam("hall_sensor/spi_speed", spi_speed_int);
    node.getParam("hall_sensor/spi_delay", spi_delay_int);
    node.getParam("hall_sensor/zero_point", zero_point);
    node.getParam("hall_sensor/spi_multiplexor_select_pin_1",spi_mux_sel_pin_1_int);
    node.getParam("hall_sensor/spi_multiplexor_select_pin_2",spi_mux_sel_pin_2_int);
    node.getParam("hall_sensor/spi_error_motor_default_value",spi_error_motor_default_value);

    uint8_t spi_cs_id = static_cast<uint8_t>(spi_cs_id_int);
    uint8_t spi_mode = static_cast<uint8_t>(spi_mode_int);
    uint8_t spi_bits = static_cast<uint8_t>(spi_bits_int);
    uint32_t spi_speed = static_cast<uint32_t>(spi_speed_int);
    uint16_t spi_delay = static_cast<uint16_t>(spi_delay_int);
    uint16_t spi_mux_sel_pin_1 = static_cast<uint16_t>(spi_mux_sel_pin_1_int);
    uint16_t spi_mux_sel_pin_2 = static_cast<uint16_t>(spi_mux_sel_pin_2_int);

    std::string can_name;
    node.getParam("motor/can_name", can_name);
    int can_id_integer;
    node.getParam("motor/can_id", can_id_integer);
    uint8_t can_id = static_cast<uint8_t>(can_id_integer);
    ROS_DEBUG("Node: device: %s, id: %u, mode: %u, bits: %u, speed: %u, delay: %u, sel. pins: %u, %u \n", spi_device.c_str(), spi_cs_id, spi_mode, spi_bits, spi_speed, spi_delay, spi_mux_sel_pin_1, spi_mux_sel_pin_2);

    SwingJoint swing_joint(joint_name, spi_device, spi_cs_id, spi_mode, spi_bits, spi_speed, spi_delay, can_name, can_id, zero_point, spi_error_motor_default_value, spi_mux_sel_pin_1, spi_mux_sel_pin_2);
    
    controller_manager::ControllerManager controller_manager(&swing_joint);  
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Time previous_time = ros::Time::now();
    
    ros::Rate rate(100); // [Hz]
    while(ros::ok())
    {
        ros::Time time = ros::Time::now();
        ros::Duration period = time - previous_time;
        previous_time = time;
        
        swing_joint.read();       
        controller_manager.update(time, period);
        swing_joint.write();
        
        //TODO: read error state from swing joint and publish a diagnostic msg

        rate.sleep();
    }
    
    return EXIT_SUCCESS;
}
