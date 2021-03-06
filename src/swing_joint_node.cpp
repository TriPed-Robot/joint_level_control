#include "ros/package.h"
#include "ros/ros.h"
#include <iostream>

#include <stdio.h>
#include <stdlib.h> // itoa

#include <string>
#include <ros/console.h> // debug

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include "controller_manager/controller_manager.h"
#include "joint_level_control/joint/swing_joint.h"




int main(int argc, char** argv)
{    
    ros::init(argc, argv, "swing/joint");
    ros::NodeHandle node;
    
    // set debug level in ROS: used for ros console debug------
    
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    // ------

    std::string joint_name;    
    node.getParam("joint_name", joint_name);

    //spi params
    std::string spi_device; 
    int spi_cs_id_int, spi_mode_int, spi_bits_int, spi_speed_int, spi_delay_int, 
        spi_mux_sel_pin_1_int, spi_mux_sel_pin_2_int, spi_error_treshold_int;
    double zero_point, spi_error_motor_default_value, joint_min_pos, joint_max_pos;
    while (!node.getParam("hall_sensor/spi_device", spi_device)){sleep(0.1);}
    while (!node.getParam("hall_sensor/spi_cs_id", spi_cs_id_int)){sleep(0.1);}
    while (!node.getParam("hall_sensor/spi_mode", spi_mode_int)){sleep(0.1);}
    while (!node.getParam("hall_sensor/spi_bits", spi_bits_int)){sleep(0.1);}
    while (!node.getParam("hall_sensor/spi_speed", spi_speed_int)){sleep(0.1);}
    while (!node.getParam("hall_sensor/spi_delay", spi_delay_int)){sleep(0.1);}
    while (!node.getParam("hall_sensor/zero_point", zero_point)){sleep(0.1);}
    while (!node.getParam("hall_sensor/spi_multiplexer_select_pin_1",spi_mux_sel_pin_1_int)){sleep(0.1);}
    while (!node.getParam("hall_sensor/spi_multiplexer_select_pin_2",spi_mux_sel_pin_2_int)){sleep(0.1);}
    while (!node.getParam("hall_sensor/spi_error_motor_default_value",spi_error_motor_default_value)){sleep(0.1);}
    while (!node.getParam("hall_sensor/spi_error_treshold",spi_error_treshold_int)){sleep(0.1);}
    while (!node.getParam("joint_limits/"+joint_name+"/min_position", joint_min_pos)){sleep(0.1);}
    while (!node.getParam("joint_limits/"+joint_name+"/max_position", joint_max_pos)){sleep(0.1);}

    uint8_t spi_cs_id = static_cast<uint8_t>(spi_cs_id_int);
    uint8_t spi_mode = static_cast<uint8_t>(spi_mode_int);
    uint8_t spi_bits = static_cast<uint8_t>(spi_bits_int);
    uint32_t spi_speed = static_cast<uint32_t>(spi_speed_int);
    uint16_t spi_delay = static_cast<uint16_t>(spi_delay_int);
    uint16_t spi_mux_sel_pin_1 = static_cast<uint16_t>(spi_mux_sel_pin_1_int);
    uint16_t spi_mux_sel_pin_2 = static_cast<uint16_t>(spi_mux_sel_pin_2_int);
    uint spi_error_treshold = static_cast<uint>(spi_error_treshold_int);

    std::string can_name;
    node.getParam("motor/can_name", can_name);
    int can_id_integer;
    node.getParam("motor/can_id", can_id_integer);
    uint8_t can_id = static_cast<uint8_t>(can_id_integer);
    ROS_DEBUG("Node: device: %s, id: %u, mode: %u, bits: %u, speed: %u, delay: %u, sel. pins: %u, %u \n Joint Limits: %f, %f", 
        spi_device.c_str(), spi_cs_id, spi_mode, spi_bits, spi_speed, spi_delay, spi_mux_sel_pin_1, spi_mux_sel_pin_2, joint_min_pos, joint_max_pos);

    SwingJoint swing_joint(joint_name, spi_device, spi_cs_id, spi_mode, spi_bits, spi_speed, spi_delay, can_name, can_id, zero_point, spi_error_motor_default_value, spi_error_treshold, spi_mux_sel_pin_1, spi_mux_sel_pin_2, joint_min_pos, joint_max_pos);
    
    controller_manager::ControllerManager controller_manager(&swing_joint);  
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Time previous_time = ros::Time::now();
    
    ros::Rate rate(100); // [Hz]

    // more setup for diagnostics
    // publisher for diagnostics
    ros::Publisher diagnostic_pub = node.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics/" + joint_name,1);
    diagnostic_msgs::DiagnosticArray dia_array;
    diagnostic_msgs::DiagnosticStatus joint_status;
    
    joint_status.name = joint_name + "/status";
    joint_status.hardware_id = joint_name;
    joint_status.level = diagnostic_msgs::DiagnosticStatus::OK;
    joint_status.message = "initial status";
    dia_array.status.push_back(joint_status);
    uint errors = 0;
    sleep(2000); //wait for 2s
    diagnostic_pub.publish(dia_array);

    ros::Time debug_time = ros::Time::now();

    std::cout <<joint_name << ": ros node init complete!" << std::endl;

    while(ros::ok())
    {
        ros::Time time = ros::Time::now();
        ros::Duration period = time - previous_time;
        previous_time = time;
        
        swing_joint.read();       
        controller_manager.update(time, period);
        swing_joint.write();
        
        errors = swing_joint.getErrorState();
        if (errors)
        {
            joint_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
            joint_status.message = "SPI reading throws errors";
            dia_array.status[0] = joint_status;
            diagnostic_pub.publish(dia_array);
            ROS_DEBUG_THROTTLE(1,"ERROR STATE REACHED!");
	    } else if(time.toSec() - debug_time.toSec() > 0.5 ){
            // Not in ERROR state, give periodically updates 
            joint_status.level = diagnostic_msgs::DiagnosticStatus::OK;
            joint_status.message = "SPI reading OK";
            dia_array.status[0] = joint_status; 
            diagnostic_pub.publish(dia_array);
            debug_time = time;
            std::cout << "5s reached! " << joint_name << std::endl;
        }

        rate.sleep();
    }
    
    return EXIT_SUCCESS;
}
