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
    
    // set debug level in ROS ------
    // TODO: remove this later!
  
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    // ------

    // publisher for diagnostics
    sleep(2000); //wait for 2s
    ros::Publisher diagnostic_pub = node.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",1);
    diagnostic_msgs::DiagnosticArray dia_array;
    diagnostic_msgs::DiagnosticStatus joint_status;
    //diagnostic_msgs::KeyValue joint_status_error_value; // contains #errors of last spi readings
    //joint_status_error_value.key = "errors";
    //char int_str[4];
    //snprintf(int_str,sizeof(int_str),"%d",0); // cast int to string  
    //joint_status_error_value.value = int_str; // needs string as value

    std::string joint_name;    
    node.getParam("joint_name", joint_name);

    //spi params
    std::string spi_device; 
    int spi_cs_id_int, spi_mode_int, spi_bits_int, spi_speed_int, spi_delay_int, 
        spi_mux_sel_pin_1_int, spi_mux_sel_pin_2_int, spi_error_treshold_int;
    double zero_point, spi_error_motor_default_value;
    
    node.getParam("hall_sensor/spi_device", spi_device);
    node.getParam("hall_sensor/spi_cs_id", spi_cs_id_int);
    node.getParam("hall_sensor/spi_mode", spi_mode_int);
    node.getParam("hall_sensor/spi_bits", spi_bits_int);
    node.getParam("hall_sensor/spi_speed", spi_speed_int);
    node.getParam("hall_sensor/spi_delay", spi_delay_int);
    node.getParam("hall_sensor/zero_point", zero_point);
    node.getParam("hall_sensor/spi_multiplexer_select_pin_1",spi_mux_sel_pin_1_int);
    node.getParam("hall_sensor/spi_multiplexer_select_pin_2",spi_mux_sel_pin_2_int);
    node.getParam("hall_sensor/spi_error_motor_default_value",spi_error_motor_default_value);
    node.getParam("hall_sensor/spi_error_treshold",spi_error_treshold_int);

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
    ROS_DEBUG("Node: device: %s, id: %u, mode: %u, bits: %u, speed: %u, delay: %u, sel. pins: %u, %u \n", spi_device.c_str(), spi_cs_id, spi_mode, spi_bits, spi_speed, spi_delay, spi_mux_sel_pin_1, spi_mux_sel_pin_2);

    SwingJoint swing_joint(joint_name, spi_device, spi_cs_id, spi_mode, spi_bits, spi_speed, spi_delay, can_name, can_id, zero_point, spi_error_motor_default_value, spi_error_treshold, spi_mux_sel_pin_1, spi_mux_sel_pin_2);
    
    controller_manager::ControllerManager controller_manager(&swing_joint);  
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Time previous_time = ros::Time::now();
    
    ros::Rate rate(100); // [Hz]

    // more setup for diagnostics
    joint_status.name = joint_name + "/status";
    joint_status.hardware_id = joint_name;
    joint_status.level = diagnostic_msgs::DiagnosticStatus::OK;
    joint_status.message = "initial status";
    dia_array.status.push_back(joint_status);
    uint errors = 0;
    sleep(2000); //wait for 2s
    diagnostic_pub.publish(dia_array);

    ros::Time debug_time = ros::Time::now();

    std::cout << "Rosnode Init complete!" << std::endl;

    while(ros::ok())
    {
        ros::Time time = ros::Time::now();
        ros::Duration period = time - previous_time;
        previous_time = time;
        
        swing_joint.read();       
        controller_manager.update(time, period);
        swing_joint.write();
        
        errors = swing_joint.getErrorState();
        if (errors) // alternatively errors < max # errors
        {
            joint_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
            joint_status.message = "SPI reading throws errors";
	    dia_array.status[0] = joint_status;
	    diagnostic_pub.publish(dia_array);
            ROS_DEBUG_ONCE("ERROR STATE REACHED!");
	}else{
            //TODO: put correct status update in if(>5s) (need to actually put status in dia_array before publishing 
	    joint_status.level = diagnostic_msgs::DiagnosticStatus::OK;
            joint_status.message = "SPI reading OK";
	    ROS_DEBUG_THROTTLE(5, "SPI OK");
	    if(time.toSec() - debug_time.toSec() > 5. ){
		diagnostic_pub.publish(dia_array);
		debug_time = time;
		std::cout << "5s reached!" << std::endl;
	    }
        }
        //TODO: delete if this works without it
        /*snprintf(int_str,sizeof(int_str),"%d",errors); // cast int to string 
        joint_status_error_value.value = int_str;// send #errors in a row regardless
        joint_status.values.push_back(joint_status_error_value);*/
        //dia_array.status.clear(); // remove old status
	//dia_array.status.push_back(joint_status);



        rate.sleep();
    }
    
    return EXIT_SUCCESS;
}
