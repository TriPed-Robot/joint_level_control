#include "ros/package.h"
#include "ros/ros.h"

#include <string>
#include <ros/console.h> // debug

// IMPORTANT: This node is for testing the hall sensor only, it should not be used for joint control

#include "joint_level_control/hall_sensor/hall_sensor.h"

// Message definitions:
#include "std_msgs/Float64.h"


int main(int argc, char** argv)
{    
    ros::init(argc, argv, "hall_sensor");
    ros::NodeHandle node;

    // set debug level in ROS ------
  
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    // ------

    std::string spi_device; 
    int spi_cs_id_int, spi_mode_int, spi_bits_int, spi_speed_int, spi_delay_int, spi_mux_sel_pin_1_int, spi_mux_sel_pin_2_int;
    double zero_point;
    
    node.getParam("hall_sensor/spi_device", spi_device);
    node.getParam("hall_sensor/spi_cs_id", spi_cs_id_int);
    node.getParam("hall_sensor/spi_mode", spi_mode_int);
    node.getParam("hall_sensor/spi_bits", spi_bits_int);
    node.getParam("hall_sensor/spi_speed", spi_speed_int);
    node.getParam("hall_sensor/spi_delay", spi_delay_int);
    node.getParam("hall_sensor/zero_point", zero_point);
    node.getParam("hall_sensor/spi_multiplexer_select_pin_1", spi_mux_sel_pin_1_int);
    node.getParam("hall_sensor/spi_multiplexer_select_pin_2", spi_mux_sel_pin_2_int);


    uint8_t spi_cs_id = static_cast<uint8_t>(spi_cs_id_int);
    uint8_t spi_mode = static_cast<uint8_t>(spi_mode_int);
    uint8_t spi_bits = static_cast<uint8_t>(spi_bits_int);
    uint32_t spi_speed = static_cast<uint32_t>(spi_speed_int);
    uint16_t spi_delay = static_cast<uint16_t>(spi_delay_int);
    uint16_t spi_mux_sel_pin_1 = static_cast<uint16_t>(spi_mux_sel_pin_1_int);
    uint16_t spi_mux_sel_pin_2 = static_cast<uint16_t>(spi_mux_sel_pin_2_int);


    ROS_DEBUG("Node: device: %s, id: %u, mode: % u, bits: % u, speed: % u, delay: % u \n", spi_device, spi_cs_id, spi_mode, spi_bits, spi_speed, spi_delay);

    HallSensor hall_sensor(spi_device, spi_cs_id, spi_mode, spi_bits, spi_speed, spi_delay,zero_point,spi_mux_sel_pin_1, spi_mux_sel_pin_2);
    
    //ros::Subscriber current_subscriber = node.subscribe<std_msgs::Float64>("current", 1, setCurrentWrapper);
    ros::Publisher hall_sensor_publisher = node.advertise<std_msgs::Float64>("hall_sensor_angle",1000);
    std_msgs::Float64 angle_msg;

    ros::Rate loop_rate(10);
    
    while(ros::ok())
    {
        double angle = hall_sensor.getValue() * 2 * 3.141592655 / 16384.;
        angle_msg.data = angle;
        hall_sensor_publisher.publish(angle_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    //ros::spin();
    
    return EXIT_SUCCESS;
}
