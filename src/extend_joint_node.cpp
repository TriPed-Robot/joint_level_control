#include "ros/package.h"
#include "ros/ros.h"

#include "controller_manager/controller_manager.h"

#include "joint_level_control/joint/extend_joint.h"


int main(int argc, char** argv)
{    
    ros::init(argc, argv, "extend_joint");
    ros::NodeHandle node;
    
    std::string joint_name;    
    node.getParam("joint_name", joint_name);

    //spi params
    std::string spi_device;
    node.getParam("spi_device",spi_device);
    int spi_cs_id, spi_mode, spi_bits, spi_speed, spi_delay;
    node.getParam("spi_cs_id",spi_cs_id);
    node.getParam("spi_mode",spi_mode);
    node.getParam("spi_bits",spi_bits);
    node.getParam("spi_speed",spi_speed);
    node.getParam("spi_delay",spi_delay);

    std::string can_name;
    node.getParam("can_name", can_name);
    int can_id_integer;
    node.getParam("can_id", can_id_integer);
    uint8_t can_id = static_cast<uint8_t>(can_id_integer);
    
    ExtendJoint extend_joint(joint_name, spi_device, spi_cs_id, spi_mode, spi_bits, spi_speed, spi_delay, can_name, can_id);
    
    controller_manager::ControllerManager controller_manager(&extend_joint);  
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Time previous_time = ros::Time::now();
    
    ros::Rate rate(10); // [Hz]
    while(ros::ok())
    {
        ros::Time time = ros::Time::now();
        ros::Duration period = time - previous_time;
        previous_time = time;
        
        extend_joint.read();       
        controller_manager.update(time, period);
        extend_joint.write();
        
        rate.sleep();
    }
    
    return EXIT_SUCCESS;
}
