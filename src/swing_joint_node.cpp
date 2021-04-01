#include "ros/package.h"
#include "ros/ros.h"

#include "controller_manager/controller_manager.h"

#include "joint_level_control/joint/swing_joint.h"


int main(int argc, char** argv)
{    
    ros::init(argc, argv, "swing_joint");
    ros::NodeHandle node;
    
    std::string joint_name;    
    node.getParam("joint_name", joint_name);
    std::string can_name;
    node.getParam("motor/can_name", can_name);
    int can_id_integer;
    node.getParam("motor/can_id", can_id_integer);
    uint8_t can_id = static_cast<uint8_t>(can_id_integer);
    
    SwingJoint swing_joint(joint_name, can_name, can_id);
    
    controller_manager::ControllerManager controller_manager(&swing_joint);  
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Time previous_time = ros::Time::now();
    
    ros::Rate rate(10); // [Hz]
    while(ros::ok())
    {
        ros::Time time = ros::Time::now();
        ros::Duration period = time - previous_time;
        previous_time = time;
        
        swing_joint.read();       
        controller_manager.update(time, period);
        swing_joint.write();
        
        rate.sleep();
    }
    
    return EXIT_SUCCESS;
}
