#include "ros/package.h"
#include "ros/ros.h"

#include "controller_manager/controller_manager.h"

#include "joint_level_control/joint.h"

// Message definitions:
#include "std_msgs/Float64.h"


static void _set_disturbance(Joint& joint, const std_msgs::Float64& disturbance)
{
    joint.set_disturbance(disturbance.data);
}


int main(int argc, char** argv)
{    
    ros::init(argc, argv, "joint_interface");
    ros::NodeHandle node;
    
    // Create own node handle to access the private parameters.
    ros::NodeHandle node_private_parameters("~");
    std::string joint_name;
    node_private_parameters.getParam("joint", joint_name);
    
    Joint joint(joint_name);
    controller_manager::ControllerManager controller_manager(&joint);
    
    // Currying the _set_disturbance function with help of closures.
    auto set_disturbance_wrapper = [&joint](const std_msgs::Float64::ConstPtr& p_disturbance)
    {
        _set_disturbance(joint, *p_disturbance);
    };
    ros::Subscriber disturbance_subscriber = node.subscribe<std_msgs::Float64>("disturbance", 1, set_disturbance_wrapper);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Time previous_time = ros::Time::now();
    
    ros::Rate rate(10); // [Hz]
    while(ros::ok())
    {
        ros::Time time = ros::Time::now();
        ros::Duration period = time - previous_time;
        previous_time = time;
        
        joint.read();       
        controller_manager.update(time, period);
        joint.write();
        
        rate.sleep();
    }
    
    return EXIT_SUCCESS;
}
