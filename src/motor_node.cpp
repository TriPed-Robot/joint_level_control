#include "ros/package.h"
#include "ros/ros.h"

#include "joint_level_control/motor/motor.h"

// Message definitions:
#include "std_msgs/Float64.h"


static void setCurrent(Motor& motor, const std_msgs::Float64& current)
{
    motor.setCurrent(current.data);
}


int main(int argc, char** argv)
{    
    ros::init(argc, argv, "motor");
    ros::NodeHandle node;
    
    std::string can_name;
    node.getParam("can_name", can_name);
    int can_id_integer;
    node.getParam("can_id", can_id_integer);
    uint8_t can_id = static_cast<uint8_t>(can_id_integer);
    
    Motor motor(can_name, can_id_integer);
    
    // Currying the setCurrent function with help of closures.
    auto setCurrentWrapper = [&motor](const std_msgs::Float64::ConstPtr& p_current)
    {
        setCurrent(motor, *p_current);
    };
    ros::Subscriber current_subscriber = node.subscribe<std_msgs::Float64>("current", 1, setCurrentWrapper);
    
    ros::spin();
    
    return EXIT_SUCCESS;
}
