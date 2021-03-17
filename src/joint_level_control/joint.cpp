#include "joint_level_control/joint.h"


Joint::Joint(const std::string& joint_name)
    : _disturbance(0.0), _joint_position(0.0), _joint_velocity(0.0), _joint_effort(0.0), _joint_command_position(0.0)
{
    // Setup hardware interface:
    hardware_interface::JointStateHandle state_handle(joint_name, &_joint_position, &_joint_velocity, &_joint_effort);
    _joint_state_interface.registerHandle(state_handle);
    registerInterface(&_joint_state_interface);

    hardware_interface::JointHandle effort_handle(_joint_state_interface.getHandle(joint_name), &_joint_command_position);
    _effort_joint_interface.registerHandle(effort_handle);
    registerInterface(&_effort_joint_interface);
}


Joint::~Joint()
{
}


void Joint::set_disturbance(double disturbance)
{
    _disturbance = disturbance;
}

   
void Joint::read()
{    
}


void Joint::write()
{
    
    _joint_position = _second_order_system.calculateOutput(_joint_command_position) + _disturbance;
}
