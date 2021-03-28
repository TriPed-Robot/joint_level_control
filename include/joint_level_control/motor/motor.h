#ifndef MOTOR_H
#define MOTOR_H


#include <string>
#include <thread>

#include "joint_level_control/motor/hacker.h" 


class Motor
{
public:
    Motor(const std::string& can_name, uint8_t can_id);
    ~Motor();
    void setCurrent(int32_t current);

private:
    void startAcknowldegeTask();

private:
    int can_socket_;
    CurrentControl current_control_;
    std::thread acknowledge_thread_;
};


#endif
