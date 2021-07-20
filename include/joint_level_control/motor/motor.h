#ifndef MOTOR_H
#define MOTOR_H


#include <string>
#include <thread>

#include "joint_level_control/motor/hacker.h" 

/**
 * \brief This class abstracts the communication with the motorcontrollers (currently Hacker Herkules 5 BLDC).
 * */

class Motor
{
public:
    /**
     * \biref Constructor of the Motor class
     * @param CAN-ID and Adress of the to be controller BLDC
     **/
    Motor(const std::string& can_name, uint8_t can_id);
    ~Motor();
    /**
     * Sets the current of this class instance.
     * Contrary to the Can function it accepts a double, meaning its sets the true current
     **/
    void setCurrent(double current);
    int writeCAN(can_frame* frame);

private:
    void startAcknowldegeTask();
    int m_error;

private:
    int can_socket_;
    CurrentControl current_control_;
    std::thread acknowledge_thread_;
};


#endif
