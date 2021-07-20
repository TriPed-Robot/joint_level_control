#ifndef MESSAGEBUILDER_H_
#define MESSAGEBUILDER_H_


#include <stdlib.h>
#include <string.h>

#include "joint_level_control/motor/can_frame.h"
#include "joint_level_control/motor/hacker_const.h"
	
/**
 * \brief This class and its subclasses handle the creation of the CAN-Frames needed to control the Hacker Herkules 5 BLDC.
 */
class MessageBuilder {
    protected:
        uint8_t data[8];
        uint8_t can_address;
    public:
        /**
         * \brief Constructor of the MessageBuilder Class
         * @param address CAN-ID of the to-be controlled BLDC
         * */
        MessageBuilder(uint8_t address);
        MessageBuilder();
        /**
         * \brief gets the generated SocketCAN-Frame for utilisation by the callee.
         * @returns can_frame pointer for easy plug into write()
         **/
        can_frame* getMessage();
};
/**
 * \brief This subclass generates a Scan-Message so that new Controllers on the CAN-Bus can be found.
 * */
class ScanMessage : public MessageBuilder {
    public:
        ScanMessage();
};
/**
 * \brief This subclass generates a Ping-Message so that one can verify if a controller with an known address is active.
 * */
class PingMessage : public MessageBuilder {
    public:
        /**
         * Constructor of the PingMessage Class
         * \param address CAN-ID of the BLDC which should be pinged.
         **/
        PingMessage(uint8_t address);
};
/**
 * \brief This subclass generates Current-Control Messages which allow to control the motor via a current limit.
 * */
class CurrentControl : public MessageBuilder {
    public:
        /**
         * Constructor of the CurrentControl Class
         * \param address CAN-ID of the BLDC which is controlled by this class instance.
         **/
        CurrentControl(uint8_t address);
        /**
         * Sets the current of this class instance.
         * \param current The current which should be set. NOTE: Current is set in 0.1A increments. setCurrent(1) will set the BLDC to 0.1 A
         **/
        void setCurrent(int32_t current);
};

#endif
