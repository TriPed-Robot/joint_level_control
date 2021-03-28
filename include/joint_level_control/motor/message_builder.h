#ifndef MESSAGEBUILDER_H_
#define MESSAGEBUILDER_H_


#include <stdlib.h>
#include <string.h>

#include "joint_level_control/motor/can_frame.h"
#include "joint_level_control/motor/hacker_const.h"


class MessageBuilder {
    protected:
        uint8_t data[8];
        uint8_t can_address;
    public:
        MessageBuilder(uint8_t address);
        MessageBuilder();
        can_frame* getMessage();
};

class ScanMessage : public MessageBuilder {
    public:
        ScanMessage();
};

class PingMessage : public MessageBuilder {
    public:
        PingMessage(uint8_t address);
};

class CurrentControl : public MessageBuilder {
    public:
        CurrentControl(uint8_t address);
        void setCurrent(int32_t current);
};


#endif
