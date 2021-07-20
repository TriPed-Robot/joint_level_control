#ifndef _MESSAGEPARSER_H_
#define _MESSAGEPARSER_H_

#include "joint_level_control/motor/can_frame.h"
#include "joint_level_control/motor/response_messages.h"

#include <stdlib.h>

class MessageParser {
    private:
        uint8_t can_id;
        can_frame* control_data;
        uint8_t n;

    public:
        MessageParser(uint8_t can_address);
        ~MessageParser();
        message_t addFrame(can_frame frame);
};
#endif
