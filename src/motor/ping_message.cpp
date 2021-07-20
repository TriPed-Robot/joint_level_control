#include "joint_level_control/motor/message_builder.h"


PingMessage::PingMessage(uint8_t address) {
    can_address = address;
    data[0] = PING_SIGNATURE;
    data[1] = DATAKEY_1;
    data[2] = DATAKEY_2;
}
