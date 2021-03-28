#include "joint_level_control/motor/message_builder.h"


MessageBuilder::MessageBuilder() {
    for(int i = 0; i < 8; i++) {
        data[i] = 0x00;
    }
}


MessageBuilder::MessageBuilder(uint8_t address) {
    can_address = address;
    MessageBuilder();
}


can_frame* MessageBuilder::getMessage() {
    can_frame* frame = (can_frame*)malloc(sizeof(can_frame));
    frame->can_id = can_address;
    frame->can_dlc = CAN_DLC;
    memcpy(frame->data, data, sizeof(data));
    return frame;
}
