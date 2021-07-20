#include "joint_level_control/motor/multiple_controller_parser.h"


MultipleControllerParser::MultipleControllerParser() {
    num_of_controllers = 0;
}


int MultipleControllerParser::addController(uint8_t can_id) {
    if (num_of_controllers > 0) {
        for(uint8_t i = 0; i < num_of_controllers; i++) {
            if(can_id == can_ids[i]) {
                return 1;
            }
        }
    }
    can_ids[num_of_controllers] = can_id;
    parsers[num_of_controllers] = MessageParser(can_id);
    num_of_controllers += 1;
    return 0;
}


message_t MultipleControllerParser::handleFrame(can_frame frame) {
    for(uint8_t i = 0; i < num_of_controllers; i++) {
        if(can_ids[i] == frame.can_id) {
            return parsers[i].addFrame(frame);
        }
    }
    message_t err;
    err.type = Unknown;
    err.can_address = 0xFF;
    return err;
}


MultipleControllerParser::~MultipleControllerParser() {
    free(can_ids);
    for(uint8_t i = 0; i < num_of_controllers; i++) {
        parsers[i].~MessageParser();
    }
    free(parsers);
}
