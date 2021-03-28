#ifndef _MULTIPLECONTROLLERPARSER_H_
#define _MULTIPLECONTROLLERPARSER_H_


#include "joint_level_control/motor/message_parser.h"


class MultipleControllerParser {
    private:
        uint8_t* can_ids;
        MessageParser* parsers;
        uint8_t num_of_controllers;
    public:
        MultipleControllerParser();
        int addController(uint8_t can_id);
        message_t handleFrame(can_frame frame);
        ~MultipleControllerParser();
};


#endif
