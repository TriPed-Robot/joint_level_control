#include "joint_level_control/motor/message_parser.h"


MessageParser::MessageParser(uint8_t can_address) {
    n = 0;
    can_id = can_address;
}


MessageParser::~MessageParser() {
    free(control_data);
}


controlresponse_t* buildControlResponse(can_frame* buffer, uint8_t len) {
    controlresponse_t* response = (controlresponse_t*)malloc(sizeof(controlresponse_t));
    uint8_t b = 2;
    response->battery_current = (int16_t)(buffer[0].data[b] << 8 | buffer[0].data[b+1]);
    response->reserved_data = (uint16_t)(buffer[1].data[b] << 8 | buffer[1].data[b+1]);
    response->pwm_duty = (int16_t)(buffer[2].data[b] << 8| buffer[2].data[b+1]);
    b += 2;
    response->motor_current = (int16_t)(buffer[0].data[b] << 8 | buffer[0].data[b+1]);
    response->ppm_adc = (uint16_t)(buffer[1].data[b] << 8 | buffer[1].data[b+1]);
    response->pcb_temperature = (int16_t)(buffer[2].data[b] << 8 | buffer[2].data[b+1]);
    b += 2;
    response->battery_voltage = (uint16_t)(buffer[0].data[b] << 8 | buffer[0].data[b+1]);
    response->motor_temperature = (int16_t)(buffer[1].data[b] << 8 | buffer[1].data[b+1]);
    response->state = (state_t)buffer[2].data[b];
    response->fault = (fault_t)buffer[2].data[b+1];
    return response;
}


scanresponse_t* buildScanResponse(can_frame frame) {
    scanresponse_t* response = (scanresponse_t*)malloc(sizeof(scanresponse_t));
    uint8_t b = 1;
    response->mode = frame.data[b];
    b += 1;
    response->can_address = (uint16_t)(frame.data[b] << 8 | frame.data[b + 1]);
    b +=2;
    response->uid = (uint32_t)(frame.data[b] << 24 | frame.data[b+1] << 16 | frame.data[b+2] << 8 | frame.data[b+3]);
    return response;
}


message_t MessageParser::addFrame(can_frame frame) {
    message_t msg;
    switch(frame.data[0]) {
        case 0xD1 ... 0xDF:
            switch(n) {
                case 0:
                    if(frame.data[1] != 0x00) {
                        msg.type = Unknown;
                    } else {
                        control_data[n] = frame;
                        n += 1;
                        msg.type = Motorcontroller_State_Part;
                    }
                    break;
                case 1:
                    if(frame.data[1] != 0x01) {
                        msg.type = Unknown;
                    } else {
                        control_data[n] = frame;
                        n +=1;
                        msg.type = Motorcontroller_State_Part;
                    }
                    break;
                case 2:
                    if(frame.data[1] != 0x02) {
                        msg.type = Unknown;
                    } else {
                        control_data[n] = frame;
                        n +=1;
                        msg.type = Motorcontroller_State_Part;
                    }
                    break;
                case 3:
                    if(frame.data[1] != 0x03) {
                        msg.type = Unknown;
                    } else {
                        control_data[n] = frame;
                        n = 0;
                        msg.data = (uint8_t*)buildControlResponse(control_data, n);
                        msg.type = Motorcontroller_State;
                    }
                    break;
                default:
                    msg.type = Unknown;
            }
            break;
        case SCAN_SIGNATURE:
            msg.type = Scan_Device;
            msg.data = (uint8_t*)buildScanResponse(frame);
            break;
        case PING_SIGNATURE:
            break;
        default:
            msg.type = Unknown;
            msg.data = frame.data;
            break;
    }
    return msg;
}
