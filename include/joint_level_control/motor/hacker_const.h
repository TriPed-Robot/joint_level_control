#ifndef _HACKER_CONST_H_
#define _HACKER_CONST_H_


#include <inttypes.h>

const uint8_t CAN_DLC = 0x08;
const uint8_t RETURN_ADDRESS = 0xFE;
const uint8_t DATAKEY_1 = 0xE5;
const uint8_t DATAKEY_2 = 0xD8;
const uint8_t SCAN_ADDRESS = 0xFA;
const uint8_t SCAN_SIGNATURE = 0xB6;
const uint8_t CONTROL_SIGNATURE = 0x0F;
const uint8_t PING_SIGNATURE = 0xA0;

typedef enum {
    Motorcontroller_State_Part = 0x00, // frame data
    Motorcontroller_State = 0x01, // struct Controlresponse
    Scan_Device = 0x02, // Motorcontroller*
    Ping_Device = 0x03, // data[0] = 1|0
    Unknown = 0x04 // frame data
} MessageType;

typedef struct Message {
    uint8_t can_address;
    uint8_t* data;
    MessageType type;
} message_t;


#endif
