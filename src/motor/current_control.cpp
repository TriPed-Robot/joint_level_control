#include "joint_level_control/motor/message_builder.h"


CurrentControl::CurrentControl(uint8_t address) {
    can_address = address;
}


void CurrentControl::setCurrent(int32_t current) { // 0.1A res
    // TODO: check if Little-Endian
    // We assume that we are on a little endian device, and all data is transmitted as little endian
    data[0] = CONTROL_SIGNATURE;
    data[1] = 0x02;
    data[2] = current;
    data[3] = current>>8;
    data[4] = current>>16;
    data[5] = current>>24;
    data[6] = DATAKEY_1;
    data[7] = DATAKEY_2;

} // namespace CAN
