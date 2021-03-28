#include "joint_level_control/motor/message_builder.h"


ScanMessage::ScanMessage() {
    can_address = SCAN_ADDRESS;
    data[0] = SCAN_SIGNATURE;
    data[1] = DATAKEY_1;
    data[2] = DATAKEY_2;
}
