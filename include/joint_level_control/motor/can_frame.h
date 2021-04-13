#ifndef linux

#ifndef CAN_H_
#define CAN_H_

#include <inttypes.h>
/**
 * This struct is only a stub which is in use when not on a Linux System.
 * To view exact implementation of the struct on Linux, see
 * <a href="/usr/include/linux/can.h">/usr/include/linux/can.h</a>
 * \brief Stub to simplify development on non Linux Systems.
 * */
struct can_frame {
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8];
};

#endif
#endif
#ifdef linux
#include <linux/can.h>
#include <linux/can/raw.h>
#endif
