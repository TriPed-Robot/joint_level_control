#include "joint_level_control/rotary_encoder/rotary_encoder.h"
#include "joint_level_control/rotary_encoder/extend_sensor_rosinterface.h"


RotaryEncoder::RotaryEncoder()
{
}


RotaryEncoder::~RotaryEncoder()
{
}


double RotaryEncoder::getValue()
{
    return readExtendAngle(EXTEND_SENSOR_ID);
}

