#include "joint_level_control/hall_sensor/hall_sensor.h"
#include "joint_level_control/hall_sensor/swing_sensor_rosinterface.h"


HallSensor::HallSensor()
{
}


HallSensor::~HallSensor()
{
}


double HallSensor::getValue()
{
    return readSwingAngle(LEFT_SWING_SENSOR_ID); // currently the ID is unnecessary, however in the future a distinction is necessary
}

