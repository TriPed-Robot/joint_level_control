#include "joint_level_control/hall_sensor/hall_sensor.h"
#include "joint_level_control/hall_sensor/swing_sensor_rosinterface.h"


HallSensor::HallSensor(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint8_t spi_speed, uint8_t spi_delay) 
: spi_device__(spi_device), spi_cs_id__(spi_cs_id), spi_mode__(spi_mode), spi_bits__(spi_bits), spi_speed__(spi_speed), spi_delay__(spi_delay)
{
    // maybe move the open(spi_device) part here
}


HallSensor::~HallSensor()
{
    // maybe move the spi close part here.
}


double HallSensor::getValue()
{
    return readSwingAngle(spi_device__, spi_cs_id__, spi_mode__, spi_bits__, spi_speed__, spi_delay__); // currently the ID is unnecessary, however in the future a distinction is necessary
}

