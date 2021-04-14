#include "joint_level_control/hall_sensor/hall_sensor.h"
#include "joint_level_control/hall_sensor/swing_sensor_rosinterface.h"


HallSensor::HallSensor(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay) 
: spi_device_(spi_device), spi_cs_id_(spi_cs_id), spi_mode_(spi_mode), spi_bits_(spi_bits), spi_speed_(spi_speed), spi_delay_(spi_delay)
{
    // maybe move the open(spi_device) part here
}


HallSensor::~HallSensor()
{
    // maybe move the spi close part here.
}


double HallSensor::getValue()
{
    uint16_t counts = readSwingAngle(spi_device_, spi_cs_id_, spi_mode_, spi_bits_, spi_speed_, spi_delay_); // currently the ID is unnecessary, however in the future a distinction is necessary

    uint16_t  zero_point_=0;
    double range = 2*3.1415926535;
    double angle    = (((double)counts)/16384*range); 
    if (angle<= zero_point_-range/2)
    {
       angle = angle + range;
    }
    if (angle > zero_point_ +range/2)
    {
       angle = angle - range;
    }	 
    return angle;
}

