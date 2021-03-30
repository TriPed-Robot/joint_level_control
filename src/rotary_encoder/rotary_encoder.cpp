#include "joint_level_control/rotary_encoder/rotary_encoder.h"
#include "joint_level_control/rotary_encoder/extend_sensor_rosinterface.h"


RotaryEncoder::RotaryEncoder(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint8_t spi_speed, uint8_t spi_delay) 
: spi_device__(spi_device), spi_cs_id__(spi_cs_id), spi_mode__(spi_mode), spi_bits__(spi_bits), spi_speed__(spi_speed), spi_delay__(spi_delay)
{
}


RotaryEncoder::~RotaryEncoder()
{
}


double RotaryEncoder::getValue()
{
    return readExtendAngle(spi_device__, spi_cs_id__, spi_mode__, spi_bits__, spi_speed__, spi_delay__);
}

