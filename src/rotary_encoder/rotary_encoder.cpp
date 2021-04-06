#include "joint_level_control/rotary_encoder/rotary_encoder.h"
#include "joint_level_control/rotary_encoder/extend_sensor_rosinterface.h"


RotaryEncoder::RotaryEncoder(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay) 
: spi_device_(spi_device), spi_cs_id_(spi_cs_id), spi_mode_(spi_mode), spi_bits_(spi_bits), spi_speed_(spi_speed), spi_delay_(spi_delay)
{
}


RotaryEncoder::~RotaryEncoder()
{
}


double RotaryEncoder::getValue()
{
    return readExtendAngle(spi_device_, spi_cs_id_, spi_mode_, spi_bits_, spi_speed_, spi_delay_);
}

