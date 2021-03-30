#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H 

#include <string>

class RotaryEncoder
{
public:
    RotaryEncoder(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint8_t spi_speed, uint8_t spi_delay);
    ~RotaryEncoder();
    double getValue();

private:
    std::string spi_device__;
    uint8_t spi_cs_id__;
    uint8_t spi_mode__; 
    uint8_t spi_bits__; 
    uint8_t spi_speed__; 
    uint8_t spi_delay__;

};

#endif
