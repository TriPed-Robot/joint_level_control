#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H 

#include <string>

class RotaryEncoder
{
public:
    RotaryEncoder(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay);
    ~RotaryEncoder();
    double getValue();

private:
    std::string spi_device_;
    uint8_t spi_cs_id_;
    uint8_t spi_mode_; 
    uint8_t spi_bits_; 
    uint32_t spi_speed_; 
    uint16_t spi_delay_;

};

#endif
