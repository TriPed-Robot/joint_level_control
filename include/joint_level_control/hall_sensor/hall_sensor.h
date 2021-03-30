#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H 

#include <string>

class HallSensor
{
public:
    HallSensor(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint8_t spi_speed, uint8_t spi_delay);
    ~HallSensor();
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
