#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H 

#include <string>

class HallSensor
{
public:
    HallSensor(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay, double zero_point);
    ~HallSensor();
    double getValue();
    void setZeroPoint();

private:
    std::string spi_device_;
    uint8_t spi_cs_id_;
    uint8_t spi_mode_; 
    uint8_t spi_bits_; 
    uint32_t spi_speed_; 
    uint16_t spi_delay_;
    double zero_point_ ;

};


#endif
