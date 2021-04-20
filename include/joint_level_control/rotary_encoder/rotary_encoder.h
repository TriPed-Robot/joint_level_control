#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H 

#include <string>
/**
 * \brief This class abstracts the communication with Sensors of the Extendjoints (currently a E6A2-C rotary encoder).
 * */

class RotaryEncoder
{
public:
    /**
     * \biref Constructor of the RotaryEncoder class
     * @param SPI device, Chip select ID, mode, number of bits, communication speed and delay as well as the angle which is considered zero by the kinematics.
     **/
    RotaryEncoder(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay, double zero_point);
    ~RotaryEncoder();
   /**
     * Reads the Angle of the RotaryEncoder
     **/
    double getValue();

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
