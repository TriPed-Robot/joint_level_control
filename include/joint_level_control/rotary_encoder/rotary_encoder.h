#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H 

#include <string>
/**
 * \brief This class abstracts the communication with Sensors of the Extendjoints (currently a E6A2-C rotary encoder).
 * */
/**
 * \brief This class abstracts the communication with the sensors measuring the state of the extendjoint.
 *
 * The extendsensor measures the position of the extendjoint.This is is implemented using a omron E6A2-C rotary encoder which is connected to the drive shaft driven by the extendmotor and a photosensor (seen below)
 * \image  html https://raw.githubusercontent.com/TriPed-Robot/TriPed-Robot.github.io/master/images/extend_sensor.png height=500
 * The photosensor is needet since a rotary encoder does not measure absolute position.
 * A splint affixed to the leg is able to trigger the photosensor thereby providing a absolute position feedback.
 * There are multiple possible position of the splint, the current one can be seen below
 * \image html https://raw.githubusercontent.com/TriPed-Robot/TriPed-Robot.github.io/master/images/splint.png width=800
 * The readout of rotary encoder and photosensor is abstracted by an arduino which sends the calibrated angle values over spi
 *
 * It is important to note that the send angle is not the position of the driveshaft, but instead the state of the virtual extend joint.
 * More information about this virtual joint can be found [here](https://github.com/TriPed-Robot/TriPed-Reference-Document). 
 * */
class RotaryEncoder
{
public:
    /**
     * \biref Constructor of the RotaryEncoder class
     * @param SPI device, Chip select ID, mode, number of bits, communication speed and delay as well as the angle which is considered zero by the kinematics.
     **/
    RotaryEncoder(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay);
    ~RotaryEncoder();
   /**
     * Reads the Angle of the RotaryEncoder from the Arduino
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
