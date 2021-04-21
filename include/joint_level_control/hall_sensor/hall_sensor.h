#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H 

#include <string>

/**
 * \brief This class abstracts the communication with the sensors measuring the state of the swing Swingjoints.
 * The measurment is provided by AS504D Hall Sensors which measure the field produced by a magnet afixed to the output lever of the motor assembly.
 * ![hall_sensor_placement](https://raw.githubusercontent.com/TriPed-Robot/Wiki/master/pictures/hardware/swing_sensor.png)
 * The sensors communicate via SPI and are connected according to the <a href="https://github.com/TriPed-Robot/Wiki/wiki/Wiring-diagram"> wiring diagram</a>.
 * */

class HallSensor
{
public:    
    /**
     * \brief Constructor of the HallSensor class
     * @param SPI device, Chip select ID, mode, number of bits, communication speed and delay as well as the angle which is considered zero by the kinematics.
     **/
    HallSensor(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay, double zero_point);
    ~HallSensor();
   /**
     * Reads the Angle of the HallSensor
     **/
    double getValue();
   /**
     * Sets the next Angle sent from the Sensor as new zeropoint for the angles.
     **/
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
