#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H 

#include <string>

/**
 * \brief This class abstracts the communication with the sensors measuring the state of the swingjoints.
 *
 * The left and right swingsensor, measure the position the swingjoints.
They are implemented using As5047D Hall Sensors as well as magnets attached to the output lever of the motor assembly (pictured below).
 * ![hall_sensor_placement](https://raw.githubusercontent.com/TriPed-Robot/Wiki/master/pictures/hardware/swing_sensor.png)
 * The magnets are affixed using 3d printed caps, which offer no way to predetermine the magnetic field at a given angle.
 * This means that the sensor output which is initially in counts has to be calibrated.
 * This is done either during initialization using the zero_point value or during runtime using the setZeroPoint method.
 * 
 * After initialization the sensors produce values in the range of [-pi,pi].
 * A angle of zero is specified as the angle at which the position of the output lever is perpendicular to the two mounting brackets of the hall sensor
 * Because these mounting brackets constrict the movement output lever the actually achievable range of motion is [-pi/2,pi/2].
 *
 * The embedded communication with the Sensor is done via SPI according to their datasheet.
 * The connection of the sensor to the beaglebone can be seen in the s<a href="https://github.com/TriPed-Robot/Wiki/wiki/Wiring-diagram"> wiring diagram</a>.
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
     * \brief Reads the Angle of the HallSensor
     * 
     * This function returns a double containing the current Angle of the joint according to the angle specifications above. 
     **/
    double getValue();
   /**
     * Sets the next Angle sent from the Sensor as the new zero_point_ for the angles.
     **/
    void setZeroPoint();

private:
    std::string spi_device_; // file which controls spi device
    uint8_t spi_cs_id_; // address of sensor in spi. This is used in the multiplexer pins
    uint8_t spi_mode_; 
    uint8_t spi_bits_; 
    uint32_t spi_speed_; 
    uint16_t spi_delay_;
    double zero_point_ ;

    uint16_t mux_selector_pin_1_; // 1st multiplexer selector pin for the chip select line
    uint16_t mux_selector_pin_2_; // 2nd multiplexer selector pin for the chip select line
};


#endif
