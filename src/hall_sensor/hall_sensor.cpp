#include "joint_level_control/hall_sensor/hall_sensor.h"
#include "joint_level_control/hall_sensor/swing_sensor_rosinterface.h"
#include "joint_level_control/hall_sensor/SimpleGPIO.h" // control GPIO pins
#include <unistd.h> // usleep
using namespace std;

HallSensor::HallSensor(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay, double zero_point) 
: spi_device_(spi_device), spi_cs_id_(spi_cs_id), spi_mode_(spi_mode), spi_bits_(spi_bits), spi_speed_(spi_speed), spi_delay_(spi_delay), zero_point_(zero_point)
{
    // maybe move the open(spi_device) part here

    // setup multiplexer pins
    mux_selector_pin_1_ = 117; // p9_25, TODO: get this from yaml.
    mux_selector_pin_2_ = 115; //p9_27

    gpio_export(mux_selector_pin_1_);    // Tell OS to use this pin
    usleep(1000*1000); // wait >100 ms !!IMPORTANT!! the OS needs this time!
    gpio_export(mux_selector_pin_2_);
    usleep(1000*1000); // wait >100 ms !!IMPORTANT!! the OS needs this time!
    gpio_set_dir(mux_selector_pin_1_, OUTPUT_PIN);   // Set pin as output direction
	gpio_set_dir(mux_selector_pin_2_, OUTPUT_PIN); 
}


HallSensor::~HallSensor()
{
    // maybe move the spi close part here.

    // free multiplexer pins
    gpio_unexport(mux_selector_pin_1_);
    gpio_unexport(mux_selector_pin_2_);
}


void HallSensor::setZeroPoint()
{
    uint16_t counts = readSwingAngle(spi_device_, spi_cs_id_, spi_mode_, spi_bits_, spi_speed_, spi_delay_); // currently the ID is unnecessary, however in the future a distinction is necessary
    double range    = 2*3.1415926535;
    zero_point_     = (((double)counts)/16384*range); 

}

double HallSensor::getValue()
{
    PIN_VALUE value_pin1 = (spi_cs_id_ & 0x1) ? HIGH : LOW;
    PIN_VALUE value_pin2 = (spi_cs_id_ & 0x2) ? HIGH : LOW;
    //gpio_set_value(mux_selector_pin_1_,value_pin1);
    //gpio_set_value(mux_selector_pin_2_,value_pin2);

    gpio_set_value(mux_selector_pin_1_,HIGH);
    gpio_set_value(mux_selector_pin_2_,LOW);
    //usleep(1000*1000); // wait >100 ms !!IMPORTANT!! the OS needs this time!
    uint16_t counts = readSwingAngle(spi_device_, spi_cs_id_, spi_mode_, spi_bits_, spi_speed_, spi_delay_); // currently the ID is unnecessary, however in the future a distinction is necessary

    double range = 2*3.1415926535;
    double angle    = (((double)counts)/16384*range); 
    if (angle<= zero_point_-range/2)
    {
       angle = angle + range;
    }
    if (angle > zero_point_ +range/2)
    {
       angle = angle - range;
    }	 
    return angle-zero_point_;
}

