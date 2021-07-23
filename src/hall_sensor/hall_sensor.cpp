#include "joint_level_control/hall_sensor/hall_sensor.h"
#include "joint_level_control/hall_sensor/swing_sensor_rosinterface.h"
#include "joint_level_control/hall_sensor/SimpleGPIO.h" // control GPIO pins
#include <boost/interprocess/sync/named_mutex.hpp>
#include <unistd.h> // usleep
#include <iostream>



using namespace std; // TODO: remove this, if possible. currently needed for gpio library...

HallSensor::HallSensor(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay, double zero_point) 
: spi_device_(spi_device), spi_cs_id_(spi_cs_id), spi_mode_(spi_mode), spi_bits_(spi_bits), spi_speed_(spi_speed), spi_delay_(spi_delay), zero_point_(zero_point)
{
    // maybe move the open(spi_device) part here

    // setup multiplexer pins
    
    mux_selector_pin_1_ = 117; // p9_25, TODO: get this from yaml.
    mux_selector_pin_2_ = 115; //p9_27


    std::cout << "HS_params: spi device: " << spi_device << ", ID: " << unsigned(spi_cs_id)<< " , mode: " << unsigned(spi_mode) <<  std::endl;
    boost::interprocess::named_mutex named_mtx_{boost::interprocess::open_or_create, "multiplexer_mtx"};


}


HallSensor::~HallSensor()
{
    // maybe move the spi close part here.

}


void HallSensor::setZeroPoint()
{
    uint16_t counts = readSwingAngle(spi_device_, spi_cs_id_, spi_mode_, spi_bits_, spi_speed_, spi_delay_); // currently the ID is unnecessary, however in the future a distinction is necessary
    double range    = 2*3.1415926535;
    zero_point_     = (((double)counts)/16384*range); 

}

double HallSensor::getValue()
{
    named_mtx_.lock();

    if (spi_cs_id_ == 1)
    {
        gpio_set_value(117,HIGH); 
        gpio_set_value(115,LOW);
    }
    else if (spi_cs_id_ == 2)
    {
        gpio_set_value(117,LOW); 
        gpio_set_value(115,HIGH);
    }
    else
    {
        gpio_set_value(117,HIGH); 
        gpio_set_value(115,HIGH);
    }
    usleep(2000); 
    
    uint16_t counts = readSwingAngle(spi_device_, spi_cs_id_, spi_mode_, spi_bits_, spi_speed_, spi_delay_); // currently the ID is unnecessary, however in the future a distinction is necessary
    named_mtx_.unlock();

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

