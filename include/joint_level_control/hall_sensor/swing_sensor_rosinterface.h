#ifndef SWING_SENSOR_ROS
#define SWING_SENSOR_ROS

#include <string>

// reads angle from a swing sensor, with given ID
double readSwingAngle(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint8_t spi_speed, uint8_t spi_delay);

#endif