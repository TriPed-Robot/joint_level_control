#ifndef EXTEND_SENSOR_ROS
#define EXTEND_SENSOR_ROS

#include <string>

// reads angle from the extend sensor of the leg
// returns the angle in radians (i think)
double readExtendAngle(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay);

#endif