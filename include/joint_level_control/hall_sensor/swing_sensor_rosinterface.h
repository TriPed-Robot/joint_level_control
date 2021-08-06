#ifndef SWING_SENSOR_ROS
#define SWING_SENSOR_ROS

#include <string>

#define SPI_CMD_ERROR 0x01 // flag if the cmd frame error bit in the spi msg is set
#define SPI_PARITY_ERROR 0x02 // flag if the parity of the spi msg is wrong
#define SPI_DEVICE_ERROR 0x04 // flag if the spi device can't be opened
#define SPI_MODE_ERROR 0x08 // flag if the spi mode can't be set / read
#define ANGLE_OUT_OF_BOUNDS_ERROR 0x10 // flag if the returned value is unfeasible

// reads angle counts from a swing sensor via spi, sets flags in error parameters if errors occurred.
uint16_t readSwingAngle(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay, uint8_t* error);

#endif