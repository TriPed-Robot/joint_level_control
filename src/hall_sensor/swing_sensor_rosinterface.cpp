/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "joint_level_control/hall_sensor/swing_sensor_rosinterface.h"

#include <ros/console.h> // debug console

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))


// returns the angle the swing sensor with sensorID is measuring. Angle is in RADIANS!
double readSwingAngle(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay)
{
	//TODO: do sth with the ID --> set fitting chip select pins

	ROS_DEBUG("debug: device: %s, id: %u, mode: % u, bits: % u, speed: % u, delay: % u \n", spi_device.c_str(), spi_cs_id, spi_mode, spi_bits, spi_speed, spi_delay);
	
    int fd; // file descriptor
	fd = open(spi_device.c_str(), O_RDWR); //opens SPI device, maybe put this in a once called init
 
	uint8_t tx[] = {0xFF,0xFF}; // send buffer [not used for Swing Sensor, since MOSI pin is connected to VDD anyways]
	uint8_t rx[ARRAY_SIZE(tx)] = {0, }; // recieve buffer
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.speed_hz = spi_speed,
		.delay_usecs = spi_delay,
		.bits_per_word = spi_bits
	};

	/* maybe this is useful: ...
	et = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode);
	if (ret == -1)
		pabort("can't get spi mode");
	*/

    ioctl(fd, SPI_IOC_MESSAGE(1), &tr); // transmit data over SPI to 
    close(fd); //
	uint16_t angle = ((uint16_t) (rx[1]& 0x3F)) << 8 | rx[0];
	double resultAngle =  (((double)angle)/16384.*2*3.1415926535); // converts counts to radians
    return resultAngle;
}
