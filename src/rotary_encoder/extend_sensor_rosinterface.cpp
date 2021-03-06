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
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h> //memcpy: bytes->float

#include "joint_level_control/rotary_encoder/extend_sensor_rosinterface.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))


double readExtendAngle(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay)
{
	//TODO: do sth with the sensor ID --> set chip select pins at multiplexer
    int fd;
	fd = open(spi_device.c_str(), O_RDWR); //opens SPI device, maybe put this in a once called init
 
	uint8_t tx[] = {0x01,0x00,0x00,0x00,0x00}; // send buffer init 1 + 4 bytes
	uint8_t rx[ARRAY_SIZE(tx)] = {0, }; // recieve buffer
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.speed_hz = spi_speed,
		.delay_usecs = spi_delay,
		.bits_per_word = spi_bits
	};

    ioctl(fd, SPI_IOC_MESSAGE(1), &tr); // transmit data over SPI to 
    close(fd); //
	//printf("Test rx: %d,%d,%d,%d,%d\n",rx[0],rx[1],rx[2],rx[3],rx[4]);
	//uint16_t angle = ((uint16_t) (rx[1]& 0x3F)) << 8 | rx[0];
	float angle;
	memcpy(&angle,&rx[1],4); 
	
	double resultAngle =  (double)angle;
    return resultAngle;
}