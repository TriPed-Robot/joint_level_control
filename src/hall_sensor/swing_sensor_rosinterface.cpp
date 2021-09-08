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
//#include <iostream>

#include "joint_level_control/hall_sensor/swing_sensor_rosinterface.h"


#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

// helper function: checks if parity of recieved message is correct
// returns 1 if parity of (rx1,rx0) is even, else returns 0.
uint8_t checkparity(uint8_t rx1, uint8_t rx0){
	uint16_t rx = ((uint16_t)rx1) << 8 | rx0;
	rx ^= rx >> 8;
	rx ^= rx >> 4;
	rx ^= rx >> 2;
	rx ^= rx >> 1;
	return (~rx) & 1;
	
}

// returns the angle counts of the selected swing sensor, sets error parameter bits if necessary
uint16_t readSwingAngle(const std::string& spi_device, uint8_t spi_cs_id, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed, uint16_t spi_delay, uint8_t* error = NULL)
{

	
    int fd; // file descriptor
	fd = open(spi_device.c_str(), O_RDWR); //opens SPI device
	if(fd < 0){
		*error |= SPI_DEVICE_ERROR;
	}
 
	uint8_t tx[] = {0xFF,0xFF}; // send buffer // sends larger indice first LIFO! 
	uint8_t rx[ARRAY_SIZE(tx)] = {0, }; // recieve buffer
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.speed_hz = spi_speed,
		.delay_usecs = spi_delay,
		.bits_per_word = spi_bits
	};

	// setting spi to mode 1: CPOL=0, CPHA = 1
	int ret = ioctl(fd, SPI_IOC_WR_MODE32, &spi_mode);
	if (ret == -1){
		*error |= SPI_MODE_ERROR;
	}
		
	ret = ioctl(fd, SPI_IOC_RD_MODE32, &spi_mode);
	if (ret == -1){
		*error |= SPI_MODE_ERROR;
	}
		

    ioctl(fd, SPI_IOC_MESSAGE(1), &tr); // transmit data over SPI to 
    close(fd); // closes spi device

	uint16_t resultAngle = ((uint16_t) (rx[1]& 0x3F)) << 8 | rx[0];
	
	// checking for (parity) errors: 14th bit of msg is 1
	uint16_t errorbit = rx[1] & 0x40;
	if (errorbit) // error in last CMD frame (Read/Write command to sensor)
	{
		*error |= SPI_CMD_ERROR;
	}
	if(!checkparity(rx[1],rx[0])){
		*error |=  SPI_PARITY_ERROR;
	}

	return resultAngle;
}
