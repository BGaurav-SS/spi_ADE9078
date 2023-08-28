#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include "registerMap.h"

#ifndef spiADE9078_H
#define spiADE9078_H

uint32_t readByte(uint16_t reg, uint32_t* data);
uint32_t writeByte(uint32_t reg, uint32_t data);
int spi_open(char* dev);

#endif