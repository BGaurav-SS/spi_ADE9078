#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include "registerMap.h"

#include <wiringPiSPI.h>
#include <wiringPi.h>

//Use wiringPi pin 6; physical pin 22 to send command to reset the IC.
#define RESET_PIN   6
//Use wiringPi pin 3; physical pin 15 as pin to read interrupt from IRQ1B pin.
#define IRQ1B_PIN   3

static const int CHANNEL = 0;


static char *spiDevice = "/dev/spidev0.0" ;
static uint8_t spiMode = SPI_MODE_1;
static uint8_t spiBPW = 8 ;
static uint32_t spiSpeed = 4000000 ;    //Speed set to 4.5MHz
static uint16_t spiDelay = 0;
static uint8_t spiLSBFirst = 0;

int initialize (void){
    //Reset ADE9078
    digitalWrite(RESET_PIN, LOW);
    delay(50);
    digitalWrite(RESET_PIN, HIGH);
    delay(50);


    printf("\nWaiting for RESET_DONE signal.\n");
    while (digitalRead(IRQ1B_PIN) != 0){
        //Wait until the RESET_DONE signal is generated.
    }

    return 1;
}

int main(int argc, char* argv[]){
	
	int fd;
    wiringPiSetup();

   	fd = wiringPiSPISetup(CHANNEL, 500000);

	pinMode(RESET_PIN, OUTPUT);
    pinMode(IRQ1B_PIN, INPUT);

	unsigned char buffer[6];

	if (initialize() != 1){
		printf("Error initializing the device.\n");
		return -1;
	}	

	buffer[0] = 0x40;
	buffer[1] = 0x20;
	buffer[0] = 0xFF;
	buffer[0] = 0xFF;


	while (1){
		wiringPiSPIDataRW(CHANNEL, buffer, 1);
		sleep(5);
	}
    return 0;
}

