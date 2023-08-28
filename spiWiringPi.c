/*
1. The ADE9078 is always a SPI slave.
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <malloc.h>
#include "registerMap.h"
//#include <fcntl.h>
//#include <sys/ioctl.h>
//#include <linux/spi/spidev.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#define SPI_CHAN    0
//The CS pin corresponds to WiringPi pin 10; physical pin 24.
#define CS      10
//Use wiringPi pin 6; physical pin 22 as reset pin.
#define IRQ1B   6

//The maximum clock speed supported is 10MHz.
#define SPEED   1000000



//Universal declarations..
unsigned char *buffer;
int delay_time = 100;

//Function to perform write operation in ADE9078 16-bit registers.
bool spiWrite16 (uint16_t regAddress, uint16_t regData){
  printf("\n...Writing initiated...");
  unsigned char buffer[2];
  uint16_t commandHeader;

  // digitalWrite (CS, LOW);


  //Send character header first.
  commandHeader = ((regAddress << 4 ) & 0xFFF0);
  // printf("\nCommand Header = %x.", commandHeader);
  *buffer = (commandHeader >> 8); 
  printf("\nCommand Header MSB= %x.\n", *buffer);
  *(buffer+1) = (commandHeader & 0x00FF);
  printf("Command Header LSB= %x.\n\n", *(buffer+1));
  while(TRUE){
    if (wiringPiSPIDataRW (SPI_CHAN, buffer, 2) == -1){
      return FALSE;
    }
    // printf("Write Success.\n");
    // delay(1000);
  }
  delay(delay_time);
  //Finished sending the character header.


  // Send data.
  *buffer = (regData >> 8); 
  printf("Data MSB= %x.\n", *buffer);
  *(buffer+1) = (regData & 0x00FF);
  printf("Data MSB= %x.\n", *(buffer+1));

  if (wiringPiSPIDataRW (SPI_CHAN, buffer, 2) == -1){
    return FALSE;
  }
  delay(delay_time);
  //Finished sending the data.

  //Pull CS line high to indication completion.
  digitalWrite (CS,HIGH);
  return TRUE;
}

//Work on 32-bit data later.
bool spiWrite32 (uint16_t regAddress, uint32_t regData){

  digitalWrite (CS, LOW);
  // delay(delay_time);

  *buffer = ((regAddress << 4) & 0xFFF0);
  if (wiringPiSPIDataRW (SPI_CHAN, buffer, 2) == -1){
    return FALSE;
  }
  delay(delay_time);

  *buffer = regData;
  if (wiringPiSPIDataRW (SPI_CHAN, buffer, 4) == -1){
    return FALSE;
  }
  delay(delay_time);

  digitalWrite (CS,HIGH);
  return TRUE;
}

uint16_t spiRead16 (uint16_t regAddress, uint16_t *dataStorage){
  printf("\n...Reading initiated...");
  
  unsigned char buffer[2];
  uint16_t commandHeader;

  digitalWrite (CS, LOW);



  //Send character header first.
  commandHeader = ((regAddress << 4 ) | 0x0008);
  printf("\nCommand Header = %x.", commandHeader);
  *buffer = (commandHeader >> 8);
  printf("\nCommand Header MSB= %x.\n", *buffer);
  *(buffer+1) = (commandHeader & 0x00FF);
  printf("Command Header LSB= %x.\n\n", *(buffer+1));
  if (wiringPiSPIDataRW (SPI_CHAN, buffer, 2) == -1){
    return FALSE;
  }
  delay(delay_time);
  //Finished sending the character header.


  
  if (wiringPiSPIDataRW (SPI_CHAN, buffer, 2) == -1){
    return FALSE;
  }
  delay(500);
  printf ("Data in buffer[0] = %x.\n", *buffer);
  printf ("Data in buffer[1] = %x.\n", *(buffer+1));
  *dataStorage = *buffer;
  *dataStorage = (*dataStorage << 8) + *(buffer+1);
  printf("Data returned = %x.\n", *dataStorage);
  delay(delay_time);
  //Finished reading the data.

  //Pull CS line high to indication completion.
  digitalWrite (CS,HIGH);
  return TRUE;
}

//Work on 32-bit data later.
void spiRead32 (uint16_t regAddress, uint32_t regData){
  uint16_t commandHeader = ((regAddress << 4) & 0xFFF0);
}

void spiFailure(void){
	printf ("SPI failure: %s\n", strerror (errno)) ;
}

int main(){
  static int fd;

  wiringPiSetup();
  pinMode(CS, OUTPUT);
  pinMode(IRQ1B, OUTPUT);
  

  //Reset ADE9078
  digitalWrite(IRQ1B, LOW);
  delay(50);
  digitalWrite(IRQ1B, HIGH);
  delay(50);
  printf("\nReset Done.");


  //Initial setup of the SPI bus in Raspberry Pi.
  printf("\nSetting the SPI bus...\n");
  if ((fd=wiringPiSPISetup(SPI_CHAN, SPEED)) < 0){
    fprintf (stderr, "Can't open the SPI bus: %s\n", strerror (errno)) ;
    exit (EXIT_FAILURE) ;
  }
  printf("Setup success. SPI initialized in channel %d.\n", SPI_CHAN);

  //Variables defined for SPI bus.
  uint16_t regAddress, spiData16;
  uint32_t spiData32;
  bool success = TRUE;
  //The chip select pin should be high when idle.
  //It needs to be pulled down when the device is selected for communication.
  // pinMode(CS, OUTPUT);
  // digitalWrite(CS, LOW);
  // delay(10);
  // digitalWrite(CS, HIGH);

  //Write operation
  regAddress = ADDR_PGA_GAIN; spiData16 = 0x1AB1;
  success = spiWrite16(regAddress, spiData16);
  if(!success){
	  spiFailure();
  }


  //Read operation
  uint16_t dataStorage;
  regAddress = ADDR_PGA_GAIN;
  success = spiRead16(regAddress, &dataStorage);
  if (!success){
    spiFailure();
  }
  printf("Data read = %x.\n", dataStorage);


  return 0;
}