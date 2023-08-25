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
#include <errno.h>
#include <stdint.h>
#include "registerMap.h"

#include <wiringPi.h>

//Use wiringPi pin 6; physical pin 22 to send command to reset the IC.
#define RESET_PIN   6
//Use wiringPi pin 3; physical pin 15 as pin to read interrupt from IRQ1B pin.
#define IRQ1B_PIN   3

static uint8_t spiMode = SPI_MODE_3;
static uint8_t spiBPW = 8 ;
static uint32_t spiSpeed = 1000000 ; 
static uint16_t spiDelay = 0;
static uint8_t spiLSBFirst = 0;


static uint32_t readByte(uint16_t reg, uint32_t* data);
static uint32_t writeByte (uint32_t reg, uint32_t data);
int spi_open(char* dev);
int initialize (void);

int spi_fd;


int spi_open(char* dev){
    if((spi_fd = open(dev, O_RDWR)) < 0){
        printf("error opening %s\n",dev);
        return -1;
    }

    if(ioctl(spi_fd, SPI_IOC_WR_MODE, &spiMode)){
        printf("Error setting SPI mode %d.\n", spiMode);
        return -1;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_LSB_FIRST, &spiLSBFirst) == -1) {
        printf("Error setting MSB-first bit order\n");
        return -1;
    }
    return 0;
}


int initialize (void){
    //Reset ADE9078
    digitalWrite(RESET_PIN, LOW);
    delay(5);
    digitalWrite(RESET_PIN, HIGH);
    delay(5);

    printf("\nWaiting for RESET_DONE signal.\n");    
    //Wait until the RESET_DONE signal is generated.
    while (digitalRead(IRQ1B_PIN) != 0){}
    printf("RESET DONE.\n");
    return 0;
}



static uint32_t readByte(uint16_t reg, uint32_t* data){
    
    int error, numberOfBytes = 0;
    *data = 0;
    
    //16-bit registers: 0x480 to 0x4FE.
    if (reg >= 0x480 && reg <=0x4FE){
        numberOfBytes = 4; //8*4 = 32; 16-bits for command-header, 16-bits for data.
    }

    else{
        numberOfBytes = 6; //8*6 = 48; 16-bits for command-header, 32-bits for data.
    }

    struct spi_ioc_transfer spi = {0};

    uint8_t spiBufTx [numberOfBytes] ;
    uint8_t spiBufRx [numberOfBytes] ;
    int fillCounter=numberOfBytes;


    //First byte of the command header.
    //It is the MSB of the address.
    *(spiBufTx) = (reg >> 4);

    //Second byte of the command header.
    //The last 4-bits of address.
    //Read/write selection bit.
    //Remaining 4 bits are ignored.
    *(spiBufTx+1) = (((reg << 4) & 0x00F0)) | 0x0008;

    //Remaining slots in tx-buffer is filled with zeros.
    for (fillCounter=numberOfBytes; fillCounter > 2; fillCounter--){
        *(spiBufTx+(fillCounter-1)) = 0;
    }

    spi.tx_buf = (unsigned long)spiBufTx ;
    spi.rx_buf = (unsigned long)spiBufRx ;
    spi.len = numberOfBytes ;
    spi.delay_usecs = spiDelay ;
    spi.speed_hz = spiSpeed ;
    spi.bits_per_word = spiBPW ;

    if ((error = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi)) < 0){
        printf("Failed: Writing into register 0x%x.\n", reg);
        printf("Error: %s\n\n", strerror(errno));
        return -1;
    }
    // printf ("SPI receiver buffer B1= %x\n", *(spiBufRx + 2));
    // printf ("SPI receiver buffer B2= %x\n\n", *(spiBufRx + 3));
    // printf ("SPI receiver buffer B3= %x\n\n", *(spiBufRx + 4));
    // printf ("SPI receiver buffer B4= %x\n\n", *(spiBufRx + 5));

    for (fillCounter=2; fillCounter <numberOfBytes; fillCounter++){
        *data = *data << 8;
        *data += *(spiBufRx  + fillCounter);
    }   
    
    return 0;
}



static uint32_t writeByte (uint32_t reg, uint32_t data){

    int numberOfBytes = 0;
    int error;

    //16-bit registers: 0x480 to 0x4FE.
    if (reg >= 0x480 && reg <=0x4FE){
        //8-bits*4 = 32-bits; 16-bits for command-header, 16-bits for data.
        numberOfBytes = 4; 
    }

    else{
        //8-bits*6 = 48-bits; 16-bits for command-header, 32-bits for data.
        numberOfBytes = 6; 
    }
    
    struct spi_ioc_transfer spi = {0} ;

    uint8_t spiBufTx [numberOfBytes] ;
    uint8_t spiBufRx [numberOfBytes] ;
    int fillCounter = numberOfBytes;

    
    //First byte of the command header.
    //It is the MSB of the address.
    *(spiBufTx) = (reg >> 4);

    //Second byte of the command header.
    //The last 4-bits of address.
    //Read/write selection bit.
    //Remaining 4 bits are ignored.
    *(spiBufTx+1) = ((reg << 4) & 0x00F0);

    //The transmit buffer is filled in reverse. 
    //If 16-bit data is stored in 32-bit DATA variable, the first 16-bits are 0.
    //Actual data is in the last 16-bits.
    for (fillCounter=numberOfBytes; fillCounter > 2; fillCounter--){
        //printf("Data %x\n", data);
        *(spiBufTx+(fillCounter-1)) = data & (0xFF);
        //printf("buffer %x\n", *(spiBufTx+(fillCounter-1)));
        data = (data >> 8);
    }

    spi.tx_buf       = (unsigned long)spiBufTx ;
    spi.rx_buf       = (unsigned long)spiBufRx ;
    spi.len          = numberOfBytes ;
    spi.delay_usecs  = spiDelay ;
    spi.speed_hz     = spiSpeed ;
    spi.bits_per_word= spiBPW ;

    if ((error = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi)) < 0){
        printf("Failed: Writing into register 0x%x.\n", reg);
        printf("Error: %s\n\n", strerror(errno));
        return -1;
    }
    
    // printf ("spi = %ul\n", &spi);
    // printf ("REg= %x\n", reg);
    // printf ("SPI transmit buffer B1= %x\n", *(spiBufTx + 2));
    // printf ("SPI transmit buffer B2= %x\n", *(spiBufTx + 3));
    // printf ("SPI transmit buffer B3= %x\n", *(spiBufTx + 4));
    // printf ("SPI transmit buffer B4= %x\n", *(spiBufTx + 5));

    return 0;
}


int main(int argc, char* argv[]){

    uint32_t data;

    if(argc <= 1){
        printf("Too few args, try %s /dev/spidev0.0\n",argv[0]);
        return -1;
    }

    wiringPiSetup();
    pinMode(RESET_PIN, OUTPUT);
    pinMode(IRQ1B_PIN, INPUT);


    if (initialize() != 0){
        printf("Error initializing the device.\n");
        return -1;
    }
    printf("Success: Device Initialization.\n\n");


    // open and configure SPI channel. (/dev/spidev0.0 for example)
    printf("Opening SPI port...\n");
    if(spi_open(argv[1]) < 0){
        printf("SPI_open failed\n");
        return -1;
    }
    printf("Success: Opening SPI port.\n\n");
    delay(1000);

    // Turning the IRQ1B LED off.
    if(writeByte (ADDR_STATUS1, (1<<16)) < 0){
        return -1;
    }

    while (1){

        printf ("Writing data\n\n");
        if((writeByte (ADDR_PGA_GAIN, 0xF0F0) < 0)){
            return -1;
        }
        delay(10);

        printf ("Receiving data\n");
        readByte(ADDR_PGA_GAIN, &data);
        printf("RECEIVED: %.2X\n\n",data);

        //close(spi_fd);
        delay(10);
    }
    return 0;
}

