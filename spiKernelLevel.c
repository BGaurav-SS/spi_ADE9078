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

#include <wiringPi.h>

//Use wiringPi pin 6; physical pin 22 to send command to reset the IC.
#define RESET_PIN   6
//Use wiringPi pin 3; physical pin 15 as pin to read interrupt from IRQ1B pin.
#define IRQ1B_PIN   3

static char *spiDevice = "/dev/spidev0.0" ;
static uint8_t spiMode = 1 ;
static uint8_t spiBPW = 8 ;
static uint32_t spiSpeed = 500000 ;
static uint16_t spiDelay = 0;


int spi_fd;

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



static uint32_t readByte(uint16_t reg, int* data){
    int numberOfBytes = 0;
    
    //16-bit registers: 0x480 to 0x4FE.
    if (reg >= 0x480 && reg <=0x4FE){
        numberOfBytes = 4; //8*4 = 32; 16-bits for command-header, 16-bits for data.
    }

    else{
        numberOfBytes = 6; //8*6 = 48; 16-bits for command-header, 32-bits for data.
    }

    uint8_t spiBufTx [numberOfBytes] ;
    uint8_t spiBufRx [numberOfBytes] ;


    struct spi_ioc_transfer spi;
    //First byte of the command header.
    //It is the MSB of the address.
    *(spiBufTx) = (reg >> 4);

    //Second byte of the command header.
    //The last 4-bits of address.
    //Read/write selection bit.
    //Remaining 4 bits are ignored.
    *(spiBufTx+1) = (((reg << 4) & 0x00F0)) | 0x0008;


    spi.tx_buf = (unsigned long)spiBufTx ;
    spi.rx_buf = (unsigned long)spiBufRx ;
    spi.len = numberOfBytes ;
    spi.delay_usecs = spiDelay ;
    spi.speed_hz = spiSpeed ;
    spi.bits_per_word = spiBPW ;
    ioctl (spi_fd, SPI_IOC_MESSAGE(1), &spi);

    printf ("\nSPI receiver buffer B1= %x\n", *spiBufRx);
    printf ("SPI receiver buffer B2= %x\n", *(spiBufRx + 1));
    printf ("SPI receiver buffer B3= %x\n", *(spiBufRx + 2));
    printf ("SPI receiver buffer B4= %x\n\n", *(spiBufRx + 3));

    return *(spiBufRx + 1) ;
}



static void writeByte (uint16_t reg, uint32_t data){

    int numberOfBytes = 0;
    
    //16-bit registers: 0x480 to 0x4FE.
    if (reg >= 0x480 && reg <=0x4FE){
        numberOfBytes = 4; //8*4 = 32; 16-bits for command-header, 16-bits for data.
    }

    else{
        numberOfBytes = 6; //8*6 = 48; 16-bits for command-header, 32-bits for data.
    }

    uint8_t spiBufTx [numberOfBytes] ;
    uint8_t spiBufRx [numberOfBytes] ;
    int fillCounter = numberOfBytes;

    struct spi_ioc_transfer spi ;
    
    //First byte of the command header.
    //It is the MSB of the address .
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
        *(spiBufTx+(numberOfBytes-1)) = data & (0xFF);
        data = (data >> 8);        
    }
    

    // //MSB byte of the data.
    // *(spiBufTx+2) = (data >> 8);

    // //Second byte of the data.
    // *(spiBufTx+3) = (data & 0x00FF);


    spi.tx_buf       = (unsigned long)spiBufTx ;
    spi.rx_buf       = (unsigned long)spiBufRx ;
    spi.len          = numberOfBytes ;
    spi.delay_usecs  = spiDelay ;
    spi.speed_hz     = spiSpeed ;
    spi.bits_per_word= spiBPW ;
    ioctl (spi_fd, SPI_IOC_MESSAGE(1), &spi) ;
    // printf ("spi = %ul\n", &spi);
    // printf ("REg= %x\n", reg);
    // printf ("SPI transmit buffer B1= %x\n", *spiBufTx);
    // printf ("SPI transmit buffer B2= %x\n", *(spiBufTx + 1));
    // printf ("SPI transmit buffer B3= %x\n", *(spiBufTx + 2));
    // printf ("SPI transmit buffer B4= %x\n", *(spiBufTx + 3));


    //Do data is obtained in the receiver buffer.
//     printf ("SPI receiver buffer B1= %x\n", *spiBufRx);
//     printf ("SPI receiver buffer B2= %x\n", *(spiBufRx + 1));
//     printf ("SPI receiver buffer B3= %x\n", *(spiBufRx + 2));
//     printf ("SPI receiver buffer B4= %x\n", *(spiBufRx + 3));
}



/*spi_open
* - Open the given SPI channel and configures it.
* - there are normally two SPI devices on your PI:
* /dev/spidev0.0: activates the CS0 pin during transfer
* /dev/spidev0.1: activates the CS1 pin during transfer
*
*/
int spi_open(char* dev){
    if((spi_fd = open(dev, O_RDWR)) < 0){
    printf("error opening %s\n",dev);
    return -1;
    }
    return 0;
}


int main(int argc, char* argv[]){

    wiringPiSetup();
    pinMode(RESET_PIN, OUTPUT);
    pinMode(IRQ1B_PIN, INPUT);

    if (initialize() != 1){
        printf("Error initializing the device.\n");
        return -1;
    }

    if(argc <= 1){
        printf("Too few args, try %s /dev/spidev0.0\n",argv[0]);
        return -1;
    }
    // open and configure SPI channel. (/dev/spidev0.0 for example)
    if(spi_open(argv[1]) < 0){
        printf("SPI_open failed\n");
        return -1;
    }


    while (1){
        uint16_t address = ADDR_PGA_GAIN;
        uint32_t data = 0x12AB;
        printf ("Sending data %x to address %x. \n", data, address);
        writeByte (address, data) ;
        delay(10);

        
        printf ("\nReceiving data\n");
        data = readByte(address, &data);
        // printf("RECEIVED: %.2X\n",data);
        //close(spi_fd);
        delay(1);
    }
    return 0;

}