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

//Use wiringPi pin 6; physical pin 22 as reset pin.
#define IRQ1B   6

static char *spiDevice = "/dev/spidev0.0" ;
static uint8_t spiMode = 1 ;
static uint8_t spiBPW = 8 ;
static uint32_t spiSpeed = 500000 ;
static uint16_t spiDelay = 0;


int spi_fd;

void reset (void){
    //Reset ADE9078
    digitalWrite(IRQ1B, LOW);
    delay(50);
    digitalWrite(IRQ1B, HIGH);
    delay(50);
    printf("\nReset Done.");
}



static uint32_t readByte (uint16_t reg){

    //Trasmitter and receiver buffer.
    uint8_t spiBufTx [4] ;
    uint8_t spiBufRx [4] ;

    struct spi_ioc_transfer spi ;
    //First byte of the command header.
    //It is the MSB of the address .
    *(spiBufTx) = (reg >> 4);

    //Second byte of the command header.
    //The last 4-bits of address.
    //Read/write selection bit.
    //Remaining 4 bits are ignored.
    *(spiBufTx+1) = (((reg << 4) & 0x00F0)) | 0x0008;


    spi.tx_buf = (unsigned long)spiBufTx ;
    spi.rx_buf = (unsigned long)spiBufRx ;
    spi.len = 4 ;
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



static void writeByte (uint16_t reg, uint16_t data){

    //Transmit and Receiver Buffer.
    uint8_t spiBufTx [4] ;
    uint8_t spiBufRx [4] ;

    struct spi_ioc_transfer spi ;
    
    //First byte of the command header.
    //It is the MSB of the address .
    *(spiBufTx) = (reg >> 4);

    //Second byte of the command header.
    //The last 4-bits of address.
    //Read/write selection bit.
    //Remaining 4 bits are ignored.
    *(spiBufTx+1) = ((reg << 4) & 0x00F0);

    //First byte of the data.
    *(spiBufTx+2) = (data >> 8);

    //Second byte of the data.
    *(spiBufTx+3) = (data & 0x00FF);


    spi.tx_buf = (unsigned long)spiBufTx ;

    spi.rx_buf = (unsigned long)spiBufRx ;
    spi.len = 4 ;
    spi.delay_usecs = spiDelay ;
    spi.speed_hz = spiSpeed ;
    spi.bits_per_word = spiBPW ;
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
    pinMode(IRQ1B, OUTPUT);

    reset();

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
        uint16_t data = 0x12AB;
        printf ("Sending data %x to address %x. \n", data, ADDR_PGA_GAIN);
        writeByte (ADDR_PGA_GAIN, data) ;
        delay(10);

        
        printf ("\nReceiving data\n");
        data = readByte (ADDR_PGA_GAIN) ;
        // printf("RECEIVED: %.2X\n",data);
        //close(spi_fd);
        delay(1);
    }
    return 0;

}