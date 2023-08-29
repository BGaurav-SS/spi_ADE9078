/*  This example application illustrates the use of spiADE9078 library.
    The ADE9078 datasheet recommends to use 0x117514 for VLEVEL register.
    In writing process, this value is written to the specified register.
    In reading process, the same register is read back.

    Note that the VLEVEL register has [31:24] bits reserved. These are set 0 during reset and are read-only.
    So, these bits always read 0.
*/

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <wiringPi.h>
#include "spiADE9078.h"

//Use wiringPi pin 6; physical pin 22 to send command to reset the IC.
#define RESET_PIN   6
//Use wiringPi pin 3; physical pin 15 as pin to read interrupt from IRQ1B pin.
#define IRQ1B_PIN   3

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

        // printf ("Writing data\n\n");
        if((writeByte (ADDR_VLEVEL, 0x117514) < 0)){
            return -1;
        }
        // delay(10);


        printf ("Receiving data\n");
        if ((readByte(ADDR_VLEVEL, &data)) < 0){
            return -1 ;
        }
        printf("RECEIVED: %.2X\n\n",data);
        // delay(10);
    }
    return 0;
}