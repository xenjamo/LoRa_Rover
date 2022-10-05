/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cstdint>
#include "mbed.h"
#include "LoRa_interface.h"


//Hardware connections
#define CS_PIN PB_1
#define INT_PIN PA_8
//SPI
#define MOSI_PIN PB_15
#define MISO_PIN PB_14
#define SCLK_PIN PB_13

//BufferedSerial pc(USBTX, USBRX);
BufferedSerial pc(USBTX, USBRX, 115200);
SPI spi(MOSI_PIN, MISO_PIN, SCLK_PIN);
bool txFlag = 0;
DigitalOut led(LED1);

int main()
{
    printf("---programm start---\n");
    // initalise spi port

    spi.format(8, 0);
    spi.frequency(1000000);

    // Create LoRa Object
    RFM95 lora(CS_PIN, INT_PIN, &spi);
    // initalise LoRa
    lora.init();
    lora.setModeContRX();
    uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(data);

    uint8_t loop = 1;
    while(loop){
        if(lora.event_handler() == RX_DONE){
            lora.receive(data, &len);
            printf("recieved a message:\n");
            printf("%s\n\n",(char*) data);
        }

        
    }
    printf("---program stop---\n");
    
    
    

}

