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
    printf("---programm start Rover---\n");
    // initalise spi port

    spi.format(8, 0);
    spi.frequency(1000000);

    // Create LoRa Object
    RFM95 lora(CS_PIN, INT_PIN, &spi);
    // initalise LoRa
    lora.init();
    uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(data);
    int state = 2;
    uint8_t loop = 1;
    
    uint8_t buf[] = {"Hello World! PONG!"};
    led = 0;
    lora.setModeContRX();

    while(loop){
        
        switch(state){
            case 0:
                printf("something went wrong\n");
                ThisThread::sleep_for(2s);
                
            break;
            case 1: // transmit state

                
                if(lora.event_handler() == TX_DONE){
                    state = 2;
                    led = 0;
                    lora.setModeContRX();
                    ThisThread::sleep_for(500ms);
                    
                    
                }

            break;
            case 2: // receive state
                if(lora.event_handler() == RX_DONE){
                    lora.setModeIdle();
                    lora.receive(data, len);
                    printf("%s\n",data);
                    state = 1;
                    led = 1;
                    ThisThread::sleep_for(500ms);
                    if(!lora.transmit(buf, sizeof(buf))){ //transmit data
                        printf("transmit failed\n");
                        state = 0;
                        break;
                    }
                }

            break;
        }
        ThisThread::sleep_for(100ms);
        
    }
    printf("---program stop---\n");
    
    
    

}