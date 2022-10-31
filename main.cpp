/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cstdint>
#include "mbed.h"
#include "LoRa_interface.h"
#include "GPS_interface.h"


//Hardware connections
#define CS_PIN PB_1
#define INT_PIN PC_4
//SPI
#define MOSI_PIN PB_15
#define MISO_PIN PB_14
#define SCLK_PIN PB_13


void print_hex(const char *s, int len)
{
    while(len--){
        printf("%02x", (uint8_t) *s++);
    }
}

typedef enum{
    RTK_IDLE,
    RTK_ERR,
    RTK_TRANSMIT,
    RTK_RECEIVE,
    RTK_GET_RTCM_MSG,
    RTK_SEND_RTCM_MSG

}rtk_state;






//BufferedSerial pc(USBTX, USBRX);
UnbufferedSerial uart(PA_0, PA_1, 921600);
BufferedSerial pc(USBTX, USBRX, 115200);
SPI spi(MOSI_PIN, MISO_PIN, SCLK_PIN);
//bool txFlag = 0;
DigitalOut led(LED1);

int main()
{
    printf("---programm start Rover---\n");

    // initalise serial spi ports
    spi.format(8, 0);
    spi.frequency(1000000);
    uart.format(8,SerialBase::None,1);
    

    // Create GPS object

    RTCM3_UBLOX gps(&uart);
    gps.init();

    // Create LoRa Object

    RFM95 lora(CS_PIN, INT_PIN, &spi);
    lora.init();


    uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(data);
    uint8_t rx_len = 0;
    uint8_t tx_len = 0;
    uint8_t* rtcm_data = (uint8_t*)malloc(MAXIMUM_BUFFER_SIZE*MAXIMUM_MESSAGES);
    uint16_t rtcm_len = 0;
    uint16_t rtcm_len2 = 0;
    rtk_state state = RTK_IDLE;
    uint8_t loop = 1;
    
    uint8_t n = 0;
    //uint8_t i = 0;
    
    uint8_t* buffer = NULL;
    uint16_t buf_len = MAXIMUM_BUFFER_SIZE*MAXIMUM_MESSAGES;
    buffer = (uint8_t*)calloc(buf_len, 1);


    led = 0;


    while(loop){

        switch(state){
            case(RTK_ERR):
                printf("something went wrong\n");
                ThisThread::sleep_for(1s);
                led = !led;
                
            break;
            case(RTK_IDLE):
                ThisThread::sleep_for(500ms);

                state = RTK_RECEIVE;
                led = 0;
                buf_len = 0;
                lora.setModeContRX();


            break;
            case(RTK_TRANSMIT): // transmit state

                
                if(lora.event_handler() == TX_DONE){
                    state = RTK_RECEIVE;
                    led = 0;
                    lora.setModeContRX();
                    //printf("so far so good\n");
                    if(lora.n_payloads){
                        state = RTK_RECEIVE;
                    } else {
                        state = RTK_IDLE;
                        for(int i = 0; i < buf_len; i++){
                            printf("%c", buffer[i]);
                        }
                        printf("\n");
                        
                    }
                    //ThisThread::sleep_for(500ms);

                }

            break;
            case(RTK_RECEIVE): // receive state

                if(lora.event_handler() == RX_DONE){
                    //printf("rx_done\n");
                    lora.setModeIdle();
                    lora.receive(data, rx_len);
                    memcpy(buffer+buf_len, data+4, rx_len);
                    buf_len += (rx_len-5);
                    lora.n_payloads = data[4];
                    buffer[0] = 0;
                    printf("n=%d, buf=%d, rx=%d\n", lora.n_payloads, buf_len, rx_len-5);
                    lora.flags = lora.flags | 0x01;
                    if(!lora.transmit(buffer, 0)){ //transmit data
                        printf("transmit failed\n");
                        state = RTK_ERR;
                        break;
                    }
                    lora.flags = lora.flags & !0x01;
                    led = 1;
                    state = RTK_TRANSMIT;
                    //ThisThread::sleep_for(500ms);
                    /*
                    if(!lora.transmit(buffer, sizeof(buffer))){ //transmit data
                        printf("transmit failed\n");
                        state = RTK_ERR;
                        break;
                    }
                    */
                }

            break;
            case(RTK_GET_RTCM_MSG): //read uart
                //printf("congrats! no crash");
                
                if(gps.readCompleteMsg(rtcm_data, rtcm_len)){
                    printf("rx = 0x");
                    print_hex((char*)rtcm_data, rtcm_len);
                    printf(" %d bytes\n", rtcm_len);
                    
                    print_hex((char*)gps.rtcm_msg,gps.rtcm_msg_pointer);
                    printf("\n\n");
                    //led = !led;
                } else{
                    //no message
                }
                

                if(gps.msg_pos == MSG_ERR){
                    printf("something went wrong\n");
                    state = RTK_ERR;
                }
                //ThisThread::sleep_for(200ms);
            break;
            default:
                printf("you shoudnt get this message\n");
                ThisThread::sleep_for(10s);
            break;
        }
        
    }
    printf("---program stop---\n");
    
    
    

}

