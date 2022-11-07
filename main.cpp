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
    uint8_t* rtcm_data = (uint8_t*)malloc(MAXIMUM_RTCM_MESSAGE_LENGTH*MAXIMUM_RTCM_MESSAGES); //ideally this is dynamic but this should suffice
    uint16_t rtcm_len = 0;
    uint8_t* ubx_data = (uint8_t*)malloc(300);
    uint16_t ubx_len = 0;
    rtk_state state = RTK_IDLE;
    uint8_t loop = 1;
    
    uint8_t n = 0;

    led = 0;


    while(loop){

        switch(state){
            case(RTK_ERR):
                printf("something went wrong\n");
                ThisThread::sleep_for(1s);
                led = !led;
                
            break;
            case(RTK_IDLE):

                state = RTK_RECEIVE;
                led = 0;
                rtcm_len = 0;
                lora.setModeContRX();


            break;
            case(RTK_TRANSMIT): // transmit state

                
                if(lora.event_handler() == TX_DONE){
                    led = 0;
                    
                    lora.setModeContRX();
                    //printf("so far so good\n");
                    if(lora.n_payloads){
                        //printf("n=%d\n",lora.n_payloads);
                        state = RTK_RECEIVE;
                    } else {

                        memcpy(gps.rtcm_msg, rtcm_data, rtcm_len);
                        if(!gps.decode()){
                            printf("decode failed n = %d\n",gps.msg_ready(RTCM));
                        }
                        n = gps.msg_ready(RTCM);
                        printf("n = %d\n", n);
                        for(int i = 0; i < n; i++){
                        printf("len%d = %d,%d\n", i, gps.msg[i].length, gps.msg[i].type);
                        }
                        gps.clearAll();
                        state = RTK_SEND_RTCM_MSG;
                        /*
                        printf("rtcm msg = ");
                        for(int i = 0; i < rtcm_len; i++){
                            printf("%02x", rtcm_data[i]);
                        }
                        */
                        printf("\n\n");
                        
                        
                    }
                    //ThisThread::sleep_for(500ms);

                }

            break;
            case(RTK_RECEIVE): // receive state

                if(lora.event_handler() == RX_DONE){
                    //printf("rx_done\n");
                    led = 1;
                    lora.setModeIdle();
                    lora.receive(data, rx_len);
                    memcpy(rtcm_data+rtcm_len, data+5, rx_len-5);
                    rtcm_len += (rx_len-5);
                    lora.n_payloads = data[4];
                    data[0] = 0;
                    printf("n=%d, buf=%d, rx=%d\n", lora.n_payloads, rtcm_len, rx_len-5);
                    printf("some signal str: SNR=%d, RSSI=%d\n", lora.getSNR(),lora.getRSSI());
                    lora.flags = lora.flags | 0x01;
                    lora.setModeIdle();
                    if(!lora.transmit(data, 0)){ //transmit data
                        printf("transmit failed\n");
                        state = RTK_ERR;
                        break;
                    }
                    lora.flags = lora.flags & !0x01;
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
                
                if(gps.data_ready()){
                    
                    /*
                    printf("cpl = 0x");
                    print_hex((char*)gps.rtcm_msg, gps.rtcm_msg_pointer);
                    printf(" %d bytes\n", gps.rtcm_msg_pointer);
                    */

                    gps.decode();
                    gps.printMsgTypes();
                    gps.encode_RTCM(rtcm_data, rtcm_len);
                    //gps.encode_UBX(ubx_data, ubx_len);
                    char buf[200];
                    uint16_t l = 0;
                    gps.ubx[0].ubx2string(buf, l);
                    

                    printf("ubx = 0x");
                    //print_hex((char*)ubx_data, ubx_len);
                    printf(" %d bytes\n", ubx_len);
                    printf("rtcm = 0x");
                    //print_hex((char*)rtcm_data, rtcm_len);
                    printf(" %d bytes\n", rtcm_len);
                    



                    printf("total bytes from UART = %d\n", gps.rtcm_msg_pointer);
                    gps.clearAll();
                    /*
                    printf("rx = 0x");gps.getRtcmMsg(rtcm_data, rtcm_len)
                    print_hex((char*)rtcm_data, rtcm_len);
                    printf(" %d bytes\n", rtcm_len);
                    
                    print_hex((char*)gps.rtcm_msg,gps.rtcm_msg_pointer);
                    printf("\n\n");
                    //led = !led;
                    */

                    //maybe pack this in a function
                    lora.n_payloads = lora.get_n_payloads(rtcm_len);
                    if(lora.n_payloads){
                        tx_len = RH_RF95_MAX_MESSAGE_LEN;
                        rtcm_len -= tx_len;
                    } else {
                        tx_len = rtcm_len % RH_RF95_MAX_MESSAGE_LEN;
                        rtcm_len = 0;
                    }
                    printf("n=%d, tx=%d, buf=%d\n",lora.n_payloads, tx_len, rtcm_len);
                    /////function end
                    if(!lora.transmit(rtcm_data, tx_len)){ //transmit data
                        printf("transmit failed\n");
                        state = RTK_ERR;
                        break;
                    }
                    led = 1;
                    state = RTK_TRANSMIT;
                    
                } else{
                    //no message
                }
                

                if(gps.msg_pos == MSG_ERR){
                    printf("something went wrong\n");
                    state = RTK_ERR;
                }
                //ThisThread::sleep_for(200ms);
            break;
            case(RTK_SEND_RTCM_MSG):
                
                gps.writeCompleteMsg(rtcm_data, rtcm_len);
                state = RTK_RECEIVE;
                led = 0;
                rtcm_len = 0;
                lora.setModeContRX();

            break;
            default:
                printf("you shoudnt get this message\n");
                ThisThread::sleep_for(10s);
            break;
        }
        
    }
    printf("---program stop---\n");
    
    
    

}

