/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cstdint>
#include "mbed.h"
#include "LoRa_interface.h"
#include "GPS_interface.h"
#include "SD_interface.h"
#include "LSM9DS1.h"


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
    RTK_SEND_RTCM_MSG,
    RTK_GET_UBX_MSG,

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
    
    //init SD card
    
    int _fakeint = 2; //dont ask please
    SDCARD sd(_fakeint);
    if(!sd.init()){
        printf("SD init failed\n"); //if this fails all operations will be ignored(in case you wanna use it without sd card)
    }
    // define a header to know what values go where
    char rover_header[] = "itow[ms];carrSoln;lon;lat;height[m];x[mm];y[mm];z[mm];hAcc[mm];vAcc[mm];LoRa_valid;SNR;RSSI;ax;az;az;gx;gy;gz;\n";
    sd.write2sd(rover_header,sizeof(rover_header));

    // Create GPS object

    RTCM3_UBLOX gps(&uart);
    gps.init();

    // Create LoRa Object

    RFM95 lora(CS_PIN, INT_PIN, &spi);
    lora.init();

    //  IMU stuff
    LSM9DS1 imu(PC_9, PA_8, 0xD6, 0x3C);
    if (!imu.begin()) {
        printf("Failed to communicate with LSM9DS1.\r\n");
    }


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

    lora.setModeContRX();


    while(loop){

        switch(state){
            case(RTK_ERR):
                printf("something went wrong\n");
                ThisThread::sleep_for(1s);
                led = !led;
                
            break;
            case(RTK_IDLE):
                led = 0;
                if(lora.event_handler() == RX_DONE){
                    //printf("rx_done\n");
                    led = 1;
                    lora.setModeIdle();
                    lora.receive(data, rx_len);

                    /*
                    printf("rtcm msg = \n");
                    for(int i = 0; i < rx_len; i++){
                        printf("%02x", data[i]);
                    }
                    printf("\n");
                    */

                    memcpy(rtcm_data+rtcm_len, data+5, rx_len-5);
                    rtcm_len += (rx_len-5);
                    lora.n_payloads = data[4];
                    data[0] = 0;
                    //printf("n=%d, buf=%d, rx=%d\n", lora.n_payloads, rtcm_len, rx_len-5);
                    //printf("some signal str: SNR=%d, RSSI=%d\n", lora.getSNR(),lora.getRSSI());
                    lora.flags = lora.flags | 0x01;
                    lora.setModeIdle();
                    if(!lora.transmit(data, 0)){ //transmit data
                        printf("transmit failed\n");
                        state = RTK_ERR;
                        break;
                    }
                    lora.flags = lora.flags & !0x01;
                    state = RTK_TRANSMIT;
                    
                }
                
                if(gps.data_ready()){
                    // read IMU data
                    imu.readAccel();
                    imu.readGyro();

                    led = 1;
                    gps.decode();

                    bool signal_valid = lora.signal_valid();
                    int8_t snr = lora.getSNR();
                    uint8_t rssi = lora.getRSSI();

                    char buf[400];
                    uint16_t l = 0;
                    int i = 0;
                    l = sprintf(buf, "%d;%d;", gps.itow, gps.rtk_stat)+1;
                    sd.write2sd(buf, l);
                    l = sprintf(buf, "%.9f;%.9f;%.5f;", gps.lon, gps.lat, gps.height)+1;
                    sd.write2sd(buf, l);
                    l = sprintf(buf, "%.2f;%.2f;%.2f;", gps.rel_x, gps.rel_y, gps.rel_z)+1;
                    sd.write2sd(buf, l);
                    l = sprintf(buf, "%.2f;%.2f;", gps.hAcc, gps.vAcc);
                    sd.write2sd(buf, l);
                

                    //the accual data
                    /* // all data
                    l = 0;
                    i = 0;
                    while(gps.ubx[i].isvalid){
                        if(gps.ubx[i].ubx2string(buf, l)){
                            sd.write2sd(buf, l);
                        } else {
                            printf("no ubx[%d]\n", i);
                        }
                        i++;
                    }
                    */
                    gps.clearAll();

                    l = sprintf(buf, "%d;%d;%d;",signal_valid, snr, rssi)+1;
                    sd.write2sd(buf, l);

                    //IMU data
                    l = sprintf(buf, "%.5f;%.5f;%.5f;",imu.accX,imu.accY,imu.accZ)+1;
                    sd.write2sd(buf, l);
                    l = sprintf(buf, "%.5f;%.5f;%.5f;",imu.gyroX,imu.gyroY,imu.gyroZ)+1;
                    sd.write2sd(buf, l);
                    
                    sd.writeln();
                    
                    
                }
                


            break;
            case(RTK_TRANSMIT): // transmit state

                
                if(lora.event_handler() == TX_DONE){
                    
                    lora.setModeContRX();
                    //printf("so far so good\n");
                    if(lora.n_payloads){
                        //printf("n=%d\n",lora.n_payloads);
                        state = RTK_IDLE;
                    } else {

                        memcpy(gps.rtcm_msg, rtcm_data, rtcm_len);
                        if(!gps.decode()){
                            printf("decode failed n = %d\n",gps.msg_ready(RTCM));
                        }
                        n = gps.msg_ready(RTCM);
                        printf("n = %d\n", n);
                        for(int i = 0; i < n; i++){
                        printf("rtcm%d n bytes = %d, type = %d\n", i, gps.msg[i].length + 6, gps.msg[i].type);
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
            case(RTK_SEND_RTCM_MSG):
                
                gps.writeCompleteMsg(rtcm_data, rtcm_len);
                rtcm_len = 0;
                state = RTK_IDLE;
                gps.clearAll();

            break;
            default:
                printf("you shoudnt get this message\n");
                ThisThread::sleep_for(10s);
            break;
        }
        
    }
    printf("---program stop---\n");
    
    
    

}

