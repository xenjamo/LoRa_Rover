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

#include "data_structs.h"
#include "RST265_thread.h"


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
BufferedSerial rpi0_Uart(PC_6,PC_7,115200);

//bool txFlag = 0;
DigitalOut led(LED1);
LSM9DS1 imu(PC_9, PA_8, 0xD6, 0x3C);

int main()
{
    printf("---programm start Rover---\n");

    //block that initiats realsense
    ThisThread::sleep_for(chrono::milliseconds(100));
    //------------samping rates
    float Ts25 = .04;
    rpi0_Uart.set_format(8,BufferedSerial::None,1);
    rpi0_Uart.set_blocking(false);
    //----------------------------------
    DATA_Xchange *rs_data = new DATA_Xchange;

    RST265_thread t265(&rpi0_Uart,rs_data,Ts25);
    ThisThread::sleep_for(chrono::milliseconds(200));
    
    // end init

    // initalise serial spi ports
    SPI spi(MOSI_PIN, MISO_PIN, SCLK_PIN);
    spi.format(8, 0);
    spi.frequency(10000000);
    uart.format(8,SerialBase::None,1);
    
    //init SD card
    /*
    int _fakeint = 2; //dont ask please
    SDCARD sd(_fakeint);
    if(!sd.init()){
        printf("SD init failed\n"); //if this fails all operations will be ignored(in case you wanna use it without sd card)
    }
    // define a header to know what values go where
    char rover_header[] = "itow[ms];carrSoln;lon;lat;height[m];x[mm];y[mm];z[mm];hAcc[mm];vAcc[mm];LoRa_valid;SNR;RSSI;ax;az;az;gx;gy;gz;\n";
    sd.write2sd(rover_header,sizeof(rover_header));
    */
    // Create GPS object

    RTCM3_UBLOX gps(&uart);
    gps.init();

    // Create LoRa Object

    RFM95 lora(CS_PIN, INT_PIN, &spi);
    lora.init();

    //  IMU stuff
    
    if (!imu.begin()) {
        //printf("Failed to communicate with LSM9DS1.\n"); // seems there is a bug with this library where the whoAmI registeres are read as 0x00
    }
    

    //add these to the realsense thread;
    t265.setGpsPtr(&gps);
    t265.setLoraPtr(&lora);
    t265.setImuPtr(&imu);


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
    bool imu_valid = 0;

    led = 0;

    lora.setModeContRX();
    printf("starting in 1s\n");
    ThisThread::sleep_for(1s);
    t265.start_RST265();


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
                if((gps.msg_pos == MSG_DATA) && !imu_valid){
                    //imu.readAccel();
                    //imu.readGyro();
                    //printf("%.5f;%.5f;%.5f;\n",imu.accX,imu.accY,imu.accZ);
                    imu_valid = true;
                }
                
                if(gps.data_ready()){
                    led = 1;
                    //pack this in the data ready function
                    gps.decode();
                    printf("\n\nl = %d\n",gps.rtcm_msg_length);
                    /*
                    bool signal_valid = lora.signal_valid();
                    int8_t snr = lora.getSNR();
                    int8_t rssi = lora.getRSSI();
                    */

                    /*
                    char buf[400];
                    uint16_t l = 0;
                    int i = 0;
                    */

                    /*
                    l = sprintf(buf, "%d;%d;", gps.itow, gps.rtk_stat)+1;
                    sd.write2sd(buf, l);
                    l = sprintf(buf, "%.9f;%.9f;%.5f;", gps.lon, gps.lat, gps.height)+1;
                    sd.write2sd(buf, l);
                    l = sprintf(buf, "%.2f;%.2f;%.2f;", gps.rel_x, gps.rel_y, gps.rel_z)+1;
                    sd.write2sd(buf, l);
                    l = sprintf(buf, "%.2f;%.2f;", gps.hAcc, gps.vAcc);
                    sd.write2sd(buf, l);
                    */

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
                    /*
                    printf("%d;%d;%d;\n",signal_valid, snr, rssi);
                    l = sprintf(buf, "%d;%d;%d;",signal_valid, snr, rssi)+1;
                    sd.write2sd(buf, l);
                    */

                    

                    //IMU data
                    /*
                    l = sprintf(buf, "%.5f;%.5f;%.5f;",imu.accX,imu.accY,imu.accZ)+1;
                    sd.write2sd(buf, l);
                    //printf("%.5f;%.5f;%.5f;\n",imu.gyroX,imu.gyroY,imu.gyroZ);
                    l = sprintf(buf, "%.5f;%.5f;%.5f;",imu.gyroX,imu.gyroY,imu.gyroZ)+1;
                    sd.write2sd(buf, l);
                    imu_valid = false;
                    */


                    //sd.writeln();
                    
                    
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
                        /*
                        printf("rtcm_lora = ");
                        print_hex((char*)rtcm_data, rtcm_len);
                        printf("\n");
                        */
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
            case(RTK_RECEIVE): // receive state =========IGNORE THIS STATE!!! ITS NOT USED CURRENTLY

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
                //printf("here");
                gps.writeCompleteMsg(rtcm_data, rtcm_len);

                /* 
                printf("rtcm_lora = ");
                print_hex((char*)rtcm_data, rtcm_len);
                printf("\n");
                */
                
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

