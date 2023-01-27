#ifndef RST265_THREAD_H_
#define RST265_THREAD_H_

#include "mbed.h"
#include "data_structs.h"
#include "Unwrapper_2pi.h"
#include "ThreadFlag.h"
#include "Dense.h"
#include <mutex>

#include "LoRa_interface.h"
#include "GPS_interface.h"
#include "SD_interface.h"
#include "LSM9DS1.h"


using namespace std;
using namespace Eigen;


// "protocol" specifics
#define TERMINATOR          // somehow need to signale the end the transmision
#define BUFFER_LEN2      100  // max 256


#define LEN_OF_EXP_TYPE 4   // length in bytes of expected Type
#define NUM_OF_VALUE    7   // number of expected values


// predefiniton for callback (couldn't implement as member)

class RST265_thread
{
public:
// public members
    RST265_thread(BufferedSerial *,DATA_Xchange *,float);
    //RST265_thread(DATA_Xchange *,float);
    virtual ~RST265_thread();
    void request();         // request new set of data
    void start_RST265(void);

    // public vars
    Matrix <float, 3, 1> L0;
    float yaw_0;
    // public vars
    bool dataValid;         // is the data in Values valid?
    float values[NUM_OF_VALUE];     // stores the received/parsed values
    void quat2rotm(Matrix<float,4,1>,  Matrix<float,3,3> *);
    float quat2yaw_angle(Matrix<float,4,1>);
    void reset_RS_z0(void);
    DATA_Xchange *m_data;
    void setImuPtr(LSM9DS1 *_rs_imu);
    void setLoraPtr(RFM95 *_rs_lora);
    void setGpsPtr(RTCM3_UBLOX *_rs_gps);
    int _fakeint = 0;
    LSM9DS1 *rs_imu;
    RFM95 *rs_lora;
    RTCM3_UBLOX *rs_gps;
    SDCARD sd;
    
private:
    
    // private members
    void sendCmd(char);     // sends comand to device
    void run(void);             // runs the statemachine, call this function periodicly, returns true when new data ready (only true for 1 cycle)
    void callBack();        // ISR for storing serial bytes
    void init();            // re initializes the buffers and the statemachine
    float Ts;
    float phi, theta, psi;
    Unwrapper_2pi RS_yaw_unwrap;
    Matrix<float,4,1> i_quat0;
    void  quat2rpy(float qw, float qx, float qy, float qz, float *phi, float *theta, float *psi);
    Matrix<float,4,1> quatproduct(Matrix<float,4,1>,Matrix<float,4,1>);
    void quat_conj(Matrix<float,4,1>, Matrix<float,4,1> *);
    Thread thread;
    Ticker ticker;
	ThreadFlag threadFlag;
    void sendSignal();
    bool R0_is_set;
	//DigitalOut dout1;
// -------------------
    uint8_t buffer[BUFFER_LEN2];     // RX buffer
    uint8_t buffCnt;            // max 255
    uint8_t state;              // statemachine state variable
    BufferedSerial *uart;   // pointer to uart for communication with device
    float RS_z0;
    Mutex m_mutex;
};

#endif

// Example
/*
RawSerial pc(USBTX, USBRX);
RawSerial rsUart(PC_12, PD_2, 115200);
RST265 rsdev(&rsUart);

DigitalIn button(USER_BUTTON);

int main() {
    pc.printf("\n\rSerial Test...%d\n", 115200);

    bool pushed = false;
    while(1) {
        // register button pressed and request data
        if(!button && !pushed){
            pc.printf("\r\nsending request..");
            pushed = true;
            rsdev.request();
        }else if (button && pushed){
            // reset button
            pushed = false;
        }

        if(rsdev.run()){
            // new data ready
            pc.printf("\r\nreceived response");
            for(int i = 0; i < 3; i++){
                pc.printf("\n\r\tvalue %i: %f",i, rsdev.values[i]);
            }
        }
    }
}
*/
