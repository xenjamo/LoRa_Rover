// includes
#include "RST265_thread.h"

#define NO_OUTPUT


// #### constructor
RST265_thread::RST265_thread(BufferedSerial *com, DATA_Xchange *data,float Ts): thread(osPriorityNormal5, 4096), sd(1)// din1(PB_2)
//RST265_thread::RST265_thread( DATA_Xchange *data,float Ts): thread(osPriorityNormal, 4096)	// din1(PB_2)
{
    // init serial
    this->uart = com;
    this->Ts = Ts;
    this->m_data = data;
    ThisThread::sleep_for(chrono::milliseconds(100));
    this->RS_z0 = 0.0;
    this->R0_is_set = false;
    uart->write("s",1);

    //init SD card
    
    if(!sd.init()){
        printf("SD init failed\n"); //if this fails all operations will be ignored(in case you wanna use it without sd card)
    }
    // define a header to know what values go where
    char rover_header[] = "itow[ms];carrSoln;lon;lat;height[m];x[mm];y[mm];z[mm];hAcc[mm];vAcc[mm];x;y;z;qx;qy;qz;qw;LoRa_valid;SNR;RSSI;ax;ay;az;gx;gy;gz;\n";
    sd.write2sd(rover_header,sizeof(rover_header));

}

// #### destructor
RST265_thread::~RST265_thread()
{
}


// #### run the statemachine
void RST265_thread::run(void)
{
    // returnvalue
    bool retVal = false;
    Matrix <float, 3, 3> I3;
    bool signal_valid = false;
    int8_t snr = 0;
    int8_t rssi = 0;

	char csm;
    uint8_t k2,k3;
    I3.setIdentity();

    


    while(true) 
		{
        ThisThread::flags_wait_any(threadFlag);
        //m_mutex.lock();
		// -----------------------------------------------------
        int32_t num = uart->read(buffer, sizeof(buffer));
        rs_imu->readAccel();
        rs_imu->readGyro();
        rssi = rs_lora->getRSSI();
        snr = rs_lora->getSNR();
        signal_valid = rs_lora->signal_valid();

        bool num_valid = true;
		csm = 0;
		if(num == (LEN_OF_EXP_TYPE * NUM_OF_VALUE+1))
			{
            for(uint8_t i = 0; i < NUM_OF_VALUE; i++) 
                {
                values[i] = *(float *) &buffer[i * LEN_OF_EXP_TYPE];
                num_valid = (num_valid & !isnan(values[i]));
                for(k2 = 0;k2<4;k2++)
                    csm += buffer[i * LEN_OF_EXP_TYPE + k2];
                }
            uint8_t dcsm_RPI0 = csm - buffer[LEN_OF_EXP_TYPE * NUM_OF_VALUE];
            
            bool csm_ok = (dcsm_RPI0==0);
            if(true)//(abs(sqrt(no)-1.0f)<.00001) & num_valid)
                {
                m_data->sens_RS(0) = -values[2]; // x
                m_data->sens_RS(1) = -values[0]; // y
                m_data->sens_RS(2) = values[1];  // z
                m_data->sens_RS(3) = values[6];  // qw is 4th entry in RS quaternion!
                m_data->sens_RS(4) = -values[5]; // RS -qz  -->  our qx
                m_data->sens_RS(5) = -values[3]; // RS -qx  -->  our qy
                m_data->sens_RS(6) = values[4];  // RS  qy  -->  our qz
                 if(!R0_is_set)
                    {
                        quat_conj(m_data->sens_RS.block<4,1>(3,0),&i_quat0);
                        yaw_0 = quat2yaw_angle(m_data->sens_RS.block<4,1>(3,0));
                        R0_is_set = true; 
                    }
                }
            else
                ;
            }
		else
			{
            }
		uart->write("s",1);
        char buf[400];
        uint16_t l = 0;

        // save data to sd card
        printf("itow = %d\n", rs_gps->itow);
        l = sprintf(buf, "%d;%d;", rs_gps->itow, rs_gps->rtk_stat)+1;
        sd.write2sd(buf, l);
        l = sprintf(buf, "%.9f;%.9f;%.5f;", rs_gps->lon, rs_gps->lat, rs_gps->height)+1;
        sd.write2sd(buf, l);
        l = sprintf(buf, "%.2f;%.2f;%.2f;", rs_gps->rel_x, rs_gps->rel_y, rs_gps->rel_z)+1;
        sd.write2sd(buf, l);
        l = sprintf(buf, "%.2f;%.2f;", rs_gps->hAcc, rs_gps->vAcc);
        sd.write2sd(buf, l);
        
        //RS data;
        l = sprintf(buf, "%3.5f;%3.5f;%3.5f;",m_data->sens_RS(0),m_data->sens_RS(1),m_data->sens_RS(2));
        sd.write2sd(buf, l);
        l = sprintf(buf, "%3.5f;%3.5f;%3.5f;%3.5f;",m_data->sens_RS(3),m_data->sens_RS(4),m_data->sens_RS(5),m_data->sens_RS(6));
        sd.write2sd(buf, l);

        l = sprintf(buf, "%d;%d;%d;",signal_valid, snr, rssi)+1;
        sd.write2sd(buf, l);

        l = sprintf(buf, "%.5f;%.5f;%.5f;",rs_imu->accX,rs_imu->accY,rs_imu->accZ)+1;
        sd.write2sd(buf, l);
        
        l = sprintf(buf, "%.5f;%.5f;%.5f;",rs_imu->gyroX,rs_imu->gyroY,rs_imu->gyroZ)+1;
        sd.write2sd(buf, l);
        sd.writeln();

        }
}


void RST265_thread::start_RST265(void)
{
    thread.start(callback(this, &RST265_thread::run));
    ticker.attach(callback(this, &RST265_thread::sendSignal), Ts);
    printf("RS265 Thread started now\r\n");
}
// this is for realtime OS
void RST265_thread::sendSignal()
{
    thread.flags_set(threadFlag);
}

// SOME QUATERNION STUFF

// #### transform quaternion orientation parameters into roll-pitch-yaw (3-2-1 / Z-Y-X euler angles)
void RST265_thread::quat2rpy(float qw, float qx, float qy, float qz, float *phi, float *theta, float *psi)
{
    // - All angles lie whitin the intervall (-pi,+pi)
    // - Note that the gimbal-lock situation occurs when 2(q1q3+q0q2)=�1 (which gives a theta of +/-pi/2),
    // so it can be clearly identified before you attempt to evaluate phi and theta

    float s = 1.0f/(qw*qw + qx*qx + qy*qy + qz*qz);

    *phi = atan2f(s*(qy*qz + qw*qx), 0.5f - s*(qx*qx + qy*qy));

    *theta = -2.0f*s*(qx*qz - qw*qy);
    if(*theta >  1.0f) *theta =  1.0f;
    if(*theta < -1.0f) *theta = -1.0f;
    *theta = asinf(*theta);

    *psi = atan2f(s*(qx*qy + qw*qz), 0.5f - s*(qy*qy + qz*qz));

    return;
}

void RST265_thread::quat2rotm(Matrix<float,4,1> q, Matrix<float,3,3> *R)
{
    *R <<   1.0f - 2.0f*(q(2)*q(2) + q(3)*q(3)), 2.0f*(q(1)*q(2) - q(0)*q(3)), 2.0f*(q(1)*q(3) + q(0)*q(2)),
            2.0f*(q(1)*q(2) + q(0)*q(3)), 1.0f - 2.0f*(q(1)*q(1) + q(3)*q(3)), 2.0f*(q(2)*q(3) - q(0)*q(1)),
            2.0f*(q(1)*q(3) - q(0)*q(2)), 2.0f*(q(2)*q(3) + q(0)*q(1)), 1.0f - 2.0f*(q(1)*q(1) + q(2)*q(2));
}


// calc the yaw angle (rotZ), based on a quaternion (1st entry of Matlab-function quat2eul, formula from there)
float RST265_thread::quat2yaw_angle(Matrix<float,4,1> q)
{
    return atan2( 2.0f*(q(1)*q(2)+q(0)*q(3)), q(0)*q(0) + q(1)*q(1) - q(2)*q(2) - q(3)*q(3));
}

void RST265_thread::reset_RS_z0()
{
    RS_z0 = m_data->sens_RS(9);
}

Matrix<float,4,1> RST265_thread::quatproduct(Matrix<float,4,1> q1,Matrix<float,4,1> q2)
{
     Matrix<float,4,1> q_ret; 
     q_ret << q1(0)*q2(0)-q1(1)*q2(1)-q1(2)*q2(2)-q1(3)*q2(3),
         q1(0)*q2(1)+q1(1)*q2(0)+q1(2)*q2(3)-q1(3)*q2(2),
         q1(0)*q2(2)-q1(1)*q2(3)+q1(2)*q2(0)+q1(3)*q2(1),
         q1(0)*q2(3)+q1(1)*q2(2)-q1(2)*q2(1)+q1(3)*q2(0);
         return q_ret;
}
void RST265_thread::quat_conj(Matrix<float,4,1> q,Matrix<float,4,1> *q_ret)
{
    *q_ret << q(0), -q(1), -q(2), -q(3);
}

void RST265_thread::setImuPtr(LSM9DS1 *_rs_imu){
    rs_imu = _rs_imu;
    return;
}
void RST265_thread::setLoraPtr(RFM95 *_rs_lora){
    rs_lora = _rs_lora;
    return;
}
void RST265_thread::setGpsPtr(RTCM3_UBLOX *_rs_gps){
    rs_gps = _rs_gps;
    return;
}