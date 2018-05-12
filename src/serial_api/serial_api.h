/**
* @file serial_api.h
* @brief This is the API to use boost asio serial.
* @version 0.0
* @author LeatherWang
* @date 2018-2-9
*/

#ifndef SERIAL_API_H
#define SERIAL_API_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
#include <math.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace boost::asio;

typedef unsigned char uchar;
typedef unsigned short ushort;

#define FLAG_SIZE 5
//#define USE_ODOM_RAW
typedef enum{
    FlagVel=0,
    FlagPose,
    FlagCmd, //串口助手使用
    Temp2,
    Temp3
} Receive_Flag_;

typedef union{
    float sum;
    uchar son[4];
} UnionFloat_;

typedef union{
    uint8_t data[4];
    int32_t vel;
} UnionInt_;

namespace slam_car
{

class SerialPortAPI
{
public:
    SerialPortAPI(const string &port_name);
    ~SerialPortAPI();
    // set velocity
    void set_velocity_to_stm(const float &vx, const float &vz, Receive_Flag_ flag);
    void set_zero_velocity_to_stm();
    bool is_opened(void);
    void set_receive_flag(Receive_Flag_ index);
    bool get_and_clear_receive_flag(Receive_Flag_ index);
    // start a thread
    void start_read_serial_thread();

private:
    void init_serial(const std::string &device);
    void check_serial_data(uchar com_data);
    template <class T>
    inline int getArrayLen(T& array){
        return (sizeof(array)/sizeof(array[0]));
    }
    void decode_serial_data(uchar *data, uchar num);
    bool send_data_to_stm(uchar *bufferArray, uchar num);
    // read serial data
    static void *read_from_serial(void *__this);
    void odom_raw_data_decode(uint8_t &com_data_);

public:
    float position_x, position_y, rotation_z, velocity_th;
    bool is_pos_recvived[FLAG_SIZE]; //接收标志
    bool is_thread_exit;

private:
    io_service m_iosev;
    serial_port *p_serial_port;
    UnionFloat_ m_float_union;
    UnionInt_ m_int_union;
    uchar RxState;
    uchar RxBufferArr[256];
    uchar TxBufferArr[50];

    pthread_mutex_t mutex;
};
}//namespace
#endif
