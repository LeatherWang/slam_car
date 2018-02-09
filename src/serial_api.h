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

typedef union{
    ushort sum;
    uchar son[2];
} UnionInt_;

typedef union{
    float sum;
    uchar son[4];
} UnionFloat_;

class SerialPortAPI
{
public:
    SerialPortAPI(const string &port_name);
    ~SerialPortAPI();
    void write_to_serial(short int posionX, short int posionY, float positionZ, uchar flag);
    void read_from_serial(uchar *buf);
    bool is_opened(void);

private:
    void init_serial(const std::string &device);
    void check_serial_data(uchar com_data);
    void decode_serial_data(uchar *data, uchar num);
    template <class T>
    inline int getArrayLen(T& array){
        return (sizeof(array)/sizeof(array[0]));
    }

public:
    ushort position_x, position_y;
    float rotation_z;
    bool is_pos_recvived[5]; //接收标志

private:
    io_service m_iosev;
    serial_port *p_serial_port;
    UnionInt_ union_data_int;
    UnionFloat_ union_data_float;
};
#endif
