#ifndef SERIAL_API_H
#define SERIAL_API_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
#include <math.h>
#include <stdio.h>

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

io_service iosev;
serial_port sp(iosev);

void init_serial(const std::string &device);
void check_serial_data(uchar com_data);
void decode_serial_data(uchar *data, uchar num);
void write_to_serial(short int posionX, short int posionY, float positionZ, uchar flag);

#endif
