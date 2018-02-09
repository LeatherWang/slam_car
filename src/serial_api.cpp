
#include "serial_api.h"

#define BYTE0(dwTemp)       ( *( (uchar *)(&dwTemp)	) )
#define BYTE1(dwTemp)       ( *( (uchar *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (uchar *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (uchar *)(&dwTemp) + 3) )

UnionInt_ union_data_int;
UnionFloat_ union_data_float;

void init_serial(const std::string &device)
{
    sp.open(device);
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control());
    sp.set_option(serial_port::parity());
    sp.set_option(serial_port::stop_bits());
    sp.set_option(serial_port::character_size(8));
}

void decode_serial_data(uchar *data, uchar num)
{
    uchar sum = 0;
    ushort position_x, position_y;
    float rotation_z;
    for(uchar i=0;i<(num-1);i++)
        sum += (uchar(data[i]));
    if(!(sum == uchar(data[num-1])))	return;		//判断sum，校验--Leather
    if(!(uchar(data[0])==0xAA && uchar(data[1])==0xAF))		return;		//判断帧头--Leather

    if(uchar(data[2]) == 0X01){
        union_data_int.son[1] = uchar(data[4]);
        union_data_int.son[0] = uchar(data[5]);
        position_x = union_data_int.sum;

        union_data_int.son[1] = uchar(data[6]);
        union_data_int.son[0] = uchar(data[7]);
        position_y = union_data_int.sum;

        union_data_float.son[3] = uchar(data[8]);
        union_data_float.son[2] = uchar(data[9]);
        union_data_float.son[1] = uchar(data[10]);
        union_data_float.son[0] = uchar(data[11]);
        rotation_z = union_data_int.sum;

        cout<<"x:"<<position_x<<"y:"<<position_y<<"z:"<<rotation_z<<endl;
    }
}

uchar RxState = 0;
uchar RxBuffer[256];
void check_serial_data(uchar com_data)
{
    static uchar _data_len = 0,_data_cnt = 0;
    if(RxState==0&&com_data==0xAA){
        RxState=1;
        RxBuffer[0]=com_data;
    }
    else if(RxState==1&&com_data==0xAF){
        RxState=2;
        RxBuffer[1]=com_data;
    }
    else if(RxState==2&&com_data<=0XF1){
        RxState=3;
        RxBuffer[2]=com_data;
    }
    else if(RxState==3&&com_data<255){
        RxState = 4;
        RxBuffer[3]=com_data;
        _data_len = com_data;
        _data_cnt = 0;
    }
    else if(RxState==4&&_data_len>0){
        _data_len--;
        RxBuffer[4+_data_cnt++]=com_data;
        if(_data_len==0)
            RxState = 5;
    }
    else if(RxState==5){
        RxState = 0;
        RxBuffer[4+_data_cnt]=com_data;
        decode_serial_data(RxBuffer, _data_cnt+5);
    }
    else
        RxState = 0;
}

char array[50];
void write_to_serial(short posionX, short posionY, float positionZ, uchar flag)
{
    uchar _cnt=0;
    uchar i=0;
    uchar sum = 0;

    array[_cnt++]=0xAA;
    array[_cnt++]=0xAF;
    array[_cnt++]=flag;
    array[_cnt++]=0;

    array[_cnt++]=BYTE1(posionX);
    array[_cnt++]=BYTE0(posionX);
    array[_cnt++]=BYTE1(posionY);
    array[_cnt++]=BYTE0(posionY);

    array[_cnt++]=BYTE3(positionZ);
    array[_cnt++]=BYTE2(positionZ);
    array[_cnt++]=BYTE1(positionZ);
    array[_cnt++]=BYTE0(positionZ);

    array[3] = _cnt-4;

    for(i=0;i<_cnt;i++)
        sum += array[i];

    array[_cnt++]=sum;

    if(sp.is_open()){
        write(sp, buffer(array, _cnt));
    }
}
