
#include "serial_api.h"

namespace slam_car
{

#define BYTE0(dwTemp)       ( *( (uchar *)(&dwTemp)	) )
#define BYTE1(dwTemp)       ( *( (uchar *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (uchar *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (uchar *)(&dwTemp) + 3) )

SerialPortAPI::SerialPortAPI(const string &port_name)
{
    p_serial_port = new serial_port(m_iosev);
    if (p_serial_port){
        this->init_serial(port_name);
    }

    uchar length = getArrayLen(this->is_pos_recvived);
    for(uchar i=0; i<length; i++){
        this->is_pos_recvived[i] = false;
    }
}

SerialPortAPI::~SerialPortAPI()
{
    /** @todo 需要析构函数，就需要定义自己的复制构造函数与赋值运算符重载，以避免浅拷贝*/
    if (p_serial_port){
        delete p_serial_port;
    }

    //std::cout<<"<SerialPortAPI> is destruced"<<std::endl;

    // distory mutex
    pthread_mutex_destroy(&mutex);
}

void SerialPortAPI::init_serial(const std::string &device)
{
    p_serial_port->open(device);
    p_serial_port->set_option(serial_port::baud_rate(115200));
    p_serial_port->set_option(serial_port::flow_control());
    p_serial_port->set_option(serial_port::parity());
    p_serial_port->set_option(serial_port::stop_bits());
    p_serial_port->set_option(serial_port::character_size(8));
}

void SerialPortAPI::decode_serial_data(uchar *data, uchar num)
{
    uchar sum = 0;
    for(uchar i=0;i<(num-1);i++)
        sum += (uchar(data[i]));
    if(!(sum == uchar(data[num-1])))	return;
    if(!(uchar(data[0])==0xAA && uchar(data[1])==0xAF))		return;

    if(uchar(data[2]) == 0X01)
    {
        m_float_union.son[0] = uchar(data[4]);
        m_float_union.son[1] = uchar(data[5]);
        m_float_union.son[2] = uchar(data[6]);
        m_float_union.son[3] = uchar(data[7]);
        position_x = m_float_union.sum;

        m_float_union.son[0] = uchar(data[8]);
        m_float_union.son[1] = uchar(data[9]);
        m_float_union.son[2] = uchar(data[10]);
        m_float_union.son[3] = uchar(data[11]);
        position_y = m_float_union.sum;

        m_float_union.son[0] = uchar(data[12]);
        m_float_union.son[1] = uchar(data[13]);
        m_float_union.son[2] = uchar(data[14]);
        m_float_union.son[3] = uchar(data[15]);
        rotation_z = m_float_union.sum;

        m_float_union.son[0] = uchar(data[16]);
        m_float_union.son[1] = uchar(data[17]);
        m_float_union.son[2] = uchar(data[18]);
        m_float_union.son[3] = uchar(data[19]);
        velocity_th = m_float_union.sum;

        set_receive_flag(Receive_Flag_(data[2])); //标志位,当set标志位后才可读
    }
}

/**
* @brief 核对串口数据帧是否正确
* @attention
* @details
*/
void SerialPortAPI::check_serial_data(uchar com_data)
{
    static uchar _data_len = 0,_data_cnt = 0;

    if(RxState==0&&com_data==0xAA){
        RxState=1;
        RxBufferArr[0]=com_data;
    }
    else if(RxState==1&&com_data==0xAF){
        RxState=2;
        RxBufferArr[1]=com_data;
    }
    else if(RxState==2&&com_data<=0XF1){
        RxState=3;
        RxBufferArr[2]=com_data;
    }
    else if(RxState==3&&com_data<255){
        RxState = 4;
        RxBufferArr[3]=com_data;
        _data_len = com_data;
        _data_cnt = 0;
    }
    else if(RxState==4&&_data_len>0){
        _data_len--;
        RxBufferArr[4+_data_cnt++]=com_data;
        if(_data_len==0)
            RxState = 5;
    }
    else if(RxState==5){
        RxState = 0;
        RxBufferArr[4+_data_cnt]=com_data;
        decode_serial_data(RxBufferArr, _data_cnt+5);
    }
    else
        RxState = 0;
}

void SerialPortAPI::odom_raw_data_decode(uint8_t& com_data_)
{
    static uint8_t count = 0;
    static uint8_t i = 0;

    static union
    {
        uint8_t data[24];
        float   ActVal[6];
    }posture;

    uint8_t com_data = com_data_;
    if(count==0 && com_data==0x0D)
    {
        count++;
    }
    else if(count == 1 && com_data==0x0a)
    {
        i=0;
        count++;
    }
    else if(count == 2)
    {
        posture.data[i]=com_data;
        i++;
        if(i>=24)
        {
            i=0;
            count++;
        }
    }
    else if(count == 3 && com_data==0x0a)
        count++;
    else if(count == 4 && com_data==0x0d)
    {
        position_x = posture.ActVal[3];
        position_y = posture.ActVal[4];
        rotation_z = posture.ActVal[0];
        velocity_th = posture.ActVal[5];
        set_receive_flag(FlagPose);
        count = 0;
//        cout<<position_x<<" "<<position_y<<" "<<rotation_z<<endl;
    }
    else
        count=0;
}

/**
* @brief 核对串口数据帧是否正确
* @bug   一次只能读取一个字节的数据
* @todo  修改上述bug，形参包含要读数据个数
*/
void * SerialPortAPI::read_from_serial(void *__this)
{
    pthread_detach(pthread_self()); //将joined线程重设置为分离线程，省去资源回收的麻烦，不使用pthread_join进行资源回收，将导致僵尸线程，占据资源不释放
    SerialPortAPI * _this =(SerialPortAPI *)__this;
    uchar buf[1];
    uchar ch;
    std::cout<<"thread start"<<endl;
    while(!_this->is_thread_exit)
    {
        read(*_this->p_serial_port, buffer(buf)); //如果一直没接收到数据，将一直阻塞在这里
        ch=buf[0];

        #ifdef USE_ODOM_RAW
            _this->odom_raw_data_decode(ch);
        #else
            _this->check_serial_data(ch);
        #endif
    }
    std::cout<<"thread halted!"<<endl;
    return 0;
}

void SerialPortAPI::start_read_serial_thread()
{
    /** @todo */
    is_thread_exit = false;
    pthread_t pth;
    pthread_create(&pth,NULL,read_from_serial,(void*)this);
}

void SerialPortAPI::set_velocity_to_stm(const float &vx, const float &vz, Receive_Flag_ flag)
{
    uchar _cnt=0;
    uchar i=0;
    uchar sum = 0;

    TxBufferArr[_cnt++]=0xAA;
    TxBufferArr[_cnt++]=0xAF;
    TxBufferArr[_cnt++]=flag;
    TxBufferArr[_cnt++]=0;

    m_int_union.vel = (int32_t)(-vx*1000.0); //vx:-0.4~0.4m/s,放大1000倍发送
    TxBufferArr[_cnt++]=m_int_union.data[0];
    TxBufferArr[_cnt++]=m_int_union.data[1];
    TxBufferArr[_cnt++]=m_int_union.data[2];
    TxBufferArr[_cnt++]=m_int_union.data[3];

    m_int_union.vel = (int32_t)(vz*1000.0); //vz:-10~10rad/s,放大1000倍发送
    TxBufferArr[_cnt++]=m_int_union.data[0];
    TxBufferArr[_cnt++]=m_int_union.data[1];
    TxBufferArr[_cnt++]=m_int_union.data[2];
    TxBufferArr[_cnt++]=m_int_union.data[3];

    TxBufferArr[3] = _cnt-4;

    for(i=0;i<_cnt;i++)
        sum += TxBufferArr[i];

    TxBufferArr[_cnt++]=sum;

    send_data_to_stm(TxBufferArr, _cnt);
}

// 紧急制动
void SerialPortAPI::set_zero_velocity_to_stm()
{
    #ifndef USE_ODOM_RAW
        set_velocity_to_stm(0.0, 0.0, FlagVel);
    #endif
}

bool SerialPortAPI::send_data_to_stm(uchar *bufferArray, uchar num)
{
    if(p_serial_port->is_open()){
        write(*p_serial_port, buffer(bufferArray, num));
        return true;
    }
    else
        return false;

}

bool SerialPortAPI::is_opened()
{
    return p_serial_port->is_open();
}

void SerialPortAPI::set_receive_flag(Receive_Flag_ index)
{
    pthread_mutex_lock (&mutex);
    is_pos_recvived[index] = true;
    pthread_mutex_unlock(&mutex);
}

bool SerialPortAPI::get_and_clear_receive_flag(Receive_Flag_ index)
{
    pthread_mutex_lock (&mutex);
    if(is_pos_recvived[index])
    {
        is_pos_recvived[index] = false;
        return true;
    }
    else return false;
    pthread_mutex_unlock(&mutex);
}

} //namespace




