#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "std_msgs/String.h"
#include "serial_api.h"
#include "slam_car/pc_to_stm.h"
#include "slam_car/stm_to_pc.h"
#include <exception>
#include "color.h"
#include <slam_car/CamOdomStamp.h>
#include <slam_car/OdomTriggerControl.h>

namespace slam_car
{

class Serial_Asio
{
public:
    Serial_Asio();
    ~Serial_Asio();
    void spin();
    void handle_receive_serial_data();

private:
    ros::NodeHandle nh;
    SerialPortAPI *sp_api;
    ros::Publisher pub_stm_data;
    ros::Subscriber sub_set_motor;
    ros::Publisher cam_odom_sync;
    ros::ServiceServer serverTrigger_;

    const static int rate=200; //读取里程计频率
    bool sendTrigger;
    int sendCounter;
    double timeDiff;
    ros::Time lastTime;
    void set_motor_callback(const slam_car::pc_to_stm &msg);
    bool servTriggerCb(slam_car::OdomTriggerControl::Request &req, slam_car::OdomTriggerControl::Response &resp);
};

Serial_Asio::Serial_Asio()
{
    SerialPortAPI::callback_t data_received_cb = boost::bind(
                &Serial_Asio::handle_receive_serial_data, this); //! @attention 要加Serial_Asio::

    std::string serial_dev="/dev/ttyUSB0";

    /** @todo*/
    ros::NodeHandle n_private("~");
    n_private.param<std::string>("serial_dev", serial_dev, "/dev/ttyUSB0");
    try
    {
        sp_api = new SerialPortAPI(serial_dev, data_received_cb);
        if(sp_api->is_opened()){
            cout<<FGRN("[SUCCESS:] ")<<"open the serial port: "<<serial_dev<<endl;
        }
    }
    catch(exception e)
    {
        cout<<FRED("[ERROR:] ")<<"can't open the serial port : "<<serial_dev<<endl;
        exit(-1);
    }

    sendTrigger = false;
    sendCounter = 0;
    lastTime = ros::Time::now();
    pub_stm_data = nh.advertise<slam_car::stm_to_pc>("odomtry_from_stm", 1000);
    sub_set_motor = nh.subscribe("stm_motor", 100, &Serial_Asio::set_motor_callback, this);
    cam_odom_sync = nh.advertise<slam_car::CamOdomStamp>("/slam_car/cam_odom_sync_stamp", 1000);
    serverTrigger_ = nh.advertiseService("/slam_car/trigger_control", &Serial_Asio::servTriggerCb, this);

    sp_api->start_read_serial_thread();
}

Serial_Asio::~Serial_Asio()
{
    // 意外退出时，机器人速度置为0
    /** @todo */
    sp_api->is_thread_exit = true;
    usleep(100000);
    sp_api->set_zero_velocity_to_stm();
    usleep(100000);
std::cout<<"<Serial_Asio> is destruced"<<std::endl;
    // 手动删除指针变量!
    delete sp_api;
}

//Spin function
void Serial_Asio::spin()
{
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
//        ros::Time curTime = ros::Time::now();
//        if(sendTrigger)
//        {
//            if((curTime-lastTime).toSec() > 0.05)
//            {
//                lastTime = curTime;
//                slam_car::CamOdomStamp msg;
//                msg.frame_stamp = curTime;
//                msg.frame_seq_id = sendCounter; sendCounter++;
//                cam_odom_sync.publish(msg);
//            }
//            dataCounter++;
//        }
        //! @todo 使用回调函数或者虚函数实现??
        //! 测试一下使用rostopic，从发送到接受到的时间差有多少

        ros::spinOnce();
        loop_rate.sleep();
    }
}

// 接收完一帧完整数据，回调函数
// 注意测试该函数执行时间，是否会导致串口接收数据不正常，如果函数耗时过长，还是放到while循环里面去做吧
void Serial_Asio::handle_receive_serial_data()
{
    if(sp_api->get_and_clear_receive_flag(FlagPose))
    {
        ros::Time curTime = ros::Time::now();
        if(sendTrigger)
        {
            if(sp_api->trigger)
            {
                slam_car::CamOdomStamp msg;
                msg.frame_stamp = curTime;
                msg.frame_seq_id = sendCounter; sendCounter++;
                cam_odom_sync.publish(msg);
                timeDiff = (curTime-lastTime).toSec();
                if(timeDiff>0.043 || timeDiff<0.035)
                    cout<<timeDiff<<endl;
                lastTime = curTime;
            }
        }
        slam_car::stm_to_pc stm_data;
        stm_data.header.stamp = curTime;
        stm_data.header.frame_id = "odom_to_pc";
        stm_data.coord_x_to_pc = sp_api->position_x;
        stm_data.coord_y_to_pc = sp_api->position_y;
        stm_data.z_angle_to_pc = sp_api->rotation_z;

        stm_data.velocity_vth_to_pc = sp_api->velocity_th;
        pub_stm_data.publish(stm_data);
    }
}

void Serial_Asio::set_motor_callback(const slam_car::pc_to_stm &msg)
{
    slam_car::pc_to_stm msg_temp = msg;
    std::cout<<"send vel: "<<msg_temp.vel_x_to_stm<<" "<<msg_temp.z_angle_vel_to_stm<<std::endl;
    sp_api->set_velocity_to_stm(msg_temp.vel_x_to_stm, msg_temp.z_angle_vel_to_stm, FlagVel);
}

bool Serial_Asio::servTriggerCb(OdomTriggerControl::Request &req, OdomTriggerControl::Response &resp)
{
    if(req.trigger_enable)
    {
        sp_api->set_trigger_to_stm('1', FlagTrigger);
        sendTrigger = true;
        cout<<"start trigger, sendCounter: "<<sendCounter<<endl;
    }
    else
    {
        sp_api->set_trigger_to_stm('0', FlagTrigger);
        sendTrigger = false;
        cout<<"stop trigger, stamp_buffer_offset should be "<<sendCounter-1<<endl;
    }
    resp.success = true;
    return true;
}

} //namespace

using namespace slam_car;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_asio");
    slam_car::Serial_Asio serial_asio_obj;
    serial_asio_obj.spin();
    return 0; //相当于_exit()，导致进程及其所有线程退出
}















