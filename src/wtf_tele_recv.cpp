#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "std_msgs/String.h"
#include "serial_api.h"
#include "slam_car/pc_to_stm.h"
#include <exception>
#include "color.h"

namespace slam_car
{

class Serial_WTF_Tele
{
public:
    Serial_WTF_Tele();
    ~Serial_WTF_Tele();
    void spin();
    void handle_receive_serial_data();

private:
    ros::NodeHandle nh;
    SerialPortAPI *sp_api;
    ros::Publisher pub_motor;

    const static int rate=100; //读取里程计频率
    int wtf_yaw_mid;
};

Serial_WTF_Tele::Serial_WTF_Tele()
{
    SerialPortAPI::callback_t data_received_cb = boost::bind(
                &Serial_WTF_Tele::handle_receive_serial_data, this);

    std::string serial_dev="/dev/wtf_tele";
    wtf_yaw_mid = 0;

    /** @todo*/
    ros::NodeHandle n_private("~");
    n_private.param<std::string>("serial_dev", serial_dev, "/dev/wtf_tele");
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

    pub_motor = nh.advertise<slam_car::pc_to_stm>("stm_motor",100);
    sp_api->start_read_serial_thread();
}

Serial_WTF_Tele::~Serial_WTF_Tele()
{
    // 意外退出时，机器人速度置为0
    /** @todo */
    sp_api->is_thread_exit = true;
    usleep(100000);
std::cout<<"<wtf_tele_recv> is destruced"<<std::endl;
    // 手动删除指针变量!
    delete sp_api;
}

//Spin function
void Serial_WTF_Tele::spin()
{
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
//        handle_receive_serial_data();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Serial_WTF_Tele::handle_receive_serial_data()
{
    static char counter=0;
    if(sp_api->get_and_clear_receive_flag(FlagWTF))
    {
        slam_car::pc_to_stm pc_to_stm_data;
        // wtf_gas: 1100~1960
        // wtf_yaw: 1970~1538~1110
        // vx:-0.4~0.4m/s
        if(sp_api->wtf_gas < 1090) //1021
        {
            //cout<<FRED("[ERROR:] ")<<"yet not to recv wtf singal!!"<<endl;
            pc_to_stm_data.vel_x_to_stm = 0.0;
            pc_to_stm_data.z_angle_vel_to_stm = 0.0;
            pub_motor.publish(pc_to_stm_data);
        }
        else
        {
            if(counter<5)
            {   
                wtf_yaw_mid += sp_api->wtf_yaw;
                counter++;
                if(counter == 5)
                {
                     wtf_yaw_mid = wtf_yaw_mid/5;     
                     std::cout<<"wtf_yaw_mid: "<<wtf_yaw_mid<<std::endl;
                }     
                return;
            }
            
            double temp = sp_api->wtf_gas>1200?(sp_api->wtf_gas-1130)/1000.0:0.0; //5.0是缩放系数
            if(temp > 0.6)
                pc_to_stm_data.vel_x_to_stm = 0.6;
            else
                pc_to_stm_data.vel_x_to_stm = temp;


            //vz:-10~10rad/s
//std::cout<<sp_api->wtf_yaw<<std::endl;
            temp=sp_api->wtf_yaw>(wtf_yaw_mid+100)?((sp_api->wtf_yaw-wtf_yaw_mid)/190.0):
                                      (sp_api->wtf_yaw<(wtf_yaw_mid-100)?((sp_api->wtf_yaw-wtf_yaw_mid)/190.0):0.0);
            pc_to_stm_data.z_angle_vel_to_stm = temp;

            pub_motor.publish(pc_to_stm_data);
        }
    }

    }
}


using namespace slam_car;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "wtf_tele_recv");
    slam_car::Serial_WTF_Tele Serial_WTF_Tele_obj;
    Serial_WTF_Tele_obj.spin();
    return 0; //相当于_exit()，导致进程及其所有线程退出
}















