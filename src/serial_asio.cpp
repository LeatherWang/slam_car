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

    const static int rate=200; //读取里程计频率
    void set_motor_callback(const slam_car::pc_to_stm &msg);
};

Serial_Asio::Serial_Asio()
{
    std::string serial_dev="/dev/ttyUSB0";

    /** @todo*/
    ros::NodeHandle n_private("~");
    n_private.param<std::string>("serial_dev", serial_dev, "/dev/ttyUSB0");
    try
    {
        sp_api = new SerialPortAPI(serial_dev);
        if(sp_api->is_opened()){
            cout<<FGRN("[SUCCESS:] ")<<"open the serial port: "<<serial_dev<<endl;
        }
    }
    catch(exception e)
    {
        cout<<FRED("[ERROR:] ")<<"can't open the serial port : "<<serial_dev<<endl;
        exit(-1);
    }

    pub_stm_data = nh.advertise<slam_car::stm_to_pc>("odomtry_from_stm", 1000);
    sub_set_motor = nh.subscribe("stm_motor", 100, &Serial_Asio::set_motor_callback, this);
    sp_api->start_read_serial_thread();
}

Serial_Asio::~Serial_Asio()
{
    // 意外退出时，机器人速度置为0
    sp_api->set_zero_velocity_to_stm();

    // 手动删除指针变量!
    delete sp_api;
}

//Spin function
void Serial_Asio::spin()
{
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        handle_receive_serial_data();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Serial_Asio::handle_receive_serial_data()
{
    if(sp_api->get_and_clear_receive_flag(FlagPose))
    {
        slam_car::stm_to_pc stm_data;
        stm_data.header.stamp = ros::Time::now();
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
    std::cout<<msg_temp.vel_x_to_stm<<" "<<msg_temp.z_angle_vel_to_stm<<std::endl;
    sp_api->set_velocity_to_stm(msg_temp.vel_x_to_stm, msg_temp.z_angle_vel_to_stm, FlagVel);
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















