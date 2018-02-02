#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>
#include "std_msgs/String.h"

using namespace std;
using namespace boost::asio;
unsigned char buf[1];
io_service iosev;
serial_port sp(iosev);

void *read_serial(void *)
{
    //pthread_detach(pthread_self()); //与 pthread_attr_t 设置属性效果等同，只不过后者可以设置更多线程的属性
    while(1)
    {
        read(sp, buffer(buf));
        string str(&buf[0],&buf[1]);
        std_msgs::String msg;
        std::stringstream ss;
        ss <<str;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        //serial_pub.publish(msg);
    }
    std::cout<<"thread halt!!!"<<endl;
}

void init_serial(const std::string &device)
{
    sp.open(device);
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control());
    sp.set_option(serial_port::parity());
    sp.set_option(serial_port::stop_bits());
    sp.set_option(serial_port::character_size(8));
    ROS_INFO("open success");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "serial_asio");
    ros::NodeHandle n;

    std::string serial_dev;
    n.param<std::string>("serial_dev", serial_dev, "/dev/ttyUSB0");
    ros::Publisher serial_pub = n.advertise<std_msgs::String>("serial_raw", 1000);
    ros::Rate loop_rate(10);
    //ROS_INFO("%s",serial_dev.c_str());

    init_serial(serial_dev.c_str());

    pthread_t thread_serial;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    int rc = pthread_create(&thread_serial, &attr, read_serial, NULL); //串口接收线程
    if(rc){
        printf("ERROR; return code from pthread_create() is %d\n", rc);
        exit(-1);
    }

    while (ros::ok()) {
        // write(sp, buffer(buf1, 6));  //write the speed for cmd_val
        write(sp, buffer("Hello world", 12));
        ros::spinOnce();
        loop_rate.sleep();
    }
    iosev.run();

    pthread_attr_destroy(&attr);//Free attribute
    std::cout<<"process halt!!!"<<endl;
    return 0; //相当于_exit()，导致进程及其所有线程退出
}










