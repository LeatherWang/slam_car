#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "std_msgs/String.h"
#include "serial_api.h"

void *handle_read_from_serial(void *)
{
    uint8_t ch;
    unsigned char buf[1];
    while(1)
    {
        read(sp, buffer(buf));
        string str(&buf[0],&buf[1]);
        std_msgs::String msg;
        std::stringstream ss;
        ss <<str;
        msg.data = ss.str();
        //ROS_INFO("%s", msg.data.c_str());

        ch = buf[0];
        check_serial_data(ch);
    }
    std::cout<<"thread halt!!!"<<endl;
}

void write_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    //write(sp, buffer(buf1, 6));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "serial_asio");
    ros::NodeHandle n;

    std::string serial_dev;
    n.param<std::string>("serial_dev", serial_dev, "/dev/ttyUSB0");
    ros::Publisher read_pub = n.advertise<std_msgs::String>("serial_raw", 1000);
    ros::Subscriber write_sub = n.subscribe("serial_write", 1000, write_callback);
    ros::Rate loop_rate(1);
    //ROS_INFO("%s",serial_dev.c_str());

    init_serial(serial_dev.c_str());
    if(!sp.is_open()){
        ROS_INFO("error to serial successfully!!!");
        exit(-1);
    }

    pthread_t thread_serial;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    int rc = pthread_create(&thread_serial, &attr, handle_read_from_serial, NULL); //串口接收线程
    if(rc){
        printf("ERROR; return code from pthread_create() is %d\n", rc);
        exit(-1);
    }

    while (ros::ok()) {
        // write(sp, buffer(buf1, 6));  //write the speed for cmd_val
        //write(sp, buffer("Hello world", 12));
        write_to_serial(10,10,12.1,1);
        ros::spinOnce();
        loop_rate.sleep();
    }
    iosev.run();

    pthread_attr_destroy(&attr);//Free attribute
    std::cout<<"process halt!!!"<<endl;
    return 0; //相当于_exit()，导致进程及其所有线程退出
}










