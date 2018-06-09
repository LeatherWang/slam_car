#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <string>
#include "slam_car/stm_to_pc.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "color.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include "sensor_msgs/Imu.h"

class ds_sync
{

public:
    ds_sync();
    ~ds_sync();
    void spin();

private:
    ros::NodeHandle n;

    const static int rate = 100; /** @attention */

    double pose_x;
    double pose_y;
    double pose_th;

    double velocity_th;

    float wheel_baseline;
    float wheel_radius;

    std::string strFolderPathMain, encoder_file_store_path, imu_file_store_path;
    std::ifstream read_encoder_file;
    std::ifstream read_imu_file;

    tf::TransformBroadcaster broadcaster;
    ros::Publisher odom_pub;
    ros::Publisher IMU_pub;

    float encoder_start_time, imu_start_time;
    bool encoder_ready,imu_ready;

    bool read_encoder_for_odometry(void);
    bool read_imu(void);
};

ds_sync::ds_sync()
{
    // 初始化变量
    pose_x = pose_y = pose_th = 0.0;
    velocity_th = 0.0;

    wheel_baseline= 0.3534f; //unit: m
    wheel_radius= 0.0925f; //0.0925f is radius

    // 确定存储文件的路径
    strFolderPathMain = "/home/leather/leather_temp/ds";
    n.param<std::string>( "DataPath", strFolderPathMain, strFolderPathMain );
    if(strFolderPathMain.empty())
    {
        std::cout<<FRED("[ERROR:] ")<<"the DataPath is empty!!! "<<std::endl;
        exit(-1);
    }

    //打开encoder.txt文件
    encoder_file_store_path = strFolderPathMain + "/encoder.txt";
    //ios::trunc表示在打开文件前将文件清空,由于是写入,文件不存在则创建
    read_encoder_file.open(encoder_file_store_path.c_str());
    if(!read_encoder_file.is_open())
    {
        std::cout<<FRED("[ERROR:] ")<<"fail to open file: "<<encoder_file_store_path<<"!!!"<<std::endl;
        exit(-1);
    }

    // 打开imu.txt文件
    imu_file_store_path = strFolderPathMain + "/imu.txt";
    read_imu_file.open(imu_file_store_path.c_str());
    if(!read_imu_file.is_open())
    {
        std::cout<<FRED("[ERROR:] ")<<"fail to open file: "<<imu_file_store_path<<"!!!"<<std::endl;
        exit(-1);
    }

    odom_pub = n.advertise<nav_msgs::Odometry>("/odom",50);
    IMU_pub = n.advertise<sensor_msgs::Imu>("/imu_raw_data",50);

    encoder_ready=false;
    imu_ready=false;
}

ds_sync::~ds_sync()
{
    // 关闭文件
    read_encoder_file.close();
    read_imu_file.close();
}

void ds_sync::spin()
{
    char counter=0;
    ros::Rate sync_rate(1000);
    ros::Rate loop_rate(rate);
    encoder_start_time = 31.0; /** @todo */
    imu_start_time = 10.0;
    float time_stamp;

    // sync start time
    while(ros::ok())
    {
        if(!encoder_ready){
            std::string str_tmp;
            if(!std::getline(read_encoder_file, str_tmp))
                exit(-1);
            std::istringstream iss(str_tmp);
            iss>>time_stamp;

            // 等待开始时间戳
            if(!(time_stamp < encoder_start_time))
                encoder_ready = true;
        }

        if(!imu_ready){
            std::string str_tmp;
            if(!std::getline(read_imu_file, str_tmp))
                exit(-1);
            std::istringstream iss(str_tmp);
            iss>>time_stamp;

            if(!(time_stamp < imu_start_time))
                imu_ready = true;
        }

        if(encoder_ready && imu_ready)
            break;
        sync_rate.sleep();
    }
    std::cout<<"sync pass!"<<std::endl;

    while(ros::ok())
    {
        counter++;
        if(!read_imu())
            break;
        if(counter%10 == 0)
        {
            if(!read_encoder_for_odometry())
                break;
            counter = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout<<"process halt!"<<std::endl;
}

//里程计计算函数
bool ds_sync::read_encoder_for_odometry(void)
{
    float delta_distance, delta_oriention;   //采样时间间隔内运动的距离
    float oriention_1;

    float right_wheel_angle, left_wheel_angle; //unit: rad
    static float last_right_wheel_angle=0.0f, last_left_wheel_angle=0.0f;
    float delta_right_wheel_angle, delta_left_wheel_angle;

    float sin_, cos_;
    float encoder_time_stamp;

    // File format: timestamp (unit: seconds) left_wheel_angle right_wheel_angle (rad)
    std::string str_tmp;
    if(!std::getline(read_encoder_file, str_tmp))
        return false;
    std::istringstream iss(str_tmp);
    iss>>encoder_time_stamp>>left_wheel_angle>>right_wheel_angle;

    delta_right_wheel_angle = right_wheel_angle - last_right_wheel_angle;
    delta_left_wheel_angle = left_wheel_angle - last_left_wheel_angle;

    last_right_wheel_angle = right_wheel_angle;
    last_left_wheel_angle = left_wheel_angle;

    delta_distance = 0.5f*(delta_right_wheel_angle + delta_left_wheel_angle)*wheel_radius;
    delta_oriention = (delta_right_wheel_angle - delta_left_wheel_angle)*wheel_radius/wheel_baseline;

    oriention_1 = pose_th + 0.5f*delta_oriention; //用于三角函数计算
    pose_th = pose_th + delta_oriention; //计算出里程计方向角

    sin_ = sin(oriention_1);
    cos_ = cos(oriention_1);
    pose_x = pose_x + delta_distance*cos_;//计算出里程计x坐标
    pose_y = pose_y + delta_distance*sin_;//计算出里程计y坐标

    //方向角角度纠正
    if(pose_th > M_PI)
        pose_th -= M_PI*2;
    else
    {
        if(pose_th < -M_PI)
            pose_th += M_PI*2;
    }

    //std::cout<<encoder_time_stamp<<" "<<pose_x<<" "<<pose_y<<" "<<pose_th<<std::endl;

    ros::Time current_time = ros::Time::now();

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    geometry_msgs::Quaternion odom_quat;

    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,pose_th);

    // update transform
    odom_trans.header.stamp = current_time;
    odom_trans.transform.translation.x = pose_x;
    odom_trans.transform.translation.y = pose_y;
    odom_trans.transform.translation.z = 0.0; //height=0
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pose_th);

    //filling the odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    // position
    odom.pose.pose.position.x = pose_x;
    odom.pose.pose.position.y = pose_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //velocity
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
//	odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = velocity_th;

    // publishing the odometry and the new tf
    broadcaster.sendTransform(odom_trans);
    odom_pub.publish(odom);

    return true;
}

bool ds_sync::read_imu(void)
{
    float gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
    float imu_time_stamp;

    std::string str_tmp;
    if(!std::getline(read_imu_file, str_tmp))
        return false;
    std::istringstream iss(str_tmp);
    iss>>imu_time_stamp>>gyro_x>>gyro_y>>gyro_z>>accel_x>>accel_y>>accel_z;
    std::cout<<imu_time_stamp<<gyro_x<<gyro_y<<gyro_z<<accel_x<<accel_y<<accel_z<<std::endl;

    sensor_msgs::Imu imu_raw;
    imu_raw.header.stamp = ros::Time::now();
    imu_raw.header.frame_id = "base_footprint";
    //imu_raw.header.seq = seq
    imu_raw.orientation_covariance[0] = -1;
    imu_raw.linear_acceleration.x = accel_x;
    imu_raw.linear_acceleration.y = accel_y;
    imu_raw.linear_acceleration.z = accel_z;
    imu_raw.linear_acceleration_covariance[0] = -1;
    imu_raw.angular_velocity.x = gyro_x;
    imu_raw.angular_velocity.y = gyro_y;
    imu_raw.angular_velocity.z = gyro_z;
    imu_raw.angular_velocity_covariance[0] = -1;
    IMU_pub.publish(imu_raw);
    return true;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"ds_sync");
    ds_sync obj;
    obj.spin();
    return 0;
}
