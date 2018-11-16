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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <stdio.h>
#include "remove_file.h"
//using namespace std;

template<class T>
std::string toString(const T &value) {
    std::ostringstream os;
    os << value;
    return os.str();
}

class Sample_Data
{

public:
    Sample_Data();
    ~Sample_Data();
    void spin();

private:
    ros::NodeHandle n;

    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      slam_car::stm_to_pc> ImagesSyncPolicy;

    message_filters::Subscriber<sensor_msgs::Image>  *image_for_sync_sub_;
    message_filters::Subscriber<slam_car::stm_to_pc> *odom_for_sync_sub_;
    message_filters::Synchronizer<ImagesSyncPolicy> *sync_;

    const static int rate = 200; /** @attention */

    double pose_x;
    double pose_y;
    double pose_th;

    double velocity_th;

    std::string strFolderPathMain, image_store_path, odom_file_store_path;
    std::ofstream write_odom_file;

    ros::Time start_sample_time;
    ros::Time last_sample_time;

    void sync_callback(const sensor_msgs::ImageConstPtr& image_msg, const slam_car::stm_to_pcConstPtr& odom_msg);
};

Sample_Data::Sample_Data()
{
    // 初始化变量
    pose_x = pose_y = pose_th = 0.0;
    velocity_th = 0.0;

    // 确定存储文件的路径
    strFolderPathMain = "/home/jiga/jiga_temp/data";//home/leather/leather_temp/data
    n.param<std::string>( "DataPath", strFolderPathMain, strFolderPathMain );
    if(strFolderPathMain.empty())
    {
        std::cout<<FRED("[ERROR:] ")<<"the DataPath is empty!!! "<<std::endl;
        exit(-1);
    }
    image_store_path = strFolderPathMain + "/image/";
    odom_file_store_path = strFolderPathMain + "/rec/Odo.rec";

    // 清除文件夹下的所有文件
    printdir(image_store_path.c_str(), 0);
    std::cout<<"clear all file."<<std::endl;

    //ios::trunc表示在打开文件前将文件清空,由于是写入,文件不存在则创建
    write_odom_file.open(odom_file_store_path.c_str(), std::ios_base::trunc);
    if(!write_odom_file.is_open())
    {
        std::cout<<FRED("[ERROR:] ")<<"fail to open file: "<<odom_file_store_path<<"!!!"<<std::endl;
        exit(-1);
    }
    write_odom_file<<"# odometry info"<<"\n";
    write_odom_file<<"# format: lp timeOdo timeCam x y theta"<<"\n";

    // 时间同步
    image_for_sync_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(n, "/usb_cam/image_raw", 1);
    odom_for_sync_sub_ = new message_filters::Subscriber<slam_car::stm_to_pc>(n, "/odomtry_from_stm", 1);
    sync_ = new message_filters::Synchronizer<ImagesSyncPolicy>(ImagesSyncPolicy(10),*image_for_sync_sub_, *odom_for_sync_sub_);
    sync_->registerCallback(boost::bind(&Sample_Data::sync_callback, this, _1, _2));

    // 开始采样时间戳
    start_sample_time = ros::Time::now();
    last_sample_time = ros::Time::now();
}

Sample_Data::~Sample_Data()
{
    // 关闭文件
    write_odom_file.close();

    // 删除指针
    delete image_for_sync_sub_;
    delete odom_for_sync_sub_;
    delete sync_;
}

void Sample_Data::sync_callback(const sensor_msgs::ImageConstPtr& image_msg, const slam_car::stm_to_pcConstPtr &odom_msg)
{
    // Solve all of perception here...
    static unsigned int comein_counter=0;

//    std::cout<<"stamp: "<<image_msg->header.stamp<<" "<<odom_msg->header.stamp<<" "<<
//          (image_msg->header.stamp - odom_msg->header.stamp).toSec()<<std::endl;

    ros::Time current_time = ros::Time::now();
    double time_goesby = (current_time-last_sample_time).toSec();
    if(time_goesby < 0.06)
    {
        std::cout<<"time is too short, escape it"<<std::endl;
        return;
    }
    std::cout<<time_goesby<<std::endl;
    last_sample_time = current_time;

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat image = cv_ptr->image;

    write_odom_file<<comein_counter<<" "<<
                     (odom_msg->header.stamp-start_sample_time).toSec()<<" "<<
                     (image_msg->header.stamp-start_sample_time).toSec()<<" "<<
                     odom_msg->coord_x_to_pc*1000.0<<" "<<
                     odom_msg->coord_y_to_pc*1000.0<<" "<< //单位转换为:mm
                     odom_msg->z_angle_to_pc<<"\n";

    /** @todo 测试时间，决定是否放到线程里去做*/
//    ros::Time time_before = ros::Time::now();
    cv::imwrite(image_store_path+toString(comein_counter)+".png", image);
    comein_counter++;
//    if((ros::Time::now()-time_before).toSec() > 0.001)
//        std::cout<<(ros::Time::now()-time_before).toSec()<<std::endl;

    if(comein_counter>20000) //防止占用所有硬盘资源
        exit(-1);
//    cv::imshow("image", image);
//    cv::waitKey(1);

}

void Sample_Data::spin()
{
    ros::Rate loop_rate(rate);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"sample_data");
    Sample_Data obj;
    obj.spin();
    return 0;
}
