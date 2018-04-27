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

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

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

    string strFolderPathMain, image_store_path, odom_file_store_path;
    ofstream write_odom_file;

    ros::Time odom_lastest_sample_time, image_lastest_sample_time, start_sample_time;

    void sync_callback(const ImageConstPtr& image_msg, const slam_car::stm_to_pcConstPtr& odom_msg);
};

Sample_Data::Sample_Data()
{
    // 初始化变量
    pose_x = pose_y = pose_th = 0.0;
    velocity_th = 0.0;

    // 确定存储文件的路径
    strFolderPathMain = "/home/leather/leather_temp/data";
    n.param<string>( "DataPath", strFolderPathMain, strFolderPathMain );
    if(strFolderPathMain.empty())
    {
        cout<<FRED("[ERROR:] ")<<"the DataPath is empty!!! "<<endl;
        exit(-1);
    }
    image_store_path = strFolderPathMain + "/image/";
    odom_file_store_path = strFolderPathMain + "/rec/Odo.rec";
    //ios::trunc表示在打开文件前将文件清空,由于是写入,文件不存在则创建
    write_odom_file.open(odom_file_store_path.c_str(), ios_base::trunc);
    write_odom_file<<"# odometry info"<<"\n";
    write_odom_file<<"# format: lp timeOdo timeCam x y theta"<<"\n";

    // 时间同步
    image_for_sync_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(n, "/usb_cam/image_raw", 1);
    odom_for_sync_sub_ = new message_filters::Subscriber<slam_car::stm_to_pc>(n, "/odomtry_from_stm", 1);
    sync_ = new Synchronizer<ImagesSyncPolicy>(ImagesSyncPolicy(10),*image_for_sync_sub_, *odom_for_sync_sub_);
    sync_->registerCallback(boost::bind(&Sample_Data::sync_callback, this, _1, _2));

    // 开始采样时间戳
    start_sample_time = ros::Time::now();
}

Sample_Data::~Sample_Data()
{
    // 关闭文件
    write_odom_file.close();

    delete image_for_sync_sub_;
    delete odom_for_sync_sub_;
    delete sync_;
}

void Sample_Data::sync_callback(const ImageConstPtr& image_msg, const slam_car::stm_to_pcConstPtr &odom_msg)
{
    static unsigned int comein_counter=0;

    // Solve all of perception here...
    cout<<"stamp: "<<image_msg->header.stamp<<" "<<odom_msg->header.stamp<<" "<<
          fabs((image_msg->header.stamp - odom_msg->header.stamp).toSec())<<endl;

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat image = cv_ptr->image;

    write_odom_file<<comein_counter<<" "<<
                     (odom_msg->header.stamp-start_sample_time).toSec()<<" "<<
                     (image_msg->header.stamp-start_sample_time).toSec()<<" "<<
                     odom_msg->coord_x_to_pc*1000.0<<" "<<
                     odom_msg->coord_y_to_pc*1000.0<<" "<<
                     odom_msg->z_angle_to_pc<<"\n";

    /** @todo 测试时间，决定是否放到线程里去做*/
//    ros::Time time_before = ros::Time::now();
    cv::imwrite(image_store_path+std::to_string(comein_counter)+".png", image);
    comein_counter++;
//    if((ros::Time::now()-time_before).toSec() > 0.001)
//        cout<<(ros::Time::now()-time_before).toSec()<<endl;

    cv::imshow("image", image);
    cv::waitKey(1);

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

//void Sample_Data::odom_sub_callback(const slam_car::stm_to_pc &msg)
//{
//    odom_lastest_sample_time = ros::Time::now();

//    slam_car::stm_to_pc msg_temp = msg; //这里不能使用指针，要进行深拷贝
//    // initial position
//    pose_x = msg_temp.coord_x_to_pc;
//    pose_y = msg_temp.coord_y_to_pc;
//    pose_th =msg_temp.z_angle_to_pc;
//    velocity_th = msg_temp.velocity_vth_to_pc;
//}

//void Sample_Data::image_sub_callback(const sensor_msgs::ImageConstPtr& msg)
//{
//    static unsigned int comein_counter=0;
//    image_lastest_sample_time = ros::Time::now();

//    /** @todo 时间同步问题*/
//    if(odom_lastest_sample_time.isZero())
//    {
//        cout<<"waiting for the odometry data..."<<endl;
//        return;
//    }
////    else
////        cout<<(odom_lastest_sample_time-start_sample_time).toSec()<<" "<<(image_lastest_sample_time-start_sample_time).toSec()<<endl;

//    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);

//    cv::Mat image = cv_ptr->image;

//    write_odom_file<<comein_counter<<" "<<
//                     (odom_lastest_sample_time-start_sample_time).toSec()<<" "<<
//                     (image_lastest_sample_time-start_sample_time).toSec()<<" "<<
//                     pose_x*1000.0<<" "<<pose_y*1000.0<<" "<<pose_th<<"\n";

//    /** @todo 测试时间，决定是否放到线程里去做*/
//    ros::Time time_before = ros::Time::now();
//    cv::imwrite(image_store_path+std::to_string(comein_counter)+".png", image);
//    comein_counter++;
//    if((ros::Time::now()-time_before).toSec() > 0.001)
//        cout<<(ros::Time::now()-time_before).toSec()<<endl;

//    cv::imshow("image", image);
//    cv::waitKey(1);
//}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"sample_data");
    Sample_Data obj;
    obj.spin();
    return 0;
}
