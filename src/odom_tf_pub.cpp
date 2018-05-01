#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <string>
#include "slam_car/stm_to_pc.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"

namespace slam_car
{

class Sub_Odom_Pub
{
public:
    Sub_Odom_Pub();
    void spin();

private:
    ros::NodeHandle n;
    ros::Subscriber odom_from_stm_sub;
    ros::Publisher odom_pub;

    tf::TransformBroadcaster broadcaster;

    const static int rate = 50; /** @attention 仅仅是为了将stm发来的里程计数据转换标准格式，对频率要求不高*/

    double pose_x;
    double pose_y;
    double pose_th;

    double velocity_x;
    double velocity_y;
    double velocity_th;

    ros::Time current_time_for_calVel,last_time_for_calVel;

    void callback(const slam_car::stm_to_pc & msg);
    void init_variables();
    void update();
    double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
};

Sub_Odom_Pub::Sub_Odom_Pub()
{
    init_variables();

    ROS_INFO("Started odometry computing node");

    //subscribe the topic
    odom_from_stm_sub = n.subscribe("/odomtry_from_stm",200,&Sub_Odom_Pub::callback,this);

    //publish the topic
    odom_pub = n.advertise<nav_msgs::Odometry>("/odom",50);
}

void Sub_Odom_Pub::init_variables()
{
    pose_x = 0.0;
    pose_y = 0.0;
    pose_th = 0.0;

    velocity_x = 0.0;
    velocity_y = 0.0;
    velocity_th = 0.0;

    current_time_for_calVel = ros::Time::now();
    last_time_for_calVel = ros::Time::now();
}

//Spin function
void Sub_Odom_Pub::spin()
{
    /** @attention 设置较低的rate，以降低stm发给pc的里程计数据的频率*/
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        update();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Sub_Odom_Pub::update()
{
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
    odom.twist.twist.linear.x = velocity_x;
    odom.twist.twist.linear.y = velocity_y;
//	odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = velocity_th;

    // publishing the odometry and the new tf
    broadcaster.sendTransform(odom_trans);
    odom_pub.publish(odom);
}

double Sub_Odom_Pub::KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;//TODO--Leather
   double x_mid = x_last;
   double x_now;
   static double p_last=1;//TODO--Leather
   double p_mid;
   double p_now;
   double kg;

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q为过程噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter系数，R为观测噪声协方差
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值

   p_now=(1-kg)*p_mid;//最优值对应的covariance，后验估计协方差
   p_last = p_now; //更新covariance
   x_last = x_now; //更新系统状态值
   return x_now;
}

void Sub_Odom_Pub::callback(const slam_car::stm_to_pc &msg)
{
    static double last_pose_x,last_pose_y,last_pose_th;
    static bool first_come_in=true;
    current_time_for_calVel = ros::Time::now();
    slam_car::stm_to_pc msg_temp = msg; //这里不能使用指针，要进行深拷贝

    // initial position
    pose_x = msg_temp.coord_x_to_pc;
    pose_y = msg_temp.coord_y_to_pc;
    pose_th =msg_temp.z_angle_to_pc;
    velocity_th = msg_temp.velocity_vth_to_pc;

///    std::cout<<KalmanFilter(velocity_th, 0.000001, 0.0001)<<std::endl;

    // velocity
    if(!first_come_in)
    {
        double dt = (current_time_for_calVel-last_time_for_calVel).toSec();

        velocity_x = (pose_x - last_pose_x)/dt; /** @todo 这里的速度相对谁的?*/
        velocity_y = (pose_y - last_pose_y)/dt;
        //std::cout<<"x:"<<velocity_x <<"y"<<velocity_y <<"z:"<<velocity_th<<std::endl;
    }
    else
        first_come_in = false;

    last_pose_x = pose_x;
    last_pose_y = pose_y;
    last_pose_th = pose_th;
    last_time_for_calVel = current_time_for_calVel;
}

} //namespace

using namespace slam_car;
int main(int argc,char **argv)
{
    ros::init(argc,argv,"odom_tf_pub");
    Sub_Odom_Pub obj;
    obj.spin();
    return 0;
}
