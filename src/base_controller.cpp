#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "slam_car/pc_to_stm.h"

class Base_Control
{
public:
    Base_Control();
    void spin();

private:
    ros::NodeHandle n;
    ros::Publisher pub_motor;
    ros::Subscriber sub_web_server;
    ros::Subscriber sub_cmd_vel;

    const static int rate=50;
    double velocity_x;
    double velocity_th;

    void init_variables();

    void webserver_callback(const slam_car::pc_to_stm& msg);
    void cmdvel_callback(const geometry_msgs::Twist& msg);
};

Base_Control::Base_Control()
{
    init_variables();
    pub_motor = n.advertise<slam_car::pc_to_stm>("stm_motor",100);
    sub_web_server = n.subscribe("web_server_cmd", 1000, &Base_Control::webserver_callback,this);
    sub_cmd_vel = n.subscribe("cmd_vel", 1000, &Base_Control::cmdvel_callback,this);
}

void Base_Control::init_variables()
{
    velocity_x = 0.0;
    velocity_th = 0.0;
}

void Base_Control::spin()
{
    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Base_Control::webserver_callback(const slam_car::pc_to_stm& msg)
{
    //slam_car::pc_to_stm msg_temp = msg;
    /** @todo */
}

void Base_Control::cmdvel_callback(const geometry_msgs::Twist& msg)
{
    geometry_msgs::Twist msg_temp = msg;

    slam_car::pc_to_stm pc_to_stm_data;

    // vx:-0.4~0.4m/s
    double temp = msg_temp.linear.x/5.0; //5.0是缩放系数
    if(temp > 0.4)
        pc_to_stm_data.vel_x_to_stm = 0.4;
    else if(temp < -0.4)
        pc_to_stm_data.vel_x_to_stm = -0.4;
    else
        pc_to_stm_data.vel_x_to_stm = temp;

    pc_to_stm_data.vel_y_to_stm = 0.0;

    //vz:-10~10rad/s
    pc_to_stm_data.z_angle_vel_to_stm = msg_temp.angular.z*2.0; //2.0是缩放系数

    pub_motor.publish(pc_to_stm_data);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"base_controller");
    Base_Control obj;
    obj.spin();
}
