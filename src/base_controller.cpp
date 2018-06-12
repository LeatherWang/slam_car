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
    ros::Subscriber nav_goal_sub;

    tf::TransformBroadcaster broadcaster;

    const static int rate=100;
    double velocity_x;
    double velocity_th;

    double goal_angle;
    double goal_x;
    double goal_y;

    void init_variables();

    void webserver_callback(const slam_car::pc_to_stm& msg);
    void cmdvel_callback(const geometry_msgs::Twist& msg);
    void nav_velCallback(const geometry_msgs::PoseStamped &nav_goal_aux);
};

Base_Control::Base_Control()
{
    init_variables();
    pub_motor = n.advertise<slam_car::pc_to_stm>("stm_motor",100);
    sub_web_server = n.subscribe("web_server_cmd", 1000, &Base_Control::webserver_callback,this);
    sub_cmd_vel = n.subscribe("cmd_vel", 1000, &Base_Control::cmdvel_callback,this);
    //nav_goal_sub = n.subscribe("move_base_simple/goal",10, &Base_Control::nav_velCallback, this);
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

        // 发布tf转换树: base_goal_pose
   /*     geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_goal_pose";

        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = goal_x;
        odom_trans.transform.translation.y = goal_y;
        odom_trans.transform.translation.z = 0.0; //height=0
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(goal_angle);

        broadcaster.sendTransform(odom_trans);
*/
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
    double temp = msg_temp.linear.x/1.0; //5.0是缩放系数
    if(temp > 0.4)
        pc_to_stm_data.vel_x_to_stm = 0.4;
    else if(temp < -0.4)
        pc_to_stm_data.vel_x_to_stm = -0.4;
    else
        pc_to_stm_data.vel_x_to_stm = temp;

    pc_to_stm_data.vel_y_to_stm = 0.0;

    //vz:-10~10rad/s
    pc_to_stm_data.z_angle_vel_to_stm = msg_temp.angular.z*1.0; //2.0是缩放系数

    pub_motor.publish(pc_to_stm_data);
}

void Base_Control::nav_velCallback(const  geometry_msgs::PoseStamped &nav_goal_aux)
{
    geometry_msgs::PoseStamped nav_goal_temp = nav_goal_aux;

    double q1 = nav_goal_temp.pose.orientation.x;
    double q2 = nav_goal_temp.pose.orientation.y;
    double q3 = nav_goal_temp.pose.orientation.z;
    double q0 = nav_goal_temp.pose.orientation.w;

    goal_angle = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1);
    goal_x = nav_goal_temp.pose.position.x;
    goal_y = nav_goal_temp.pose.position.y;

    ROS_INFO("set goal, x:%lf, y:%lf, angle:%lf",nav_goal_temp.pose.position.x,nav_goal_temp.pose.position.y, goal_angle*57.2958);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"base_controller");
    Base_Control obj;
    obj.spin();
}
