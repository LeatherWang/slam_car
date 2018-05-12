/*
 * Copyright (c) 2013, Marcus Liebhardt, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cmath>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include "geometry.hpp"
#include "diff_drive_controller.hpp"
#include <ros/console.h>

namespace slam_car
{

bool DiffDrivePoseControllerROS::init()
{
    /** @attention 设置输出debug消息*/
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    enable_controller_subscriber_ = nh_.subscribe("enable", 10, &DiffDrivePoseControllerROS::enableCB, this);
    disable_controller_subscriber_ = nh_.subscribe("disable", 10, &DiffDrivePoseControllerROS::disableCB, this);
    control_velocity_subscriber_ = nh_.subscribe("control_max_vel", 10, &DiffDrivePoseControllerROS::controlMaxVelCB,
                                                 this);
    // 发布的控制底盘的指令
    command_velocity_publisher_ = nh_.advertise<slam_car::pc_to_stm>("/stm_motor", 10);
    pose_reached_publisher_ = nh_.advertise<std_msgs::Bool>("pose_reached", 10);

    // retrieve configuration parameters
    base_frame_name_ = "base_footprint";
    if (!nh_.getParam("base_frame_name", base_frame_name_))
    {
        ROS_WARN_STREAM(
                    "Couldn't retrieve parameter 'base_frame_name' from parameter server! Using default '" << base_frame_name_ << "'. [" << name_ <<"]");
    }

    goal_frame_name_ = "base_goal_pose";
    if (!nh_.getParam("goal_frame_name", goal_frame_name_))
    {
        ROS_WARN_STREAM(
                    "Couldn't retrieve parameter 'goal_frame_name' from parameter server! Using default '" << goal_frame_name_ << "'. [" << name_ <<"]");
    }

    if (!nh_.getParam("v_min", v_min_movement_))
    {
        ROS_WARN_STREAM(
                    "Couldn't retrieve parameter 'v_min' from parameter server! Using default '" << v_min_movement_ << "'. [" << name_ <<"]");
    }
    v_min_ = v_min_movement_;
    if (!nh_.getParam("v_max", v_max_))
    {
        ROS_WARN_STREAM(
                    "Couldn't retrieve parameter 'v_max' from parameter server! Using default '" << v_max_ << "'. [" << name_ <<"]");
    }
    // 一般我们不使能后退运动
    //  v_min_ = -v_max_; //if we also want to enable driving backwards
    if (!nh_.getParam("w_min", w_min_movement_))
    {
        ROS_WARN_STREAM(
                    "Couldn't retrieve parameter 'w_min' from parameter server! Using default '" << w_min_movement_ << "'. [" << name_ <<"]");
    }
    w_max_ = M_PI / 4 * v_max_;
    if (!nh_.getParam("w_max", w_max_))
    {
        ROS_WARN_STREAM(
                    "Couldn't retrieve parameter 'w_max' from parameter server! Using default '" << w_max_ << "'. [" << name_ <<"]");
    }
    w_min_ = -w_max_;

    if (!nh_.getParam("k_1", k_1_))
    {
        ROS_WARN_STREAM(
                    "Couldn't retrieve parameter 'k_1' from parameter server! Using default '" << k_1_ << "'. [" << name_ <<"]");
    }
    if (!nh_.getParam("k_2", k_2_))
    {
        ROS_WARN_STREAM(
                    "Couldn't retrieve parameter 'k_2' from parameter server! Using default '" << k_2_ << "'. [" << name_ <<"]");
    }
    if (!nh_.getParam("beta", beta_))
    {
        ROS_WARN_STREAM(
                    "Couldn't retrieve parameter 'beta' from parameter server! Using default '" << beta_ << "'. [" << name_ <<"]");
    }
    if (!nh_.getParam("lambda", lambda_))
    {
        ROS_WARN_STREAM(
                    "Couldn't retrieve parameter 'lambda' from parameter server! Using default '" << lambda_ << "'. [" << name_ <<"]");
    }
    if (!nh_.getParam("dist_thres", dist_thres_))
    {
        ROS_WARN_STREAM(
                    "Couldn't retrieve parameter 'dist_thres' from parameter server! Using default '" << dist_thres_ << "'. [" << name_ <<"]");
    }
    if (!nh_.getParam("orient_thres", orient_thres_))
    {
        ROS_WARN_STREAM(
                    "Couldn't retrieve parameter 'orient_thres' from parameter server! Using default '" << orient_thres_ << "'. [" << name_ <<"]");
    }
    dist_eps_ = dist_eps_ * 0.2;
    if (!nh_.getParam("dist_eps", dist_eps_))
    {
        ROS_WARN_STREAM(
                    "Couldn't retrieve parameter 'dist_eps' from parameter server! Using default '" << dist_eps_ << "'. [" << name_ <<"]");
    }
    orient_eps_ = orient_thres_ * 0.2;
    if (!nh_.getParam("orient_eps", orient_eps_))
    {
        ROS_WARN_STREAM(
                    "Couldn't retrieve parameter 'orient_eps' from parameter server! Using default '" << orient_eps_ << "'. [" << name_ <<"]");
    }

    ROS_DEBUG_STREAM("Controller initialised with the following parameters: [" << name_ <<"]");
    ROS_DEBUG_STREAM(
                "base_frame_name = " << base_frame_name_ <<", goal_frame_name = " << goal_frame_name_ << " [" << name_ <<"]");
    ROS_DEBUG_STREAM(
                "v_max = " << v_max_ <<", k_1 = " << k_1_ << ", k_2 = " << k_2_ << ", beta = " << beta_ << ", lambda = " << lambda_ << ", dist_thres = " << dist_thres_ << ", orient_thres = " << orient_thres_ <<" [" << name_ <<"]");

    return true;
}

void DiffDrivePoseControllerROS::spinOnce()
{
    if (this->getState())
    {
        ROS_DEBUG_STREAM_THROTTLE(1.0, "Controller spinning. [" << name_ <<"]");
        // determine pose difference in polar coordinates
        if (!getPoseDiff())
        {
            ROS_WARN_STREAM_THROTTLE(1.0, "Getting pose difference failed. Skipping control loop. [" << name_ <<"]");
            return;
        }

        // determine controller output (v, w) and check if goal is reached
        step();

        // set control output (v, w)
        setControlOutput();
        // Logging
        ROS_DEBUG_STREAM_THROTTLE(1.0, "Current state: [" << name_ <<"]");
        ROS_DEBUG_STREAM_THROTTLE(1.0,
                                  "r = " << r_ << ", theta = " << theta_ << ", delta = " << delta_ << " [" << name_ <<"]");
        ROS_DEBUG_STREAM_THROTTLE(1.0, "cur = " << cur_ << ", v = " << v_ << ", w = " << w_ << " [" << name_ <<"]");
    }
    else
    {
        ROS_DEBUG_STREAM_THROTTLE(3.0, "Controller is disabled. Idling ... [" << name_ <<"]");
    }
}

bool DiffDrivePoseControllerROS::getPoseDiff()
{
    // use tf to get information about the goal pose relative to the base
    // 得到目标相对base的位姿
    /** @todo goal_frame_name_是什么意思?*/
    try
    {
        tf_listener_.lookupTransform(base_frame_name_, goal_frame_name_, ros::Time(0), tf_goal_pose_rel_);
    }
    catch (tf::TransformException const &ex)
    {
        ROS_WARN_STREAM_THROTTLE(1.0, "Couldn't get transform from base to goal pose! [" << name_ <<"]");
        ROS_DEBUG_STREAM_THROTTLE(1.0, "tf error: " << ex.what());
        return false;
    }

    // determine distance to goal
    double r = std::sqrt(
                std::pow(tf_goal_pose_rel_.getOrigin().getX(), 2) + std::pow(tf_goal_pose_rel_.getOrigin().getY(), 2));

    // determine orientation of r relative to the base frame
    // r相对base帧的旋转
    double delta = std::atan2(-tf_goal_pose_rel_.getOrigin().getY(), tf_goal_pose_rel_.getOrigin().getX());

    // determine orientation of r relative to the goal frame
    double heading = slam_car::wrapAngle(tf::getYaw(tf_goal_pose_rel_.getRotation()));
    // helper: theta = tf's orientation + delta
    // r相对与目标帧的旋转: theta
    // 目标位姿的朝向
    double theta = heading + delta;

    setInput(r, delta, theta);

    return true;
}

void DiffDrivePoseControllerROS::onGoalReached()
{
    std_msgs::Bool bool_msg;
    bool_msg.data = true;
    pose_reached_publisher_.publish(bool_msg);
}

void DiffDrivePoseControllerROS::setControlOutput()
{
//    geometry_msgs::TwistPtr cmd_vel(new geometry_msgs::Twist());
    slam_car::pc_to_stm pc_to_stm_data;
    if (!pose_reached_)
    {
//        cmd_vel->linear.x = v_;
//        cmd_vel->angular.z = w_;

        // vx:-0.4~0.4m/s
        double temp = v_/1.0; //5.0是缩放系数
        if(temp > 0.4)
            pc_to_stm_data.vel_x_to_stm = 0.4;
        else if(temp < -0.4)
            pc_to_stm_data.vel_x_to_stm = -0.4;
        else
            pc_to_stm_data.vel_x_to_stm = temp;

        pc_to_stm_data.vel_y_to_stm = 0.0;

        //vz:-10~10rad/s
        pc_to_stm_data.z_angle_vel_to_stm = w_*1.0;
    }
    command_velocity_publisher_.publish(pc_to_stm_data);
}

void DiffDrivePoseControllerROS::controlMaxVelCB(const std_msgs::Float32ConstPtr msg)
{
    v_max_ = msg->data;
    //v_min_ = -v_max_; //if we also want to enable driving backwards
    ROS_INFO_STREAM("Maximum linear control velocity has been set to " << v_max_ << ". [" << name_ << "]");
}

void DiffDrivePoseControllerROS::enableCB(const std_msgs::StringConstPtr msg)
{
    if (!msg->data.empty())
    {
        goal_frame_name_ = msg->data;
    }

    if (this->enable())
    {
        ROS_INFO_STREAM("Controller has been enabled [" << name_ << "] with goal frame [" << goal_frame_name_ << "].");
    }
    else
    {
        ROS_INFO_STREAM("Controller was already enabled [" << name_ <<"], now tracking goal frame [" << goal_frame_name_ << "].");
    }
}

void DiffDrivePoseControllerROS::disableCB(const std_msgs::EmptyConstPtr msg)
{
    if (this->disable())
    {
        ROS_INFO_STREAM("Controller has been disabled. [" << name_ <<"]");
    }
    else
    {
        ROS_INFO_STREAM("Controller was already disabled. [" << name_ <<"]");
    }
}

} // namespace


using namespace slam_car;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "diff_drive_controller");
    ros::NodeHandle nh( "~" );
    std::string name_("diff_drive_");
    slam_car::DiffDrivePoseControllerROS controller_(nh, name_);

    /** @attention 修改控制频率*/
    double spin_rate_param = 20;
    if(nh.getParam("spin_rate", spin_rate_param))
    {
        ROS_DEBUG_STREAM("Controller will spin at " << spin_rate_param << " hz. [" << name_ <<"]");
    }
    else
    {
        ROS_WARN_STREAM("Couldn't retrieve parameter 'spin_rate' from parameter server! Using default '"
                        << spin_rate_param << "'. [" << name_ <<"]");
    }

    ros::Rate spin_rate_ = ros::Rate(spin_rate_param);

    bool start_enabled = true;
    //nh.getParam("start_enabled", start_enabled);

    if (controller_.init()) //初始化
    {
        if (start_enabled)
        {
            controller_.enable();
            ROS_INFO_STREAM("Controller will start enabled. [" << name_ <<"]");
        }
        else
        {
            controller_.disable();
            ROS_INFO_STREAM("Controller will start disabled. [" << name_ <<"]");
        }

        ROS_INFO_STREAM("Controller initialised. [" << name_ << "]");
    }
    else
    {
        ROS_ERROR_STREAM("Couldn't initialise controller! Please restart. [" << name_ << "]");
    }

    while (ros::ok())
    {
        controller_.spinOnce();
        spin_rate_.sleep();
    }

    return 0;
}
