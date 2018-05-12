/*
   Way point Manager

   LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE

   Author : Jihoon Lee
   Date   : Dec 2013
 */

#include "waypoint_provider.hpp"
#include "yaml_parser.hpp"
#include <yocs_msgs/WaypointList.h>
#include "navi_waypoints.hpp"

namespace slam_car
{
WaypointsGoalNode::WaypointsGoalNode()
{
    mode_ = NONE;
    state_ = IDLE;

    ros::NodeHandle pnh("~");
    pnh.param("frequency",      frequency_,     5.0);
    pnh.param("close_enough",   close_enough_,  0.1);  /** @attention 距离阈值->开启下waypoint的导航*/
    pnh.param("robot_frame",    robot_frame_,    std::string("/base_footprint"));
    pnh.param("world_frame",    world_frame_,    std::string("/odom")); /** @attention /map修改为/odom */

    // reset goal way points
    waypoints_.clear();
    waypoints_it_ = waypoints_.end();

    waypoints_sub_  = n.subscribe("/waypoints",  1, &WaypointsGoalNode::waypointsCB, this);
    trajectories_sub_  = n.subscribe("/trajectories",  1, &WaypointsGoalNode::trajectoriesCB, this);
    pub_goal_waypoint = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",50);
}

WaypointsGoalNode::~WaypointsGoalNode(){}

void WaypointsGoalNode::waypointsCB(const yocs_msgs::WaypointList::ConstPtr& wps_msg)
{
    wp_list_ = *wps_msg;
    ROS_INFO_STREAM("Received " << wp_list_.waypoints.size() << " way points.");
}

void WaypointsGoalNode::trajectoriesCB(const yocs_msgs::TrajectoryList::ConstPtr& trajs_msg)
{
    static bool once = false;
    traj_list_ = *trajs_msg;

    if(!once)
    {
        goal_name = "wps_oval";
        navCtrlCB();
        once = true;
    }

    ROS_INFO_STREAM("Received " << traj_list_.trajectories.size() << " trajectories.");
}

void WaypointsGoalNode::resetWaypoints()
{
    ROS_DEBUG("Full reset: clear markers, delete waypoints and goal and set state to IDLE");
    waypoints_.clear();
    waypoints_it_ = waypoints_.end();
    goal_  = NOWHERE;
    mode_  = NONE;
}

void WaypointsGoalNode::navCtrlCB(void)
{
    bool goal_found = false;
    if ((state_ == IDLE) || (state_ == COMPLETED))
    {
        resetWaypoints();
        if (!goal_found)
        {
            for (unsigned int traj = 0; traj < traj_list_.trajectories.size(); ++traj)
            {
                // 如果轨迹图中包含该goal_name，则接收到的整个轨迹图，比如:go_to_room1
                if (goal_name == traj_list_.trajectories[traj].name)
                {
                    for (unsigned int wp = 0; wp < traj_list_.trajectories[traj].waypoints.size(); ++wp)
                    {
                        geometry_msgs::PoseStamped pose;
                        pose.header = traj_list_.trajectories[traj].waypoints[wp].header;
                        pose.pose = traj_list_.trajectories[traj].waypoints[wp].pose;
                        waypoints_.push_back(pose);
                    }
                    waypoints_it_ = waypoints_.begin();
                    ROS_INFO_STREAM("Prepared to navigate along the trajectory '" << goal_name << "'.");
                    ROS_INFO_STREAM("# of way points = " << waypoints_.size());
                    goal_found = true;
                    break;
                }
            }
        }

        // 如果发现goal_name，则职位相关状态
        if (goal_found)
        {
            state_ = START;
            mode_  = GOAL;
        }
        else
            ROS_WARN_STREAM("Could not find provided way point or trajectory.");
    }
    else
        ROS_WARN_STREAM("Cannot start way point/trajectory execution, because navigator is currently active. "
                        << "Please stop current activity first.");
}

void WaypointsGoalNode::spin()
{
    geometry_msgs::PoseStamped current_goal;
    ros::Rate rate(frequency_);
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();

        if (state_ == START)
        {
            //循环路径
            if (mode_ == LOOP)
            {
                if (waypoints_it_ == waypoints_.end())
                    waypoints_it_ = waypoints_.begin();
            }

            if (waypoints_it_ < waypoints_.end())
            {
                current_goal.header.stamp = ros::Time::now();
                current_goal.header.frame_id = waypoints_it_->header.frame_id;
                current_goal.pose.position = waypoints_it_->pose.position;
                current_goal.pose.orientation = waypoints_it_->pose.orientation;

                ROS_INFO("New goal: %.2f, %.2f, %.2f",
                         current_goal.pose.position.x, current_goal.pose.position.y,
                         tf::getYaw(current_goal.pose.orientation));
                // 发布目标
                pub_goal_waypoint.publish(current_goal);
                state_ = ACTIVE;
            }
            else
            {
                ROS_ERROR_STREAM("Cannot start execution. Already at the last way point.");
                state_ = IDLE;
            }
        }
        else if (state_ == ACTIVE)
        {
            // When close enough to current goal (except for the final one!), go for the
            // next waypoint, so we avoid the final slow approach and subgoal obsession
            // 当非常接近当前目标时，为避免缓慢运动，直接前往下一个waypoint
            if (waypoints_it_ < (waypoints_.end() - 1))
            {
                tf::StampedTransform robot_gb, goal_gb;
                try
                {
                    tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gb);
                }
                catch (tf::TransformException& e)
                {
                    ROS_WARN("Cannot get tf %s -> %s: %s", world_frame_.c_str(), robot_frame_.c_str(), e.what());
                    continue;
                }

                slam_car::pose2tf(current_goal.pose, goal_gb); /** @attention 转换*/
                double distance = slam_car::distance2D(robot_gb, goal_gb);
                if (distance <= close_enough_)
                {
                    waypoints_it_++;
                    state_ = START;
                    ROS_INFO("Close enough to current goal (%.2f <= %.2f m).", distance, close_enough_);
                    ROS_INFO_STREAM("Requesting next way point.");
                }
                else
                {
                    // keep going until get close enough
                }
            }
            else
            {
                // keep going, since we approaching last way point
            }
        }
    }
}
} // namespace

using namespace slam_car;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "navi_waypoints");
    WaypointsGoalNode navi_waypoints;
    navi_waypoints.spin();
    return 0;
}

