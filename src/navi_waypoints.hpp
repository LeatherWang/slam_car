/**
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#ifndef YOCS_WAYPOINT_NAVI_HPP_
#define YOCS_WAYPOINT_NAVI_HPP_

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <yocs_msgs/NavigationControl.h>
#include <yocs_msgs/NavigationControlStatus.h>
#include <yocs_msgs/TrajectoryList.h>
#include <yocs_msgs/WaypointList.h>
#include "common.hpp"
#include "geometry.hpp"

namespace slam_car
{

/*
 * TODO
 *  * think about how to best visualise the waypoint(s)/trajectory(ies) which are being executed
 *  * add RViz interface to yocs_waypoint_provider
 */

class WaypointsGoalNode
{
public:
    WaypointsGoalNode();
    ~WaypointsGoalNode();

    void spin();
    void waypointsCB(const yocs_msgs::WaypointList::ConstPtr& wps_msg);
    void trajectoriesCB(const yocs_msgs::TrajectoryList::ConstPtr& trajs_msg);
    void navCtrlCB(void);

private:
    ros::NodeHandle n;

    const geometry_msgs::PoseStamped NOWHERE;

    enum { NONE = 0,
           GOAL,
           LOOP
         } mode_;

    enum { IDLE = 0,
           START,
           ACTIVE,
           COMPLETED
         } state_;

    double      frequency_;
    double      close_enough_;
    std::string robot_frame_;
    std::string world_frame_;

    std::vector<geometry_msgs::PoseStamped>           waypoints_;
    std::vector<geometry_msgs::PoseStamped>::iterator waypoints_it_;

    geometry_msgs::PoseStamped goal_;

    yocs_msgs::WaypointList wp_list_;
    yocs_msgs::TrajectoryList traj_list_;

    tf::TransformListener tf_listener_;

    ros::Subscriber    waypoints_sub_;
    ros::Subscriber    trajectories_sub_;

    ros::Publisher  pub_goal_waypoint;
    std::string goal_name;

    void resetWaypoints();
};

} // namespace yocs

#endif /* YOCS_WAYPOINT_NAVI_HPP_ */
