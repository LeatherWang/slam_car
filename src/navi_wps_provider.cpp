#include "waypoint_provider.hpp"
#include "yaml_parser.hpp"
#include <yocs_msgs/WaypointList.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navi_wps_provider");
  ros::NodeHandle priv_n("~");
  ros::NodeHandle n;
  slam_car::WaypointProvider* wm;
  yocs_msgs::WaypointList wps;
  yocs_msgs::TrajectoryList trajs;
  std::string filename;

  if(!priv_n.getParam("filename", filename)) {
    ROS_ERROR("Waypoint Provider : filename argument is not set");
    return -1;
  }

  if(!slam_car::loadWaypointsAndTrajectoriesFromYaml(filename, wps, trajs))
  {
    ROS_ERROR("Waypoint Provider : Failed to parse yaml[%s]",filename.c_str());
    return -1;
  }

  wm = new slam_car::WaypointProvider(n, wps, trajs);

  ROS_INFO("Waypoint Provider : Initialized");
  wm->spin();
  ROS_INFO("Waypoint Provider : Bye Bye");

  delete wm;

  return 0;
}
