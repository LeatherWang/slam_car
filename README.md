

# 1. Prerequisites

`sudo apt-get install ros-indigo-ecl`

`sudo apt-get install ros-indigo-yocs-msgs`

# 2. run command
Sample data from wheel encoder and camera by the node `sample_data` automaticly and in order to move the robot you should set the next goal by using the tool `2D Nav Goal` in rviz.

`roslaunch slam_car navi_only_use_odom.launch`

`roslaunch slam_car sample_data.launch`


Sample data from wheels encoder and camera by the node `sample_data` automaticly and the robot can move automaticly, edit the file `navi_waypoints.yaml`, you can set you own waypoints trajectory.

`roslaunch slam_car navi_only_use_odom.launch`

`roslaunch slam_car wps_navi.launch`

`roslaunch slam_car sample_data.launch`


In order to control the robot remotely, you can configure the host and slave in ROS

in slave: 

`roslaunch slam_car navi_only_use_odom_slave.launch`

`roslaunch slam_car sample_data.launch`

in host:

`roslaunch slam_car rviz_display.launch`

