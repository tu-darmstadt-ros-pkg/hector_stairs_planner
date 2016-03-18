#include <ros/ros.h>
#include <hector_change_map/hector_change_map.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_change_map_node");

  ROS_INFO("Starting Hector change map Node");
  hector_change_map::HectorChangeMap obj;
  ros::spin();
  exit(0);
}
