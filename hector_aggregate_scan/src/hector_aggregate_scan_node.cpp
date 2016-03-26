#include <ros/ros.h>
#include <hector_aggregate_scan/hector_aggregate_scan.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_aggregate_scan_node");

  ROS_INFO("Starting Hector Aggregate Scan Node");
  hector_aggregate_scan::HectorAggregateScan obj;
  ros::spin();
  exit(0);
}
