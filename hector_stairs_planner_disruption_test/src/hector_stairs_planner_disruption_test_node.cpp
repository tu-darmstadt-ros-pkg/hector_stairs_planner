#include <ros/ros.h>
#include <hector_stairs_planner_disruption_test/hector_stairs_planner_disruption_test.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_stairs_planner_disruption_test_node");

  ROS_INFO("Starting Hector stairs planner disruption Node");
  hector_stairs_planner_disruption_test::HectorStairsPlannerDisruptionTest obj;
  ros::spin();
  exit(0);
}
