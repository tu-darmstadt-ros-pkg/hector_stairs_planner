#ifndef HECTOR_STAIRS_PLANNER_DISRUPTION_TEST_H
#define HECTOR_STAIRS_PLANNER_DISRUPTION_TEST_H

#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/GetLinkState.h>
#include <geometry_msgs/PoseStamped.h>

namespace hector_stairs_planner_disruption_test{

    class HectorStairsPlannerDisruptionTest {

    public:
         HectorStairsPlannerDisruptionTest();
         virtual ~HectorStairsPlannerDisruptionTest();

    protected:
          ros::NodeHandle nh_;
          ros::ServiceClient body_wrench_srv_client_;
          ros::ServiceClient robot_position_srv_client_;

          ros::Subscriber robot_pos_sub_;
          ros::Subscriber carrot_pos_sub_;

          double desruptionPosX_;
          double desruptionPosY_;
          double toqueZNm_;
          double forceXN_;
          double duration_;

    };
}

#endif
