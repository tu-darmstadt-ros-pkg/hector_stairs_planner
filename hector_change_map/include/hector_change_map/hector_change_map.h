#ifndef HECTOR_CHANHGE_MAP_H
#define HECTOR_CHANHGE_MAP_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <hector_stair_detection_msgs/BorderAndOrientationOfStairs.h>
#include <hector_stair_detection_msgs/PositionAndOrientaion.h>
#include <math.h>
#include <tf/tf.h>
#include <hector_move_base_msgs/MoveBaseActionGoal.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <hector_change_layer_msgs/Change_layer_msg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <fstream>
#include <libgen.h>
#include <yaml-cpp/yaml.h>
#include <map_server/image_loader.h>

namespace hector_change_map{

struct staircase {
    Eigen::Vector3f bottom1;
    Eigen::Vector3f bottom2;
    Eigen::Vector3f top1;
    Eigen::Vector3f top2;
    Eigen::Vector3f direction;
    float yaw;
    float pitch;
    float number_of_points;
    geometry_msgs::PoseStamped orientation;
};

struct layer_information {
    nav_msgs::OccupancyGrid map;
    std::vector<staircase> staircases;
};

    class HectorChangeMap {

    public:
         HectorChangeMap();
         virtual ~HectorChangeMap();

    protected:
          ros::NodeHandle nh_;

          ros::Publisher map_pub_;
          ros::Publisher stairs_information_reset_pub_;
          ros::Publisher stairs_information_pub_;
          ros::Publisher stairs_information_pub_debug_;

          ros::Subscriber change_layer_sub_;

          tf::TransformListener tf_listener_;

          std::vector<layer_information> all_layer_information_;

          std::string map_1_file_;
          std::string map_0_file_;
          std::string map_2_file_;
          std::string stairs_info_file_;
          std::string frame_id_;

          int current_robot_layer_;

          void ChangeLayerCB(const hector_change_layer_msgs::Change_layer_msg layer_msg);

          void publishMapForLayer();

          void insertStairs(Eigen::Vector3f bottom1, Eigen::Vector3f bottom2, Eigen::Vector3f top1, Eigen::Vector3f top2, Eigen::Vector3f direction, float yaw, float pitch, float number_of_points, geometry_msgs::PoseStamped orientation, int layer);
          void publishStairsInformationForLayer(int layer);
          void loadMap(std::string file_to_load);
          void loadStairsInfo(std::string file_to_load);

    };
}

#endif
