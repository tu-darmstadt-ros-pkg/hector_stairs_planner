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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <dynamic_reconfigure/server.h>
#include <hector_change_map/HectorChangeMapConfig.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <hazard_model_msgs/HazardModel.h>
#include <grid_map_core/GridMap.hpp>

namespace hector_change_map {

struct LayerInformation {
  nav_msgs::OccupancyGrid original_map, traversability_map, current_map;
  grid_map::GridMap grid_map;
  int publisher_index = -1; // inside static_layer_publishers_, invalid if negative
};

class HectorChangeMap {

public:
 HectorChangeMap();
 virtual ~HectorChangeMap();

protected:
  ros::NodeHandle nh_;

  ros::Publisher map_pub_;
  ros::Publisher original_map_pub_;
  ros::Publisher traversability_map_pub_;
  ros::Publisher initial_pose_pub_;
  ros::Publisher map_list_pub_;
  ros::Publisher reset_pub_;

  ros::Subscriber change_layer_sub_;
  ros::Subscriber initial_pose_2D_sub_;
  ros::Subscriber robot_pose_sub_;
  ros::Subscriber hazard_model_sub_;
  
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<LayerInformation> all_layer_information_;
  std::vector<ros::Publisher> static_layer_publishers_;
  
  std::string frame_id_;
  double robot_layer_distance_threshold_;
  
  int current_robot_layer_;
  
  void changeCurrentLayer(int new_layer);

  void ChangeLayerCB(const hector_change_layer_msgs::Change_layer_msg layer_msg);
  void InitialPose2DCB(const geometry_msgs::PoseWithCovarianceStamped initial_pose_2D);
  void RobotPoseChangedCB(const geometry_msgs::PoseStamped::ConstPtr& pose);
  void HazardModelCB(const hazard_model_msgs::HazardModel& model);

  void publishMapForCurrentLayer(bool original = true);
  void publishResetSignal() const;
  
  nav_msgs::OccupancyGrid loadMap(std::string file_to_load);
  
  void publishMapStaticTFInfo(std::string map_frame, tf::Vector3 translation, tf::Quaternion rotation);
  
  void addStaticMapPublisher(LayerInformation& layer, std::string topic);
  
  int getLayerFromPose(geometry_msgs::PoseStamped const& pose);

  //Dynamic reconfigure
  dynamic_reconfigure::Server<hector_change_map::HectorChangeMapConfig> dynamic_recf_server;
  dynamic_reconfigure::Server<hector_change_map::HectorChangeMapConfig>::CallbackType dynamic_recf_type;
  void dynamic_recf_cb(hector_change_map::HectorChangeMapConfig &config, uint32_t level);
};

} // namespace hector_change_map

#endif
