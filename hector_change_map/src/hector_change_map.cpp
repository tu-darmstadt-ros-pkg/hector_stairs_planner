#include <ros/ros.h>
#include <hector_change_map/hector_change_map.h>
#include <XmlRpcException.h>
#include <hector_change_layer_msgs/MapLayerList.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <std_msgs/String.h>

namespace hector_change_map{

HectorChangeMap::HectorChangeMap(): tf_listener_(tf_buffer_), current_robot_layer_(-1) {
  ROS_INFO ("change map node started");
  ros::NodeHandle pnh("~");

  hector_change_layer_msgs::MapLayerList layers_msg;

  pnh.param("frame_id", frame_id_, std::string("world"));
  try {
    XmlRpc::XmlRpcValue map_list;
    pnh.getParam("maps", map_list);
    
    //load maps
    for (int i = 0; i < map_list.size(); i++) {
      XmlRpc::XmlRpcValue &map_params = map_list[i];

      LayerInformation layer;
      layer.original_map = loadMap(static_cast<std::string>(map_params["file"]));
      if (map_params.hasMember("file_traversability"))
        layer.traversability_map = loadMap(static_cast<std::string>(map_params["file_traversability"]));
      else
        layer.traversability_map = layer.original_map;
      layer.current_map = layer.original_map;
      
      if(map_params.hasMember("publish_map")) {
        XmlRpc::XmlRpcValue& publish_params = map_params["publish_map"];
        std::string map_frame = publish_params["frame_id"];
        layer.current_map.header.frame_id = layer.original_map.header.frame_id = map_frame;
        
        std::string topic = publish_params["topic"];
        layers_msg.map_topics.push_back(topic);
        
        if(publish_params.hasMember("publish_frame")) {
          XmlRpc::XmlRpcValue& frame_params = publish_params["publish_frame"];
          XmlRpc::XmlRpcValue& trans_params = frame_params["translation"];
          XmlRpc::XmlRpcValue& rot_params = frame_params["rotation"];
          tf::Vector3 trans(trans_params[0], trans_params[1], trans_params[2]);
          tf::Quaternion rot(rot_params[0], rot_params[1], rot_params[2], rot_params[3]);
          publishMapStaticTFInfo(map_frame, trans, rot);
        }
  
        addStaticMapPublisher(layer, topic);
      }
      all_layer_information_.push_back(layer);
    }
  } catch(XmlRpc::XmlRpcException const& e) {
    ROS_WARN_STREAM("change map node: could not load 'maps' parameters: " << e.getMessage() << ". Doing nothing.");
    return;
  }
  
  if(all_layer_information_.empty()) {
    ROS_WARN("change map node: no maps were specified. Doing nothing.");
    return;
  }

  map_pub_ =  nh_.advertise<nav_msgs::OccupancyGrid>("/map", 100, true);
  original_map_pub_= nh_.advertise<nav_msgs::OccupancyGrid>("/original_map", 10, true);
  traversability_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/traversability_map", 10, true);
  initial_pose_pub_= nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100);
  map_list_pub_ = nh_.advertise<hector_change_layer_msgs::MapLayerList>("/layer_list", 10, true);
  reset_pub_ = nh_.advertise<std_msgs::String>("/reset", 10, true);
  
  change_layer_sub_ = nh_.subscribe<hector_change_layer_msgs::Change_layer_msg>("/hector_change_map/change_layer", 1, &HectorChangeMap::ChangeLayerCB, this);
  initial_pose_2D_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_2D", 1, &HectorChangeMap::InitialPose2DCB, this);
  robot_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/robot_pose", 1, &HectorChangeMap::RobotPoseChangedCB, this);
  
  dynamic_recf_type = boost::bind(&HectorChangeMap::dynamic_recf_cb, this, _1, _2);
  dynamic_recf_server.setCallback(dynamic_recf_type);
  
  pnh.param("layer_distance_threshold", robot_layer_distance_threshold_, 0.5);
  map_list_pub_.publish(layers_msg);

  double map_publish_period;
  if (pnh.getParam("map_publish_period", map_publish_period)) {
    map_pub_timer_ = pnh.createTimer(ros::Duration(map_publish_period), &HectorChangeMap::MapPubTimerCB, this);
  }
}

HectorChangeMap::~HectorChangeMap()
{
  
}

void HectorChangeMap::ChangeLayerCB(const hector_change_layer_msgs::Change_layer_msg layer_msg){
  changeCurrentLayer(layer_msg.layerNumber);
}

void HectorChangeMap::InitialPose2DCB(const geometry_msgs::PoseWithCovarianceStamped initial_pose_2D){
  //initial pose 2D <=> z always 0
  geometry_msgs::PoseWithCovarianceStamped initial_pose;
  initial_pose= initial_pose_2D;
  initial_pose.pose.pose.position.z= all_layer_information_.at(current_robot_layer_).current_map.info.origin.position.z;
  initial_pose_pub_.publish(initial_pose);
}

void HectorChangeMap::publishMapForCurrentLayer(bool original)
{
  //stair traversal layer are always numberd uneven
  if(!all_layer_information_.empty())
  {
    //publish map for layer
    ROS_DEBUG("provide map for layer, %i", current_robot_layer_);
    if(current_robot_layer_ >= 0 && current_robot_layer_< static_cast<int>(all_layer_information_.size()))
    {
      const LayerInformation& layer = all_layer_information_.at(static_cast<size_t>(current_robot_layer_));
      nav_msgs::OccupancyGrid map = layer.current_map;
      nav_msgs::OccupancyGrid original_map = layer.original_map;
      nav_msgs::OccupancyGrid traversibility_map = layer.traversability_map;

      try
      {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = map.header.frame_id;
        pose.header.stamp = ros::Time::now();
        pose.pose = map.info.origin;
        pose = tf_buffer_.transform(pose, frame_id_, ros::Duration(1));
        map.info.origin = pose.pose;
      }
      catch (const tf2::TransformException& ex)
      {
        ROS_WARN_STREAM("[HectorChangeMap] Could not transform map pose from frame '" << map.header.frame_id << "' to frame '" << frame_id_ << "': " <<ex.what());
      }

      map.header.frame_id = frame_id_;
      map_pub_.publish(map);

      if (original)
      {
        original_map.header = map.header;
        original_map.info = map.info;
        original_map_pub_.publish(original_map);
        traversibility_map.header = map.header;
        traversibility_map.info = map.info;
        traversability_map_pub_.publish(traversibility_map);
      }
    }
    else
    {
      ROS_ERROR("[HectorChangeMap] no map available for layer %i", current_robot_layer_);
    }
  }
}

void HectorChangeMap::publishResetSignal() const
{
  reset_pub_.publish(std_msgs::String());
}

nav_msgs::OccupancyGrid HectorChangeMap::loadMap(std::string file_to_load)
{
  double res;
  std::string mapfname = "";
  double origin[3];
  int negate;
  double occ_th, free_th;
  
  YAML::Node file = YAML::LoadFile(file_to_load.c_str());
  
  res= file["resolution"].as<double>();
  negate=file["negate"].as<int>();
  occ_th=file["occupied_thresh"].as<double>();
  free_th=file["free_thresh"].as<double>();
  
  origin[0]=file["origin"][0].as<double>();
  origin[1]=file["origin"][1].as<double>();
  origin[2]=file["origin"][2].as<double>();
  
  mapfname=file["image"].as<std::string>();
  if(mapfname[0] != '/')
  {
    char* fname_copy = strdup(file_to_load.c_str());
    mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
    free(fname_copy);
  }
  
  ROS_INFO_STREAM("[hector_change_map] file loaded: " << file_to_load);
  
  LayerInformation layer_info;
  nav_msgs::GetMap::Response map_resp_;
  double origin_for_mapserver[]= {origin[0], origin[1], origin[3]};
  map_server::loadMapFromFile(&map_resp_, mapfname.c_str(),res,negate,occ_th,free_th, origin_for_mapserver);
  nav_msgs::OccupancyGrid map;
  map=map_resp_.map;
  map.info.origin.position.x= origin[0];
  map.info.origin.position.y= origin[1];
  map.info.origin.position.z= origin[2];
  map.info.map_load_time = ros::Time::now();
  map.header.frame_id = frame_id_;
  map.header.stamp = ros::Time::now();

  return map;
}

void HectorChangeMap::dynamic_recf_cb(hector_change_map::HectorChangeMapConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Callback enterd");
  changeCurrentLayer(config.current_robot_layer);
}

void HectorChangeMap::publishMapStaticTFInfo(std::string map_frame, tf::Vector3 translation, tf::Quaternion rotation) {
  geometry_msgs::TransformStamped static_transformStamped;
  
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = frame_id_;
  static_transformStamped.child_frame_id = map_frame;
  tf::vector3TFToMsg(translation, static_transformStamped.transform.translation);
  tf::quaternionTFToMsg(rotation, static_transformStamped.transform.rotation);

  static_broadcaster_.sendTransform(static_transformStamped);
}

void HectorChangeMap::addStaticMapPublisher(LayerInformation &layer, std::string topic) {
  layer.publisher_index = static_layer_publishers_.size();
  static_layer_publishers_.emplace_back();
  ros::Publisher& pub = static_layer_publishers_.back();
  pub = nh_.advertise<nav_msgs::OccupancyGrid>(topic, 10, true);
  pub.publish(layer.current_map);
}

void HectorChangeMap::RobotPoseChangedCB(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
  // remove leading slash since this is not allowed in tf2
  geometry_msgs::PoseStamped pose = *pose_msg;
  pose.header.frame_id = tf::strip_leading_slash(pose.header.frame_id);
  
  int layer = getLayerFromPose(pose);
  if(layer >= 0 && layer != current_robot_layer_)
    changeCurrentLayer(layer);
}

void HectorChangeMap::changeCurrentLayer(int new_layer) {
  ROS_INFO_STREAM("Received request to change current map to " << new_layer);
  if(new_layer >= 0 && new_layer < static_cast<int>(all_layer_information_.size())) {
    if(current_robot_layer_ != new_layer) {
      current_robot_layer_ = new_layer;
      publishMapForCurrentLayer();
      publishResetSignal();
    } else {
      ROS_INFO_STREAM("Not updating map because requested map is already the current map.");
    }
  } else {
    ROS_ERROR("[hector_change_map] no map available for layer %i", new_layer);
  }
}

void HectorChangeMap::MapPubTimerCB(const ros::TimerEvent& event)
{
  publishMapForCurrentLayer();
}

int HectorChangeMap::getLayerFromPose(geometry_msgs::PoseStamped const &pose) {
  int best_match = -1;
  double best_dist = std::numeric_limits<double>::max();
  
  for(size_t i = 0; i < all_layer_information_.size(); i++) {
    auto const& map = all_layer_information_.at(i).current_map;
    
    try {
      geometry_msgs::PoseStamped pose_transformed = tf_buffer_.transform(pose, map.header.frame_id, ros::Duration(1));
      if(best_match < 0 || std::abs(pose_transformed.pose.position.z) < best_dist) {
        best_match = i;
        best_dist = std::abs(pose_transformed.pose.position.z);
      }
      
    } catch(const tf2::TransformException& ex) {
      ROS_WARN_STREAM("HectorChangeMap: Could not transform robot pose from frame '" << pose.header.frame_id << "' to frame '" << map.header.frame_id << "': " <<ex.what());
    }
  }
  
  if(best_dist <= robot_layer_distance_threshold_) {
    return best_match;
  } else {
    return -1;
  }
}
  
} // namespace hector_change_map{
