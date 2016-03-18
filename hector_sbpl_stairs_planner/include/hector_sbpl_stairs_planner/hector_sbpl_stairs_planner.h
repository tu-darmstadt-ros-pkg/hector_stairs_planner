#ifndef HECTOR_SBPL_STAIRS_PLANNER_H
#define HECTOR_SBPL_STAIRS_PLANNER_H

#include <iostream>
#include <vector>

using namespace std;

/** ROS **/
#include <ros/ros.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

// sbpl headers
#include <sbpl/headers.h>

//global representation
#include <nav_core/base_global_planner.h>

#include <hector_sbpl_flipper_control_env/hector_sbpl_flipper_control_env.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <hector_stair_detection_msgs/BorderAndOrientationOfStairs.h>
#include <eigen3/Eigen/Dense>
#include <pluginlib/class_list_macros.h>
#include <hector_stairs_planner_msgs/Path_segment.h>
#include <hector_stairs_planner_msgs/Path_with_Flipper.h>
#include <std_msgs/Bool.h>

namespace hector_sbpl_stairs_planner{

struct stairs_information {
    double minX;
    double maxX;
    double minY;
    double maxY;
    double minZ;
    double maxZ;
    double yaw;
    double pitch;
    int points_on_line;
    int direction;
    geometry_msgs::PoseStamped pose;
};

struct path_with_flipper {
    std::vector<geometry_msgs::PoseStamped> path;
    float flipperFront;
    float flipperRear;
};

class HectorSbplStairsPlanner : public nav_core::BaseGlobalPlanner{
public:

  /**
   * @brief  Constructor for the SBPLTerrainPlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  HectorSbplStairsPlanner();

  HectorSbplStairsPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros_in);
  /**
   * @brief  Initialization function for the SBPLTerrainPlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros_in);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan);

  bool makeExtendedPlan(const geometry_msgs::PoseStamped& start,
                                                 const geometry_msgs::PoseStamped& goal,
                                                 std::vector<geometry_msgs::PoseStamped>& plan, bool &hasFlipperActions);

  virtual ~HectorSbplStairsPlanner(){};


  int getNumberOfStairs();
  void resetAllStairsInformation();

private:

  void visualizeInititalCostValues();

  bool initialized_;

  SBPLPlanner* planner_;

  costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */

  hector_sbpl_flipper_control_env::HectorSBPLFlipperControlEnv* env_;

  std::string planner_type_; /**< sbpl method to use for planning.  choices are ARAPlanner and ADPlanner */

  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;
  unsigned char sbpl_cost_multiplier_;
  int obst_cost_thresh_;
  double maxFlipperOffsetX_;
  double maxFlipperOffsetY_;
  double pitchTresh_;
  double maxHeightInWorld_;
  double inflationRadius_;

  double allocated_time_; /**< amount of time allowed for search */
  double initial_epsilon_; /**< initial epsilon for beginning the anytime search */

  std::string environment_type_; /** what type of environment in which to plan.  choices are 2D and XYThetaLattice. */
  std::string cost_map_topic_; /** what topic is being used for the costmap topic */

  bool forward_search_; /** whether to use forward or backward search */
  std::string primitive_filename_; /** where to find the motion primitives for the current robot */
  int force_scratch_limit_; /** the number of cells that have to be changed in the costmap to force the planner to plan from scratch even if its an incremental planner */

  hector_stairs_planner_msgs::Path_with_Flipper refined_path_;

  ros::Publisher plan_pub_;
  ros::Publisher refined_plan_pub_;
  ros::Publisher flipperActuationPoints_pub_;
  ros::Time t_lastMapPos_;

  boost::shared_ptr<tf::TransformListener> tf_listener_;
  geometry_msgs::Point map_center_map;

  std::vector<stairs_information> all_stairs_information_;
  visualization_msgs::MarkerArray borders_of_all_stairs_;
  double between_staris_distance_threshold_;
  double same_stairs_distance_threshold_;
  int number_orientaions_;

  ros::Subscriber information_about_stairs_sub_;
  ros::Subscriber stairs_information_reset_sub_;
  ros::Publisher all_stairs_information_border_pub_;
  ros::Publisher all_stairs_information_orientaion_pub_;
  ros::Publisher occupancyGrid_pub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr vis_pc;
  ros::Publisher vis_pc_pub_;

  void resetStairsCB(const std_msgs::Bool reset_msg);

  unsigned char costMapCostToSBPLCost(unsigned char newcost);

  void updateInformationAboutStairs_Callback(hector_stair_detection_msgs::BorderAndOrientationOfStairs information);
  void insertStairsInformation(visualization_msgs::MarkerArray &border_of_stairs, geometry_msgs::PoseStamped &pose_of_stairs, int points_on_line);
  void modifyStairsInformation(visualization_msgs::MarkerArray &border_of_stairs, geometry_msgs::PoseStamped &pose_of_stairs, int modify_stairs_idx, int number_points_on_line);
  int getZComponent(Eigen::Vector2f directionStairs, Eigen::Vector2f minXminY, Eigen::Vector2f maxXminY, Eigen::Vector2f minXmaxY);

  bool pointInStairsFootprint(Eigen::Vector2f checkPoint, Eigen::Vector2f minXminY, Eigen::Vector2f maxXmaxY);
  float sign(Eigen::Vector2f pt, Eigen::Vector2f v1, Eigen::Vector2f v2);
  bool PointInTriangle (Eigen::Vector2f pt, Eigen::Vector2f v1, Eigen::Vector2f v2, Eigen::Vector2f v3);
  float distancePointLine(Eigen::Vector2f checkPointIn, Eigen::Vector2f baseIn, Eigen::Vector2f pointIn);
  void getPitchAndYawFromNearestStaris(float flagPosX, float flagPosY, float &pitch, float &yaw);
  int updateCostInExtendedStairsArea(Eigen::Vector2f base, Eigen::Vector2f minXmaxY, Eigen::Vector2f maxXminY, Eigen::Vector2f maxXmaxY, int angleNumber, int minCost, int maxCost, vector<nav2dcell_t> &changedcellsV);

};
}

#endif
