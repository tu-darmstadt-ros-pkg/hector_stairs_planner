/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Mike Phillips
*********************************************************************/

#include <hector_sbpl_stairs_planner/hector_sbpl_stairs_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>

using namespace std;
using namespace ros;
using namespace hector_sbpl_stairs_planner;

PLUGINLIB_DECLARE_CLASS(hector_sbpl_stairs_planner, HectorSbplStairsPlanner, hector_sbpl_stairs_planner::HectorSbplStairsPlanner, nav_core::BaseGlobalPlanner);

class LatticeSCQ : public StateChangeQuery{
public:
    LatticeSCQ(hector_sbpl_flipper_control_env::HectorSBPLFlipperControlEnv* env, std::vector<nav2dcell_t> const & changedcellsV)
        : env_(env), changedcellsV_(changedcellsV) {
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getPredecessors() const{
        if(predsOfChangedCells_.empty() && !changedcellsV_.empty())
            env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
        return &predsOfChangedCells_;
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getSuccessors() const{
        if(succsOfChangedCells_.empty() && !changedcellsV_.empty())
            env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
        return &succsOfChangedCells_;
    }

    hector_sbpl_flipper_control_env::HectorSBPLFlipperControlEnv * env_;
    std::vector<nav2dcell_t> const & changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
};


HectorSbplStairsPlanner::HectorSbplStairsPlanner(){
    initialized_=false;
};

HectorSbplStairsPlanner::HectorSbplStairsPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros_in) :
    costmap_ros_(NULL), initialized_(false) {
    HectorSbplStairsPlanner::initialize(name, costmap_ros_in);
}

//void HectorSbplStairsPlanner::initialize(std::string name, nav_msgs::OccupancyGrid &grid){
void HectorSbplStairsPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros_in){
    if(!initialized_){
        ros::NodeHandle private_nh("~/"+name);
        ros::NodeHandle nh("~/");
        ros::NodeHandle n("");

        costmap_ros_ = costmap_ros_in;

        ROS_INFO("[hector_stairs_planner] Name is %s", name.c_str());
        tf_listener_.reset(new tf::TransformListener());

        private_nh.param("planner_type", planner_type_, string("ARAPlanner"));
        private_nh.param("allocated_time", allocated_time_, 15.0); //30
        private_nh.param("initial_epsilon",initial_epsilon_,5.0); //15
        nh.param("environment_type", environment_type_, string("HectorSBPLFlipperControlEnv"));
        private_nh.param("forward_search", forward_search_, bool(true));
        private_nh.param("primitive_filename",primitive_filename_,string(""));
        private_nh.param("force_scratch_limit",force_scratch_limit_,500);
        double nominalvel_mpersecs, timetoturn45degsinplace_secs;
        private_nh.param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
        private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 2.0);
        private_nh.param("between_staris_distance_threshold", between_staris_distance_threshold_, 3.0);
        private_nh.param("same_stairs_distance_threshold", same_stairs_distance_threshold_, 1.5);
        private_nh.param("number_orientaions", number_orientaions_, 16);
        private_nh.param("obst_cost_thresh", obst_cost_thresh_, 30);
        private_nh.param("maxFlipperOffsetX", maxFlipperOffsetX_, 0.5);
        private_nh.param("maxFlipperOffsetY", maxFlipperOffsetY_, 0.5);
        private_nh.param("pitchTresh", pitchTresh_, 1.0);
        private_nh.param("maxHeightInWorld", maxHeightInWorld_, 3.5);
        private_nh.param("robot_symmetric", robot_symmetric_, bool(false));

        int lethal_obstacle;
        private_nh.param("lethal_obstacle",lethal_obstacle,100);
        lethal_obstacle_ = (unsigned char) lethal_obstacle;
        inscribed_inflated_obstacle_ = lethal_obstacle_-1;
        sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);

        env_ = new hector_sbpl_flipper_control_env::HectorSBPLFlipperControlEnv();

        std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

        inflationRadius_= costmap_ros_->getLayeredCostmap()->getInscribedRadius();

        vector<sbpl_2Dpt_t> perimeterptsV;
        perimeterptsV.reserve(footprint.size());

        for (size_t i(0); i < footprint.size(); ++i) {
            sbpl_2Dpt_t pt;
            pt.x = footprint[i].x;
            pt.y = footprint[i].y;
            perimeterptsV.push_back(pt);
            ROS_INFO("footprint idx: %i,   x: %f,   y: %f", i, pt.x, pt.y);
        }

        bool ret;
        vis_pc.reset(new pcl::PointCloud<pcl::PointXYZ>());
        vis_pc->header.frame_id="/world";
        try{
            ret = env_->InitializeEnv(costmap_ros_->getCostmap()->getSizeInCellsX(), // width
                                      costmap_ros_->getCostmap()->getSizeInCellsY(), // height
                                      0, // mapdata
                                      0, 0, 0, false, // start (x, y, theta, flipper)
                                      0, 0, 0, false, // goal (x, y, theta, flipper)
                                      0, 0, 0, //goal tolerance
                                      perimeterptsV, costmap_ros_->getCostmap()->getResolution(), nominalvel_mpersecs,
                                      timetoturn45degsinplace_secs, obst_cost_thresh_,
                                      primitive_filename_.c_str(), maxFlipperOffsetX_, maxFlipperOffsetY_, vis_pc, inflationRadius_);
        }
        catch(SBPL_Exception e){
            ROS_ERROR("SBPL encountered a fatal exception!");
            ret = false;
        }

        if(!ret){
            ROS_ERROR("SBPL initialization failed!");
            exit(1);
        }

        //update costs
        for (ssize_t ix(0); ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ++ix)
            for (ssize_t iy(0); iy < costmap_ros_->getCostmap()->getSizeInCellsY(); ++iy)
                env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy)));

        if ("ARAPlanner" == planner_type_){
            ROS_INFO("[hector_stairs_planner] Planning with ARA*");
            planner_ = new ARAPlanner(env_, forward_search_);
        }
        else if ("ADPlanner" == planner_type_){
            ROS_INFO("[hector_stairs_planner] Planning with AD*");
            planner_ = new ADPlanner(env_, forward_search_);
        }
        else{
            ROS_ERROR("[hector_stairs_planner] ARAPlanner and ADPlanner are currently the only supported planners!\n");
            exit(1);
        }

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        refined_plan_pub_=private_nh.advertise<hector_stairs_planner_msgs::Path_with_Flipper>("/hector_sbpl_stairs_planner/refinedPlan", 100, true);
        flipperActuationPoints_pub_=  nh.advertise<visualization_msgs::MarkerArray>("/hector_sbpl_stairs_planner/flipperActuationPoints", 100, true);

        information_about_stairs_sub_= nh.subscribe("/hector_stair_detection/border_and_orientation_of_stairs", 1, &HectorSbplStairsPlanner::updateInformationAboutStairs_Callback, this);
        stairs_information_reset_sub_= nh.subscribe("/change_layer/reset_stairs_information", 1, &HectorSbplStairsPlanner::resetStairsCB, this);

        all_stairs_information_border_pub_=  nh.advertise<visualization_msgs::MarkerArray>("/hector_sbpl_stairs_planner/all_stairs_border", 100, true);
        all_stairs_information_orientaion_pub_= nh.advertise<geometry_msgs::PoseArray> ("/hector_sbpl_stairs_planner/all_stairs_orientaion", 100, true);

        vis_pc_pub_= nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_sbpl_stairs_planner/expandedNodes", 100, true);

        initialized_ = true;

        nav_msgs::OccupancyGrid::Ptr occupancyGrid(new nav_msgs::OccupancyGrid());
        occupancyGrid->header.frame_id="/world";
        occupancyGrid->info.resolution=costmap_ros_->getCostmap()->getResolution();
        occupancyGrid->info.height=costmap_ros_->getCostmap()->getSizeInCellsY();
        occupancyGrid->info.width=costmap_ros_->getCostmap()->getSizeInCellsX();
        occupancyGrid->info.origin.position.x=costmap_ros_->getCostmap()->getOriginX();
        occupancyGrid->info.origin.position.y=costmap_ros_->getCostmap()->getOriginY();
        occupancyGrid->info.origin.position.z=0;
        occupancyGrid->data.resize(costmap_ros_->getCostmap()->getSizeInCellsX() * costmap_ros_->getCostmap()->getSizeInCellsY());

        for(int x=0; x<occupancyGrid->info.width; x++){
            for(int y=0; y<occupancyGrid->info.height; y++){
                occupancyGrid->data.at(x + y * occupancyGrid->info.width)=env_->GetMapCost(x,y);
            }
        }

        occupancyGrid_pub_= nh.advertise<nav_msgs::OccupancyGrid>("/stairs_planner/occupancyGridUpdated", 1, this);

        if(occupancyGrid_pub_>0){
            occupancyGrid_pub_.publish(occupancyGrid);
        }

        ROS_INFO("[hector_stairs_planner] Initialized successfully");
    }
}


//This rescales the costmap according to a rosparam which sets the obstacle cost
unsigned char HectorSbplStairsPlanner::costMapCostToSBPLCost(unsigned char newcost){
    if(newcost == costmap_2d::LETHAL_OBSTACLE || newcost == costmap_2d::NO_INFORMATION)
        return lethal_obstacle_;
    else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        return lethal_obstacle_;
    else
        return 0;
}

bool HectorSbplStairsPlanner::makeExtendedPlan(const geometry_msgs::PoseStamped& start,
                                               const geometry_msgs::PoseStamped& goal,
                                               std::vector<geometry_msgs::PoseStamped>& plan, bool& hasFlipperActions){
    bool ret = makePlan(start, goal, plan);
    if(ret==true){
        if(plan.size()>1){
            hasFlipperActions=true;
        }else{
            hasFlipperActions=false;
        }
        return true;
    }else{
        return false;
    }

}

bool HectorSbplStairsPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                       const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan){
    vis_pc->clear();
    double startX= start.pose.position.x;
    double startY= start.pose.position.y;

    double goalX= goal.pose.position.x;
    double goalY= goal.pose.position.y;

    if(!initialized_){
        ROS_ERROR("Global planner is not initialized");
        return false;
    }

    plan.clear();

    ROS_DEBUG("[sbpl_lattice_planner] getting fresh copy of costmap");
    ROS_DEBUG("[sbpl_lattice_planner] robot footprint cleared");

    ROS_INFO("[hector_sbpl_stairs_planner] getting start point (%g,%g) goal point (%g,%g)",
             startX, startY, goalX, goalY);
    double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
    double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

    if(robot_symmetric_){
    //argos robot is symmetric
    if(fabs(theta_start-theta_goal)> M_PI_2 && fabs(theta_start-theta_goal)< M_PI+M_PI_2 ){
        theta_goal=theta_goal-M_PI;
        if(theta_goal<0){
            theta_goal=theta_goal+2*M_PI;
        }
    }
    }

    //update Heightmap with stairs information; replace stairs with ramps
    int allCount = 0;
    vector<nav2dcell_t> changedcellsV;

    //     update costs, update "normal" cost changes
    for (ssize_t ix(0); ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ++ix){
        for (ssize_t iy(0); iy < costmap_ros_->getCostmap()->getSizeInCellsY(); ++iy){
            unsigned char oldCost = env_->GetMapCost(ix,iy);
            unsigned char newCost = costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy));
            if(oldCost == newCost) continue;
            env_->UpdateCost(ix, iy, newCost);
        }
    }

    Eigen::Vector3f minXminY, minXmaxY, maxXminY, maxXmaxY;
    int startIterationX_d, endIterartionX_d ,startIterationY_d, endIterartionY_d;
ROS_INFO("[hector_stairs_planner] number of stairs: %i", all_stairs_information_.size());
    for(int i=0; i<all_stairs_information_.size(); i++){
        if(all_stairs_information_.at(i).pitch < pitchTresh_){

            minXminY(0)=borders_of_all_stairs_.markers.at(i*4+0).pose.position.x;
            minXminY(1)=borders_of_all_stairs_.markers.at(i*4+0).pose.position.y;
            minXminY(2)=borders_of_all_stairs_.markers.at(i*4+0).pose.position.z;

            minXmaxY(0)=borders_of_all_stairs_.markers.at(i*4+1).pose.position.x;
            minXmaxY(1)=borders_of_all_stairs_.markers.at(i*4+1).pose.position.y;
            minXmaxY(2)=borders_of_all_stairs_.markers.at(i*4+1).pose.position.z;

            maxXminY(0)=borders_of_all_stairs_.markers.at(i*4+2).pose.position.x;
            maxXminY(1)=borders_of_all_stairs_.markers.at(i*4+2).pose.position.y;
            maxXminY(2)=borders_of_all_stairs_.markers.at(i*4+2).pose.position.z;

            maxXmaxY(0)=borders_of_all_stairs_.markers.at(i*4+3).pose.position.x;
            maxXmaxY(1)=borders_of_all_stairs_.markers.at(i*4+3).pose.position.y;
            maxXmaxY(2)=borders_of_all_stairs_.markers.at(i*4+3).pose.position.z;

            startIterationX_d= CONTXY2DISC(minXminY(0) - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
            startIterationY_d= CONTXY2DISC(minXminY(1) - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
            endIterartionX_d= CONTXY2DISC(maxXmaxY(0) - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
            endIterartionY_d= CONTXY2DISC(maxXmaxY(1) - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());

            Eigen::Vector2f p0;
            p0(0)=minXminY(0);
            p0(1)=minXminY(1);
            Eigen::Vector2f p1;
            p1(0)=minXmaxY(0);
            p1(1)=minXmaxY(1);
            Eigen::Vector2f p2;
            p2(0)=maxXminY(0);
            p2(1)=maxXminY(1);
            Eigen::Vector2f p3;
            p3(0)=maxXmaxY(0);
            p3(1)=maxXmaxY(1);

            // param maxHeightInWorld, scale height to ramp
            float maxStairsZ=minXminY(2);
            if(maxStairsZ<minXmaxY(2)){
                maxStairsZ=minXmaxY(2);
            }
            if(maxStairsZ<maxXminY(2)){
                maxStairsZ=maxXminY(2);
            }
            if(maxStairsZ<maxXmaxY(2)){
                maxStairsZ=maxXmaxY(2);
            }
            float minStairsZ=minXminY(2);
            if(minStairsZ<minXmaxY(2)){
                minStairsZ=minXmaxY(2);
            }
            if(minStairsZ<maxXminY(2)){
                minStairsZ=maxXminY(2);
            }
            if(minStairsZ<maxXmaxY(2)){
                minStairsZ=maxXmaxY(2);
            }

            int maxHeigthInGrid=((100-obst_cost_thresh_)*maxStairsZ)/maxHeightInWorld_;
            bool pointOnBorder=false;

            int computedCost= 100;
            int floorOffset=minStairsZ;
            int angleNumber=-1;
            float inflationStairs=0.4;
            inflationStairs=inflationRadius_;
            //update cost in environment
            for(ssize_t ix(startIterationX_d); ix < endIterartionX_d; ++ix) {
                for(ssize_t iy(startIterationY_d); iy < endIterartionY_d; ++iy) {
                    pointOnBorder=false;
                    unsigned char oldCost = env_->GetMapCost(ix,iy);

                    Eigen::Vector2f checkPoint;
                    checkPoint(0)=DISCXY2CONT(ix, costmap_ros_->getCostmap()->getResolution()) + costmap_ros_->getCostmap()->getOriginX();
                    checkPoint(1)=DISCXY2CONT(iy, costmap_ros_->getCostmap()->getResolution()) + costmap_ros_->getCostmap()->getOriginY();
                    if(true){
                        float dist;
                        float lengthStairs;
                        //depend on yaw
                        if(all_stairs_information_.at(i).yaw>=0 && all_stairs_information_.at(i).yaw< M_PI_4  || all_stairs_information_.at(i).yaw>= 2*M_PI-M_PI_4){
                            angleNumber=0;
                            dist= distancePointLine(checkPoint, p3, p2);
                            lengthStairs= std::sqrt(std::pow(p3(0)-p1(0),2)+std::pow(p3(1)-p1(1),2));
                            if(distancePointLine(checkPoint, p3, p1)<=inflationStairs || distancePointLine(checkPoint, p0, p2)<=inflationStairs){
                                pointOnBorder=true; //=> cost going to be heigh
                            }
                        }
                        if(all_stairs_information_.at(i).yaw>=M_PI_4 && all_stairs_information_.at(i).yaw< M_PI_2+M_PI_4){
                            angleNumber=1;
                            dist= distancePointLine(checkPoint, p1, p3);
                            lengthStairs= std::sqrt(std::pow(p0(0)-p1(0),2)+std::pow(p0(1)-p1(1),2));
                            if(distancePointLine(checkPoint, p0, p1)<=inflationStairs || distancePointLine(checkPoint, p2, p3)<=inflationStairs){
                                pointOnBorder=true; //=> cost going to be heigh
                            }
                        }
                        if(all_stairs_information_.at(i).yaw>=M_PI_2+M_PI_4 && all_stairs_information_.at(i).yaw< M_PI+M_PI_4){
                            angleNumber=2;
                            dist= distancePointLine(checkPoint, p1, p0);
                            lengthStairs= std::sqrt(std::pow(p3(0)-p1(0),2)+std::pow(p3(1)-p1(1),2));
                            if(distancePointLine(checkPoint, p3, p1)<=inflationStairs || distancePointLine(checkPoint, p0, p2)<=inflationStairs){
                                pointOnBorder=true; //=> cost going to be heigh
                            }
                        }
                        if(all_stairs_information_.at(i).yaw>=M_PI+M_PI_4 && all_stairs_information_.at(i).yaw< 2*M_PI-M_PI_4){
                            angleNumber=3;
                            dist= distancePointLine(checkPoint, p0, p2);
                            lengthStairs= std::sqrt(std::pow(p0(0)-p1(0),2)+std::pow(p0(1)-p1(1),2));
                            if(distancePointLine(checkPoint, p0, p1)<=inflationStairs || distancePointLine(checkPoint, p2, p3)<=inflationStairs){
                                pointOnBorder=true; //=> cost going to be heigh
                            }
                        }

                        if(pointOnBorder==true){
                            computedCost=100;
                        }else{
                            computedCost= (maxHeigthInGrid*dist)/lengthStairs + floorOffset;
                            //                                                ROS_INFO("index Y: %i,  heigth: %i", iy, computedCost);
                        }
                    }else{
                        computedCost=oldCost;
                    }


                    unsigned char newCost = computedCost;

                    if(oldCost == newCost) continue;

                    allCount++;
                    env_->UpdateCost(ix, iy, computedCost);

                    nav2dcell_t nav2dcell;
                    nav2dcell.x = ix;
                    nav2dcell.y = iy;
                    changedcellsV.push_back(nav2dcell);
                }
            }

            //offset in fornt and in back of the stairs
            allCount=allCount + updateCostInExtendedStairsArea(p0,p1,p2,p3, angleNumber, floorOffset, maxHeigthInGrid, changedcellsV);
        }else{
            for(ssize_t ix(startIterationX_d); ix < endIterartionX_d; ++ix) {
                for(ssize_t iy(startIterationY_d); iy < endIterartionY_d; ++iy) {
                    env_->UpdateCost(ix, iy, 100);
                }
            }
        }

    }

    try{
        if(!changedcellsV.empty()){
            StateChangeQuery* scq = new LatticeSCQ(env_, changedcellsV);
            planner_->costs_changed(*scq);
            delete scq;
        }

        if(allCount > force_scratch_limit_)
            planner_->force_planning_from_scratch();
    }
    catch(SBPL_Exception e){
        ROS_ERROR("[SBPL_STAIRS_PLANNER] SBPL failed to update the heighmap with stairs infromation");
        return false;
    }

    nav_msgs::OccupancyGrid::Ptr occupancyGrid(new nav_msgs::OccupancyGrid());
    occupancyGrid->header.frame_id="/world";
    occupancyGrid->info.resolution=costmap_ros_->getCostmap()->getResolution();
    occupancyGrid->info.height=costmap_ros_->getCostmap()->getSizeInCellsY();
    occupancyGrid->info.width=costmap_ros_->getCostmap()->getSizeInCellsX();
    occupancyGrid->info.origin.position.x=costmap_ros_->getCostmap()->getOriginX();
    occupancyGrid->info.origin.position.y=costmap_ros_->getCostmap()->getOriginY();
    occupancyGrid->info.origin.position.z=0;
    occupancyGrid->data.resize(costmap_ros_->getCostmap()->getSizeInCellsX() * costmap_ros_->getCostmap()->getSizeInCellsY());

    for(int x=0; x<occupancyGrid->info.width; x++){
        for(int y=0; y<occupancyGrid->info.height; y++){
            occupancyGrid->data.at(x + y * occupancyGrid->info.width)=env_->GetMapCost(x,y);
        }
    }
    if(occupancyGrid_pub_>0){
        occupancyGrid_pub_.publish(occupancyGrid);
    }

    bool startOnStairs=false;
    try{
        int ret = env_->SetStart(startX - costmap_ros_->getCostmap()->getOriginX(), startY  - costmap_ros_->getCostmap()->getOriginY(), theta_start, false, startOnStairs);
        if(ret < 0 || planner_->set_start(ret) == 0){
            ROS_ERROR("ERROR: failed to set start state\n");
            return false;
        }
    }
    catch(SBPL_Exception e){
        ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
        return false;
    }

    try{
        int ret = env_->SetGoal(goalX - costmap_ros_->getCostmap()->getOriginX(), goalY  - costmap_ros_->getCostmap()->getOriginY(), theta_goal, false);
        if(ret < 0 || planner_->set_goal(ret) == 0){
            ROS_ERROR("ERROR: failed to set goal state\n");
            return false;
        }
    }
    catch(SBPL_Exception e){
        ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
        return false;
    }

    //setting planner parameters
    ROS_DEBUG("allocated:%f, init eps:%f\n",allocated_time_,initial_epsilon_);
    planner_->set_initialsolution_eps(initial_epsilon_);
    planner_->set_search_mode(false);

    ROS_DEBUG("[sbpl_lattice_planner] run planner");
    vector<int> solution_stateIDs;
    int solution_cost;
    try{
        ROS_INFO("replan");
                int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
        if(ret){
            ROS_INFO("Solution is found");
            ROS_DEBUG("Solution is found\n");}
        else{
            ROS_INFO("Solution not found\n");
            for(int i=0; i< vis_pc->size(); i++){
                vis_pc->at(i).x=vis_pc->at(i).x+ costmap_ros_->getCostmap()->getOriginX();
                vis_pc->at(i).y=vis_pc->at(i).y+ costmap_ros_->getCostmap()->getOriginY();
            }
            vis_pc_pub_.publish(vis_pc);
            return false;
        }
    }
    catch(SBPL_Exception e){
        ROS_ERROR("SBPL encountered a fatal exception while planning");
        return false;
    }

    ROS_INFO("size of solution=%d", (int)solution_stateIDs.size());

    vector<sbpl_xy_theta_flip_pt_t> sbpl_path;
    vector<EnvNAVXYTHETAFLIPAction_t> action_list;
    try{
        env_->ConvertStateIDPathintoXYThetaFlipperPath(&solution_stateIDs, &sbpl_path);
        env_->GetActionsFromStateIDPath(&solution_stateIDs, &action_list);
    }
    catch(SBPL_Exception e){
        ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
        return false;
    }
    ros::Time plan_time = ros::Time::now();

    ROS_INFO("Plan has %d points.\n", (int)sbpl_path.size());
    visualization_msgs::MarkerArray all_flipper_actuation_pos;
    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(sbpl_path.size());
    gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    gui_path.header.stamp = plan_time;
    int flipperFlagCounter=0;

    refined_path_.path.clear();
    hector_stairs_planner_msgs::Path_segment path_w_flipper;

    int startFlipperOffset=0;
    if(startOnStairs){
        ROS_INFO("plan starts on stairs");
        startFlipperOffset=2;
    }

    bool robotMovesForward=false;
    bool sameDirection=false;
    bool robotMovesUpstairs=false;

    for(unsigned int i=0; i<sbpl_path.size(); i++){
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = costmap_ros_->getGlobalFrameID();

        pose.pose.position.x = sbpl_path[i].x + costmap_ros_->getCostmap()->getOriginX();
        pose.pose.position.y = sbpl_path[i].y + costmap_ros_->getCostmap()->getOriginY();
        pose.pose.position.z = 0;

        tf::Quaternion temp;
        temp.setRPY(0,0,sbpl_path[i].theta);

        pose.pose.orientation.x = temp.getX();
        pose.pose.orientation.y = temp.getY();
        pose.pose.orientation.z = temp.getZ();
        pose.pose.orientation.w = temp.getW();

        plan.push_back(pose);
        path_w_flipper.segment.header.frame_id=costmap_ros_->getGlobalFrameID();
        path_w_flipper.segment.poses.push_back(pose);
        //default
        path_w_flipper.flipperFront=-M_PI_2;
        path_w_flipper.flipperRear=-M_PI_2;

        if(sbpl_path[i].flipperFlag==true){
            //insert current flipper pos
            visualization_msgs::Marker marker;
            marker.header.frame_id = pose.header.frame_id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.ns = "hector_sbpl_stairs_planner";
            marker.id = i;

            marker.pose.position.x = pose.pose.position.x;
            marker.pose.position.y = pose.pose.position.y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            int flipperDecision= (flipperFlagCounter+startFlipperOffset)%4;
            float stairs_pitch;
            float stairs_yaw;

            getPitchAndYawFromNearestStaris(marker.pose.position.x, marker.pose.position.y, stairs_pitch, stairs_yaw);

            if(sbpl_path[0].robotHeight < sbpl_path.back().robotHeight){
                robotMovesUpstairs=true;
            }else{
                robotMovesUpstairs=false;
            }

            if(fabs(stairs_yaw - sbpl_path[i].theta) > M_PI_2){
                sameDirection=false;
            }else{
                sameDirection=true;
            }

            if(sameDirection && robotMovesUpstairs){
                robotMovesForward=false;
                ROS_INFO("robot goes UPstairs backward");
            }

            if(!sameDirection && robotMovesUpstairs){
                robotMovesForward=true;
                ROS_INFO("robot goes UPstairs forward");
            }

            if(sameDirection && !robotMovesUpstairs){
                robotMovesForward=true;
                ROS_INFO("robot goes DOWNstairs forward");
            }

            if(!sameDirection && !robotMovesUpstairs){
                robotMovesForward=false;
                ROS_INFO("robot goes DOWNstairs backward");
            }

            switch(flipperDecision){
            case 0:
                if(robotMovesUpstairs){
                    path_w_flipper.flipperFront=-stairs_pitch;
                    path_w_flipper.flipperRear=0;
                }else{
                    path_w_flipper.flipperFront=0;
                    path_w_flipper.flipperRear=0;
                }

                break;
            case 1:
                if(robotMovesUpstairs){
                    path_w_flipper.flipperFront=0;
                    path_w_flipper.flipperRear=0;
                }else{
                    path_w_flipper.flipperFront=-M_PI_2;
                    path_w_flipper.flipperRear=-M_PI_2;
                }

                break;
            case 2:
                if(robotMovesUpstairs || startFlipperOffset==2){
                    path_w_flipper.flipperFront=0;
                    path_w_flipper.flipperRear=0;
                }else{
                    path_w_flipper.flipperFront=-M_PI_2;
                    path_w_flipper.flipperRear=-M_PI_2;
                }

                break;
            case 3:
                path_w_flipper.flipperFront=-M_PI_2;
                path_w_flipper.flipperRear=-M_PI_2;

                break;
            }

            if(robotMovesForward==false){
                double tempPos;
                tempPos=path_w_flipper.flipperFront;
                path_w_flipper.flipperFront=path_w_flipper.flipperRear;
                path_w_flipper.flipperRear=tempPos;
            }

            refined_path_.path.push_back(path_w_flipper);
            path_w_flipper.segment.poses.clear();

            all_flipper_actuation_pos.markers.push_back(marker);
            flipperFlagCounter++;

        }

        gui_path.poses[i].pose.position.x = plan[i].pose.position.x;
        gui_path.poses[i].pose.position.y = plan[i].pose.position.y;
        gui_path.poses[i].pose.position.z = 0;
    }


    for(int i=0; i< vis_pc->size(); i++){
        vis_pc->at(i).x=vis_pc->at(i).x+ costmap_ros_->getCostmap()->getOriginX();
        vis_pc->at(i).y=vis_pc->at(i).y+ costmap_ros_->getCostmap()->getOriginY();
    }
    vis_pc_pub_.publish(vis_pc);

    if(flipperActuationPoints_pub_.getNumSubscribers()>0){
        flipperActuationPoints_pub_.publish(all_flipper_actuation_pos);
    }

    if(refined_path_.path.size() == 2 && !startOnStairs && robotMovesUpstairs){
        path_w_flipper.flipperFront=0;
        path_w_flipper.flipperRear=0;
    }

    refined_path_.path.push_back(path_w_flipper);

    std::cout<<"size of path vector: "<<refined_path_.path.size()<<std::endl;
    //publish if flipper action is required
    if(refined_path_.path.size()>=1){
        refined_plan_pub_.publish(refined_path_);
    }

    plan_pub_.publish(gui_path);
    return true;

}

int HectorSbplStairsPlanner::updateCostInExtendedStairsArea(Eigen::Vector2f base, Eigen::Vector2f minXmaxY, Eigen::Vector2f maxXminY, Eigen::Vector2f maxXmaxY, int angleNumber, int minCost, int maxCost, vector<nav2dcell_t> &changedcellsV){
    int numberUpdates=0;
    unsigned char newCost;
    int startIterationX_d, endIterartionX_d ,startIterationY_d, endIterartionY_d;
    //extention at top of the stairs
    switch(angleNumber){
    case 0:
        startIterationX_d= CONTXY2DISC(base(0) - fabs(base(0)-maxXminY(0))*1 - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        startIterationY_d= CONTXY2DISC(base(1) - fabs(base(1)-maxXminY(1))*1  - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        endIterartionX_d= CONTXY2DISC(minXmaxY(0) - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        endIterartionY_d= CONTXY2DISC(minXmaxY(1) - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        break;
    case 1:
        startIterationX_d= CONTXY2DISC(base(0) - fabs(base(0)-minXmaxY(0))*1 - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        startIterationY_d= CONTXY2DISC(base(1) - fabs(base(1)-minXmaxY(1))*1  - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        endIterartionX_d= CONTXY2DISC(maxXminY(0) - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        endIterartionY_d= CONTXY2DISC(maxXminY(1) - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        break;
    case 2:
        startIterationX_d= CONTXY2DISC(maxXminY(0) - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        startIterationY_d= CONTXY2DISC(maxXminY(1) - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        endIterartionX_d= CONTXY2DISC(maxXmaxY(0) + fabs(maxXmaxY(0)-minXmaxY(0))*1 - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        endIterartionY_d= CONTXY2DISC(maxXmaxY(1) + fabs(maxXmaxY(1)-minXmaxY(1))*1 - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        break;
    case 3:
        startIterationX_d= CONTXY2DISC(minXmaxY(0) - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        startIterationY_d= CONTXY2DISC(minXmaxY(1) - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        endIterartionX_d= CONTXY2DISC(maxXmaxY(0) + fabs(maxXmaxY(0)-maxXminY(0))*1 - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        endIterartionY_d= CONTXY2DISC(maxXmaxY(1) + fabs(maxXmaxY(1)-maxXminY(1))*1 - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        break;
    }

    newCost=maxCost + minCost;
    //    ROS_INFO("extended Staris cost: %i", newCost);
    float inflationStairs=0.4;
    inflationStairs=inflationRadius_;
    inflationStairs=0.0;
    bool pointOnBorder=false;
    for(ssize_t ix(startIterationX_d); ix < endIterartionX_d; ++ix) {
        for(ssize_t iy(startIterationY_d); iy < endIterartionY_d; ++iy) {
            Eigen::Vector2f checkPoint;
            pointOnBorder=false;
            checkPoint(0)=DISCXY2CONT(ix, costmap_ros_->getCostmap()->getResolution()) + costmap_ros_->getCostmap()->getOriginX();
            checkPoint(1)=DISCXY2CONT(iy, costmap_ros_->getCostmap()->getResolution()) + costmap_ros_->getCostmap()->getOriginY();
            switch(angleNumber){
            case 0:
                if(distancePointLine(checkPoint, maxXmaxY, minXmaxY)<=inflationStairs || distancePointLine(checkPoint, base, maxXminY)<=inflationStairs){
                    pointOnBorder=true; //=> cost going to be heigh
                }
                break;
            case 1:
                if(distancePointLine(checkPoint, base, minXmaxY)<=inflationStairs || distancePointLine(checkPoint, maxXminY, maxXmaxY)<=inflationStairs){
                    pointOnBorder=true; //=> cost going to be heigh
                }
                break;
            case 2:
                if(distancePointLine(checkPoint, maxXmaxY, minXmaxY)<=inflationStairs || distancePointLine(checkPoint, base, maxXminY)<=inflationStairs){
                    pointOnBorder=true; //=> cost going to be heigh
                }
                break;
            case 3:
                if(distancePointLine(checkPoint, base, minXmaxY)<=inflationStairs || distancePointLine(checkPoint, maxXminY, maxXmaxY)<=inflationStairs){
                    pointOnBorder=true; //=> cost going to be heigh
                }
                break;
            }

            if(pointOnBorder){
                env_->UpdateCost(ix, iy, 100);
                nav2dcell_t nav2dcell;
                nav2dcell.x = ix;
                nav2dcell.y = iy;
                changedcellsV.push_back(nav2dcell);
                numberUpdates=numberUpdates+1;
            }else{
                unsigned char oldCost = env_->GetMapCost(ix,iy);
                if(oldCost == newCost){
                    continue;
                }

                env_->UpdateCost(ix, iy, newCost);
                nav2dcell_t nav2dcell;
                nav2dcell.x = ix;
                nav2dcell.y = iy;
                changedcellsV.push_back(nav2dcell);
                numberUpdates=numberUpdates+1;
            }
        }
    }



    //extention at bottom of the stairs
    switch(angleNumber){
    case 2:
        startIterationX_d= CONTXY2DISC(base(0) - fabs(base(0)-maxXminY(0))*1 - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        startIterationY_d= CONTXY2DISC(base(1) - fabs(base(1)-maxXminY(1))*1  - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        endIterartionX_d= CONTXY2DISC(minXmaxY(0) - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        endIterartionY_d= CONTXY2DISC(minXmaxY(1) - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        break;
    case 3:
        startIterationX_d= CONTXY2DISC(base(0) - fabs(base(0)-minXmaxY(0))*1 - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        startIterationY_d= CONTXY2DISC(base(1) - fabs(base(1)-minXmaxY(1))*1  - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        endIterartionX_d= CONTXY2DISC(maxXminY(0) - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        endIterartionY_d= CONTXY2DISC(maxXminY(1) - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        break;
    case 0:
        startIterationX_d= CONTXY2DISC(maxXminY(0) - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        startIterationY_d= CONTXY2DISC(maxXminY(1) - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        endIterartionX_d= CONTXY2DISC(maxXmaxY(0) + fabs(maxXmaxY(0)-minXmaxY(0))*1 - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        endIterartionY_d= CONTXY2DISC(maxXmaxY(1) + fabs(maxXmaxY(1)-minXmaxY(1))*1 - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        break;
    case 1:
        startIterationX_d= CONTXY2DISC(minXmaxY(0) - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        startIterationY_d= CONTXY2DISC(minXmaxY(1) - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        endIterartionX_d= CONTXY2DISC(maxXmaxY(0) + fabs(maxXmaxY(0)-maxXminY(0))*1 - costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getResolution());
        endIterartionY_d= CONTXY2DISC(maxXmaxY(1) + fabs(maxXmaxY(1)-maxXminY(1))*1 - costmap_ros_->getCostmap()->getOriginY(), costmap_ros_->getCostmap()->getResolution());
        break;
    }

    newCost= minCost;
    inflationStairs=inflationRadius_;
    inflationStairs=0.0;
    newCost=0;
    for(ssize_t ix(startIterationX_d); ix < endIterartionX_d; ++ix) {
        for(ssize_t iy(startIterationY_d); iy < endIterartionY_d; ++iy) {
            Eigen::Vector2f checkPoint;
            pointOnBorder=false;
            checkPoint(0)=DISCXY2CONT(ix, costmap_ros_->getCostmap()->getResolution()) + costmap_ros_->getCostmap()->getOriginX();
            checkPoint(1)=DISCXY2CONT(iy, costmap_ros_->getCostmap()->getResolution()) + costmap_ros_->getCostmap()->getOriginY();
            switch(angleNumber){
            case 0:
                if(distancePointLine(checkPoint, maxXmaxY, minXmaxY)<=inflationStairs || distancePointLine(checkPoint, base, maxXminY)<=inflationStairs){
                    pointOnBorder=true; //=> cost going to be heigh
                }
                break;
            case 1:
                if(distancePointLine(checkPoint, base, minXmaxY)<=inflationStairs || distancePointLine(checkPoint, maxXminY, maxXmaxY)<=inflationStairs){
                    pointOnBorder=true; //=> cost going to be heigh
                }
                break;
            case 2:
                if(distancePointLine(checkPoint, maxXmaxY, minXmaxY)<=inflationStairs || distancePointLine(checkPoint, base, maxXminY)<=inflationStairs){
                    pointOnBorder=true; //=> cost going to be heigh
                }
                break;
            case 3:
                if(distancePointLine(checkPoint, base, minXmaxY)<=inflationStairs || distancePointLine(checkPoint, maxXminY, maxXmaxY)<=inflationStairs){
                    pointOnBorder=true; //=> cost going to be heigh
                }
                break;
            }

            if(pointOnBorder){
                env_->UpdateCost(ix, iy, 100);
                nav2dcell_t nav2dcell;
                nav2dcell.x = ix;
                nav2dcell.y = iy;
                changedcellsV.push_back(nav2dcell);
                numberUpdates=numberUpdates+1;
            }else{
                unsigned char oldCost = env_->GetMapCost(ix,iy);
                if(oldCost == newCost){
                    continue;
                }
                env_->UpdateCost(ix, iy, newCost);
                nav2dcell_t nav2dcell;
                nav2dcell.x = ix;
                nav2dcell.y = iy;
                changedcellsV.push_back(nav2dcell);
                numberUpdates=numberUpdates+1;
            }
        }
    }

    return numberUpdates;
}

void HectorSbplStairsPlanner::getPitchAndYawFromNearestStaris(float flagPosX, float flagPosY, float &pitch, float &yaw){
    float min_dist=FLT_MAX;
    for(int i=0; i<all_stairs_information_.size(); i++){
        float temp_dist= std::sqrt(std::pow(flagPosX - all_stairs_information_.at(i).pose.pose.position.x ,2) + std::pow(flagPosY - all_stairs_information_.at(i).pose.pose.position.y ,2));
        if(temp_dist < min_dist){
            min_dist=temp_dist;
            pitch=all_stairs_information_.at(i).pitch;
            yaw=all_stairs_information_.at(i).yaw;
        }
    }
}

float HectorSbplStairsPlanner::distancePointLine(Eigen::Vector2f checkPointIn, Eigen::Vector2f baseIn, Eigen::Vector2f pointIn){
    Eigen::Vector3f checkPoint;
    checkPoint(0)=checkPointIn(0);
    checkPoint(1)=checkPointIn(1);
    checkPoint(2)=0;
    Eigen::Vector3f base;
    base(0)=baseIn(0);
    base(1)=baseIn(1);
    base(2)=0;
    Eigen::Vector3f point;
    point(0)=pointIn(0);
    point(1)=pointIn(1);
    point(2)=0;
    Eigen::Vector3f direction=point-base;

    float tempA= std::sqrt(direction.cross(checkPoint-base).dot(direction.cross(checkPoint-base)));
    float lengthDirection= std::sqrt(direction.dot(direction));
    return tempA/lengthDirection;
}


bool HectorSbplStairsPlanner::pointInStairsFootprint(Eigen::Vector2f checkPoint, Eigen::Vector2f minXminY, Eigen::Vector2f maxXmaxY){
    if(checkPoint(0)>=minXminY(0) && checkPoint(1)>=minXminY(1) && checkPoint(0)<=maxXmaxY(0) && checkPoint(1)<=maxXmaxY(1)){
        return true;
    }else{
        return false;
    }
}

float HectorSbplStairsPlanner::sign(Eigen::Vector2f pt, Eigen::Vector2f v1, Eigen::Vector2f v2){
    return (pt(0) - v2(0)) * (v1(1) - v2(1)) - (v1(0) - v2(0)) * (pt(1) - v2(1));
}

bool HectorSbplStairsPlanner::PointInTriangle (Eigen::Vector2f pt, Eigen::Vector2f v1, Eigen::Vector2f v2, Eigen::Vector2f v3){
    bool b1, b2, b3;

    b1 = sign(pt, v1, v2) < 0.0f;
    b2 = sign(pt, v2, v3) < 0.0f;
    b3 = sign(pt, v3, v1) < 0.0f;

    return ((b1 == b2) && (b2 == b3));
}

//stairs_detection callback; update stairs areas and orientations
void HectorSbplStairsPlanner::updateInformationAboutStairs_Callback(hector_stair_detection_msgs::BorderAndOrientationOfStairs information){
    if(information.clearStairsInformation){
        resetAllStairsInformation();
    }
    visualization_msgs::MarkerArray border_of_stairs= information.border_of_stairs;
    geometry_msgs::PoseStamped pose_of_stairs= information.orientation_of_stairs;
    int number_points_on_line= information.number_of_points;
    bool insertStairs= true;
    int modify_stairs_idx=-1;
    //check if staris already exits, if not insert, else modify the existing stairs or do nothing
    if(all_stairs_information_.size() == 0){
        ROS_INFO("instert new Stairs");
        insertStairsInformation(border_of_stairs, pose_of_stairs, number_points_on_line);
    }else{
        for(int i=0; i<all_stairs_information_.size(); i++){
            //check euclidean distance, between the staris poses
            float euclidenDist=std::sqrt(std::pow(pose_of_stairs.pose.position.x - all_stairs_information_.at(i).pose.pose.position.x, 2) + std::pow(pose_of_stairs.pose.position.y - all_stairs_information_.at(i).pose.pose.position.y, 2));
            ROS_INFO("euclidean dist betweem possible staris: %f", euclidenDist);
            if(euclidenDist < between_staris_distance_threshold_){
                insertStairs=false;
                if(euclidenDist < same_stairs_distance_threshold_){
                    modify_stairs_idx=i;
                }
            }
        }

        if(insertStairs){
            ROS_INFO("insert new Stairs");
            insertStairsInformation(border_of_stairs, pose_of_stairs, number_points_on_line);
        }else{
            if(modify_stairs_idx >=0){
                ROS_INFO("modify old Stairs");
                modifyStairsInformation(border_of_stairs, pose_of_stairs, modify_stairs_idx, number_points_on_line);
            }
        }
    }

    borders_of_all_stairs_.markers.clear();
    for(int i=0; i<all_stairs_information_.size(); i++){
        Eigen::Vector2f directionStairs;
        directionStairs(0)=information.directionX;
        directionStairs(1)=information.directionY;
        Eigen::Vector2f minXminY;
        minXminY(0)=all_stairs_information_.at(i).minX;
        minXminY(1)=all_stairs_information_.at(i).minY;
        Eigen::Vector2f maxXminY;
        maxXminY(0)=all_stairs_information_.at(i).maxX;
        maxXminY(1)=all_stairs_information_.at(i).minY;
        Eigen::Vector2f minXmaxY;
        minXmaxY(0)=all_stairs_information_.at(i).minX;
        minXmaxY(1)=all_stairs_information_.at(i).maxY;
        int componetOfDirection;

        if(insertStairs && i==all_stairs_information_.size()-1){
            componetOfDirection= getZComponent(directionStairs, minXminY, maxXminY, minXmaxY);
            all_stairs_information_.at(i).direction=componetOfDirection;
        }else{
            componetOfDirection=all_stairs_information_.at(i).direction;
        }

        for(int j=0; j<4; j++){
            //insert all borders
            visualization_msgs::Marker marker;
            marker.header.frame_id = pose_of_stairs.header.frame_id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.ns = "hector_sbpl_stairs_environment";
            marker.id = i*4+j;

            if(j==0){
                marker.pose.position.x = all_stairs_information_.at(i).minX;
                marker.pose.position.y = all_stairs_information_.at(i).minY;
                switch(componetOfDirection) {
                case 1: marker.pose.position.z=marker.pose.position.z=all_stairs_information_.at(i).maxZ;
                    break;
                case 2: marker.pose.position.z=marker.pose.position.z=all_stairs_information_.at(i).maxZ;
                    break;
                case 3: marker.pose.position.z=all_stairs_information_.at(i).minZ;
                    break;
                case 4: marker.pose.position.z=all_stairs_information_.at(i).minZ;
                    break;
                default:;
                }

            }
            if(j==1){
                marker.pose.position.x = all_stairs_information_.at(i).minX;
                marker.pose.position.y = all_stairs_information_.at(i).maxY;
                switch(componetOfDirection) {
                case 1: marker.pose.position.z=all_stairs_information_.at(i).maxZ;
                    break;
                case 2: marker.pose.position.z=all_stairs_information_.at(i).minZ;
                    break;
                case 3: marker.pose.position.z=all_stairs_information_.at(i).minZ;
                    break;
                case 4: marker.pose.position.z=all_stairs_information_.at(i).maxZ;
                    break;
                default:;
                }
            }
            if(j==2){
                marker.pose.position.x = all_stairs_information_.at(i).maxX;
                marker.pose.position.y = all_stairs_information_.at(i).minY;
                switch(componetOfDirection) {
                case 1: marker.pose.position.z=all_stairs_information_.at(i).minZ;
                    break;
                case 2: marker.pose.position.z=all_stairs_information_.at(i).maxZ;
                    break;
                case 3: marker.pose.position.z=all_stairs_information_.at(i).maxZ;
                    break;
                case 4: marker.pose.position.z=all_stairs_information_.at(i).minZ;
                    break;
                default:;
                }
            }
            if(j==3){
                marker.pose.position.x = all_stairs_information_.at(i).maxX;
                marker.pose.position.y = all_stairs_information_.at(i).maxY;

                switch(componetOfDirection) {
                case 1: marker.pose.position.z=all_stairs_information_.at(i).minZ;
                    break;
                case 2: marker.pose.position.z=all_stairs_information_.at(i).minZ;
                    break;
                case 3: marker.pose.position.z=all_stairs_information_.at(i).maxZ;
                    break;
                case 4: marker.pose.position.z=all_stairs_information_.at(i).maxZ;
                    break;
                default:;
                }
            }

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            borders_of_all_stairs_.markers.push_back(marker);
        }

    }

    //publish all_stairs_information
    if(all_stairs_information_border_pub_.getNumSubscribers()>0){
        all_stairs_information_border_pub_.publish(borders_of_all_stairs_);
    }

    if(all_stairs_information_orientaion_pub_.getNumSubscribers()>0){
        geometry_msgs::PoseArray orientaions;
        geometry_msgs::Pose tempPose;
        orientaions.header.frame_id= pose_of_stairs.header.frame_id;
        for(int i=0; i< all_stairs_information_.size(); i++){
            tempPose.orientation= all_stairs_information_.at(i).pose.pose.orientation;
            tempPose.position= all_stairs_information_.at(i).pose.pose.position;
            orientaions.poses.push_back(tempPose);
        }
        all_stairs_information_orientaion_pub_.publish(orientaions);
    }

}

int HectorSbplStairsPlanner::getZComponent(Eigen::Vector2f directionStairs, Eigen::Vector2f minXminY, Eigen::Vector2f maxXminY, Eigen::Vector2f minXmaxY){
    Eigen::Vector2f direction1;
    Eigen::Vector2f direction2;
    Eigen::Vector2f direction3;
    Eigen::Vector2f direction4;

    direction1=maxXminY-minXminY;
    direction2=minXmaxY-minXminY;
    direction3=minXminY-maxXminY;
    direction4=minXminY-minXmaxY;

    float angle1=acos((directionStairs.dot(direction1))/(directionStairs.norm()*direction1.norm()));
    float angle2=acos((directionStairs.dot(direction2))/(directionStairs.norm()*direction2.norm()));
    float angle3=acos((directionStairs.dot(direction3))/(directionStairs.norm()*direction3.norm()));
    float angle4=acos((directionStairs.dot(direction4))/(directionStairs.norm()*direction4.norm()));

    if(angle1>=angle2 && angle1>=angle3 && angle1>=angle4){
        return 1;
    }

    if(angle2>=angle1 && angle2>=angle3 && angle2>=angle4){
        return 2;
    }

    if(angle3>=angle2 && angle3>=angle1 && angle3>=angle4){
        return 3;
    }

    if(angle4>=angle2 && angle4>=angle3 && angle4>=angle1){
        return 4;
    }
}

void HectorSbplStairsPlanner::insertStairsInformation(visualization_msgs::MarkerArray &border_of_stairs, geometry_msgs::PoseStamped &pose_of_stairs, int points_on_line){
    stairs_information stairs;
    stairs.points_on_line=points_on_line;
    stairs.minX=FLT_MAX;
    stairs.maxX=-FLT_MAX;
    stairs.minY=FLT_MAX;
    stairs.maxY=-FLT_MAX;
    stairs.minZ=FLT_MAX;
    stairs.maxZ=-FLT_MAX;

    for(int i=0; i<border_of_stairs.markers.size(); i++){
        if(border_of_stairs.markers.at(i).pose.position.x < stairs.minX){
            stairs.minX=border_of_stairs.markers.at(i).pose.position.x;
        }

        if(border_of_stairs.markers.at(i).pose.position.x > stairs.maxX){
            stairs.maxX=border_of_stairs.markers.at(i).pose.position.x;
        }

        if(border_of_stairs.markers.at(i).pose.position.y < stairs.minY){
            stairs.minY=border_of_stairs.markers.at(i).pose.position.y;
        }

        if(border_of_stairs.markers.at(i).pose.position.y > stairs.maxY){
            stairs.maxY=border_of_stairs.markers.at(i).pose.position.y;
        }

        if(border_of_stairs.markers.at(i).pose.position.z < stairs.minZ){
            stairs.minZ=border_of_stairs.markers.at(i).pose.position.z;
        }

        if(border_of_stairs.markers.at(i).pose.position.z > stairs.maxZ){
            stairs.maxZ=border_of_stairs.markers.at(i).pose.position.z;
        }
    }

    stairs.pose= pose_of_stairs;

    tf::Quaternion q_tf;
    tf::quaternionMsgToTF(pose_of_stairs.pose.orientation, q_tf);

    double r, pitch, yaw;
    tf::Matrix3x3(q_tf).getEulerZYX(yaw,pitch,r);
    if(yaw<0){
        yaw=yaw+2*M_PI;
    }
    stairs.yaw=yaw;
    stairs.pitch=pitch;

    all_stairs_information_.push_back(stairs);
}

void HectorSbplStairsPlanner::modifyStairsInformation(visualization_msgs::MarkerArray &border_of_stairs, geometry_msgs::PoseStamped &pose_of_stairs, int modify_stairs_idx, int number_points_on_line){
    stairs_information old_stairs= all_stairs_information_.at(modify_stairs_idx);

    for(int i=0; i<border_of_stairs.markers.size(); i++){
        if(border_of_stairs.markers.at(i).pose.position.x < old_stairs.minX){
            all_stairs_information_.at(modify_stairs_idx).minX=border_of_stairs.markers.at(i).pose.position.x;
        }

        if(border_of_stairs.markers.at(i).pose.position.x > old_stairs.maxX){
            all_stairs_information_.at(modify_stairs_idx).maxX=border_of_stairs.markers.at(i).pose.position.x;
        }

        if(border_of_stairs.markers.at(i).pose.position.y < old_stairs.minY){
            all_stairs_information_.at(modify_stairs_idx).minY=border_of_stairs.markers.at(i).pose.position.y;
        }

        if(border_of_stairs.markers.at(i).pose.position.y > old_stairs.maxY){
            all_stairs_information_.at(modify_stairs_idx).maxY=border_of_stairs.markers.at(i).pose.position.y;
        }

        if(border_of_stairs.markers.at(i).pose.position.z < old_stairs.minZ){
            all_stairs_information_.at(modify_stairs_idx).minZ=border_of_stairs.markers.at(i).pose.position.z;
        }

        if(border_of_stairs.markers.at(i).pose.position.z > old_stairs.maxZ){
            all_stairs_information_.at(modify_stairs_idx).maxZ=border_of_stairs.markers.at(i).pose.position.z;
        }
    }

    //more points on lina <=> more cluster at stairs <=> might be more accurate
    float x_of_stairs= all_stairs_information_.at(modify_stairs_idx).minX + fabs(all_stairs_information_.at(modify_stairs_idx).maxX- all_stairs_information_.at(modify_stairs_idx).minX)/2;
    float y_of_stairs=all_stairs_information_.at(modify_stairs_idx).minY + fabs(all_stairs_information_.at(modify_stairs_idx).maxY- all_stairs_information_.at(modify_stairs_idx).minY)/2;

    float euclidenDist_old=std::sqrt(std::pow(x_of_stairs - all_stairs_information_.at(modify_stairs_idx).pose.pose.position.x, 2) + std::pow(y_of_stairs - all_stairs_information_.at(modify_stairs_idx).pose.pose.position.y, 2));
    float euclidenDist_new=std::sqrt(std::pow(x_of_stairs - pose_of_stairs.pose.position.x, 2) + std::pow(y_of_stairs - pose_of_stairs.pose.position.y, 2));

    if(number_points_on_line >= all_stairs_information_.at(modify_stairs_idx).points_on_line && euclidenDist_new < euclidenDist_old){
        ROS_DEBUG("MODIFY POSE");
        all_stairs_information_.at(modify_stairs_idx).points_on_line=number_points_on_line;
        all_stairs_information_.at(modify_stairs_idx).pose= pose_of_stairs;

        tf::Quaternion q_tf;
        tf::quaternionMsgToTF(pose_of_stairs.pose.orientation, q_tf);

        double r, pitch, yaw;
        tf::Matrix3x3(q_tf).getEulerZYX(yaw,pitch,r);
        if(yaw<0){
            yaw=yaw+2*M_PI;
        }


        all_stairs_information_.at(modify_stairs_idx).yaw=yaw;
        all_stairs_information_.at(modify_stairs_idx).pitch=pitch;
    }

}

int HectorSbplStairsPlanner::getNumberOfStairs(){
    return all_stairs_information_.size();
}

void HectorSbplStairsPlanner::resetStairsCB(const std_msgs::Bool reset_msg)
{
    if(reset_msg.data){
       resetAllStairsInformation();
    }
}

void HectorSbplStairsPlanner::resetAllStairsInformation(){
    ROS_INFO("clear staris information");
    all_stairs_information_.clear();
    borders_of_all_stairs_.markers.clear();
}

