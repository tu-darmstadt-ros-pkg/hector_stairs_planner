#include <ros/ros.h>
#include <hector_sbpl_flipper_control_env/hector_sbpl_flipper_control_env.h>
#include <hector_sbpl_stairs_planner/hector_sbpl_stairs_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <gridmap/gridmap.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace hector_sbpl_stairs_planner;

class PlannerFlipperControlTest{
public:
    PlannerFlipperControlTest(){
        ros::NodeHandle nh;

        drivepath_pub_= nh.advertise<nav_msgs::Path>("/drivepath",2, this);
        occupancyGrid_pub_= nh.advertise<nav_msgs::OccupancyGrid>("/stairs_planner/occupancyGridUpdated", 1, this);
        goal_pose_sub_ = nh.subscribe("/goal", 10, &PlannerFlipperControlTest::goalPoseCallback, this);

        path_.header.frame_id = "/world";

        map_= new gridmap::GridMap();
        map_->readGridMap("stairs_planner_virtual_ramp_test");
        //        map_->readGridMap("stairs_planner_test");
//        stairs_planner_virtual_step_test
//        stairs_planner_virtual_ramp_test
//      emptySquare
//        emptyRectangle

        nav_msgs::OccupancyGrid::Ptr occupancyGrid(new nav_msgs::OccupancyGrid());
        occupancyGrid->header.frame_id="/world";
        occupancyGrid->info.resolution=map_->getInfo().resolution;
        occupancyGrid->info.height=map_->getInfo().width;
        occupancyGrid->info.width= map_->getInfo().height;
        occupancyGrid->info.origin.position.x=map_->getInfo().origin.position.x;
        occupancyGrid->info.origin.position.y=map_->getInfo().origin.position.y;
        occupancyGrid->info.origin.position.z=map_->getInfo().origin.position.z;
        occupancyGrid->data.resize(occupancyGrid->info.height * occupancyGrid->info.width);

        int factor=1;
        for(int x=0; x<occupancyGrid->info.height; x++){
            for(int y=0; y<occupancyGrid->info.width; y++){
                    if(!map_->getCell(x + y * occupancyGrid->info.height).empty){
                        if(occupancyGrid->data.at(x + y * occupancyGrid->info.height)=map_->getHeight(x + y * occupancyGrid->info.height)*factor > 100){
                            occupancyGrid->data.at(x + y * occupancyGrid->info.height)=100;
                        }else{
                            occupancyGrid->data.at(x + y * occupancyGrid->info.height)=map_->getHeight(x + y * occupancyGrid->info.height)*factor;
                        }
//                      occupancyGrid->data.at(x + y * occupancyGrid->info.height)=0;
                    }else{
                        occupancyGrid->data.at(x + y * occupancyGrid->info.height)=100;
                    }

            }
        }

        occupancyGrid_pub_.publish(occupancyGrid);
        std::cout<<"b2"<<std::endl;
        costmap_2d::Costmap2DROS* costmap_ros_ (new costmap_2d::Costmap2DROS("testSBPLStairsPlanner", tf_)); /**< manages the cost map for us */
          std::cout<<"b22"<<std::endl;
//        costmap_ros_->getCostmap()->resizeMap( occupancyGrid->info.width= map_->getInfo().height, occupancyGrid->info.height=map_->getInfo().width,  occupancyGrid->info.width= map_->getInfo().resolution, occupancyGrid->info.origin.position.x, occupancyGrid->info.origin.position.y);
          costmap_ros_->getCostmap()->resizeMap( 10, 20,  0.05, 0,0);
          std::cout<<"b3"<<std::endl;
        for (ssize_t ix(0); ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ++ix)
            for (ssize_t iy(0); iy < costmap_ros_->getCostmap()->getSizeInCellsY(); ++iy)
                costmap_ros_->getCostmap()->setCost(ix, iy, occupancyGrid->data.at(ix + iy * occupancyGrid->info.width));

std::cout<<"c"<<std::endl;

        planner_ = new HectorSbplStairsPlanner();
        planner_->initialize("hector_sbpl_flipper_contorl_planner", costmap_ros_);

        //test own A_star
//        own_planner_= new a_star::A_Star(*occupancyGrid);

        ROS_INFO("PlannerFlipperControlTest node created");

    }

    void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        ROS_INFO("Goal pose callback");


        //Start always at 0.0, 0.0 for now;

        geometry_msgs::PoseStamped start_pose;
        start_pose.header.frame_id="/map";
        start_pose.pose.position.x=0;
        start_pose.pose.position.y=0;
        start_pose.pose.position.z=0;
        start_pose.pose.orientation.x=0;
        start_pose.pose.orientation.y=0;
        start_pose.pose.orientation.z=0;
        start_pose.pose.orientation.w=1;

        ROS_INFO("Goal pose callback1");

        ROS_INFO("Goal pose callback2");
        planner_->makePlan(start_pose, *msg, path_.poses);
//        own_planner_->makePlan(start_pose, *msg, path_.poses);

        ROS_INFO("Goal pose callback3");

        path_.header.stamp = ros::Time::now();

        drivepath_pub_.publish(path_);

        ROS_INFO("Goal pose callbackEND");
//        std::cout<<path_<<std::endl;
}

protected:

    HectorSbplStairsPlanner* planner_;
//a_star::A_Star* own_planner_;
    ros::ServiceServer exploration_plan_service_server_;

    gridmap::GridMap *map_;

    nav_msgs::Path path_;

    ros::Publisher drivepath_pub_;
    ros::Publisher occupancyGrid_pub_;

    ros::Subscriber goal_pose_sub_;
    tf::TransformListener tf_;


};

int main(int argc, char **argv) {
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    PlannerFlipperControlTest pt;

    ros::spin();

    return 0;
}
