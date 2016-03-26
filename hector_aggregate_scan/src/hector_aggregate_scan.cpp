#include <ros/ros.h>
#include <hector_aggregate_scan/hector_aggregate_scan.h>

namespace hector_aggregate_scan{
HectorAggregateScan::HectorAggregateScan(){
    tfl_.reset(new tf::TransformListener());
    wait_duration_ = ros::Duration(0.5);

    mulit_scan_sub_ = nh_.subscribe("/multi_sensor_scan", 10, &HectorAggregateScan::scanCallback, this);
    point_cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/mulitsensor_scan/scan_cloud",100,true);
    scan_sub_ = nh_.subscribe("/mulitsensor_scan/scan_cloud", 100, &HectorAggregateScan::pclCB, this);
    pointcloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/hector_aggregate_cloud/aggregated_cloud", 100, true);
    inputcloud_pub_debug_= nh_.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/hector_aggregate_cloud/input_cloud_debug", 100, true);
    timer_ = nh_.createTimer(ros::Duration(5.0), &HectorAggregateScan::timerCallback, this);
    aggregated_cloud_.header.frame_id= "/map";
}

HectorAggregateScan::~HectorAggregateScan()
{}

void HectorAggregateScan::pclCB(const sensor_msgs::PointCloud2::ConstPtr& pc_msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pc_msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*input_cloud);

    //transform cloud to /map
    Eigen::Affine3d to_map;
    tf::StampedTransform transform_cloud_to_map;
    try{
        ros::Time time = pc_msg->header.stamp;
        listener_.waitForTransform("/map", pc_msg->header.frame_id,
                                   time, ros::Duration(3.0));
        listener_.lookupTransform("/map", pc_msg->header.frame_id,
                                  time, transform_cloud_to_map);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("Lookup Transform failed: %s",ex.what());
        return;
    }

    tf::transformTFToEigen(transform_cloud_to_map, to_map);

    // Transform to /world
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*input_cloud, *cloud_tmp, to_map);
    input_cloud = cloud_tmp;
    input_cloud->header.frame_id= transform_cloud_to_map.frame_id_;


    for(int i=0; i<input_cloud->size(); i++){
        pcl::PointXYZ pt;
        pt.x=input_cloud->at(i).x;
        pt.y=input_cloud->at(i).y;
        pt.z=input_cloud->at(i).z;
        aggregated_cloud_.push_back(pt);
    }
}

void HectorAggregateScan::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    if(!listener_.waitForTransform(scan_in->header.frame_id, "/map", scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(1.0))){
         return;
      }

      projector_.transformLaserScanToPointCloud("/map",*scan_in,cloud2_,listener_);

  if (cloud2_.data.size() > 0){
    point_cloud2_pub_.publish(cloud2_);
  }
}

void HectorAggregateScan::timerCallback(const ros::TimerEvent& event)
{
    pointcloud_pub_.publish(aggregated_cloud_);
}
}
