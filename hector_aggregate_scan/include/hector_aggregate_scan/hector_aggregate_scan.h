#ifndef HECTOR_AGGREGATE_SCAN_H
#define HECTOR_AGGREGATE_SCAN_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>

namespace hector_aggregate_scan{

    class HectorAggregateScan {

    public:
         HectorAggregateScan();
         virtual ~HectorAggregateScan();
         
         void pclCB(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);

    protected:
          ros::NodeHandle nh_;
          
          ros::Publisher pointcloud_pub_;
          ros::Publisher inputcloud_pub_debug_;
          ros::Publisher point_cloud2_pub_;
          ros::Subscriber scan_sub_;
          ros::Subscriber mulit_scan_sub_;

          sensor_msgs::PointCloud2 cloud2_;
          ros::Timer timer_;
          tf::TransformListener listener_;

          boost::shared_ptr<tf::TransformListener> tfl_;
          ros::Duration wait_duration_;

          laser_geometry::LaserProjection projector_;

          pcl::PointCloud<pcl::PointXYZ> aggregated_cloud_;

          void timerCallback(const ros::TimerEvent& event);
          void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
    };
}

#endif
