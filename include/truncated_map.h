#ifndef TRUNCATED_MAP_H
#define TRUNCATED_MAP_H

#include <iostream>
#include <vector>
#include <string>
#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#define SUBSCRIBE_TO_POINTCLOUD2 0

class TruncatedMap 
{
private:
    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher filtered_cloud_pub_;

    // Box filter parameters
    Eigen::Vector4f min_point_vec_;
    Eigen::Vector4f max_point_vec_;
    
    std::queue<nav_msgs::Odometry> odom_queue_;

public:
    TruncatedMap(ros::NodeHandle nh);
    ~TruncatedMap();

    #if SUBSCRIBE_TO_POINTCLOUD2
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    #else
    void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& cloud_msg);
    #endif
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    void SyncOdom(const sensor_msgs::PointCloud &ros_point_cloud, Eigen::Vector3f& sync_position);
    void CropOutVolume(const sensor_msgs::PointCloud &ros_point_cloud, pcl::PCLPointCloud2Ptr input_cloud, pcl::PCLPointCloud2& output_cloud);

};

#endif // TRUNCATED_MAP_H