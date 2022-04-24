#ifndef _OBJECT_FILTER_H_
#define _OBJECT_FILTER_H_

#include <iostream>
#include <vector>
#include <string>
#include <queue>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

typedef geometry_msgs::PoseWithCovarianceStamped ObjectPoseType;
typedef sensor_msgs::PointCloud2 PointCloudType;

void SetNodeParameters(const ros::NodeHandle& node);
class DynamicObject
{
private:
    Eigen::Vector4f max_vec_;
    Eigen::Vector4f min_vec_;

    

public:

    void SetObjBBox();
    void FilterObject(pcl::PCLPointCloud2& point_cloud);

    void MsgCallback(const ObjectPoseType::ConstPtr& obj_pose_msg);
};

/**************************
 ObjectsFiltering class to 
 encapsulate and process
 objects and filtering.
 **************************/
class ObjectsFiltering
{
private:
    ros::Publisher filtered_cloud_pub_;
    ros::Subscriber point_cloud_sub_;

public:
    ObjectsFiltering(ros::NodeHandle node);
    ~ObjectsFiltering();

    void CloudCallBack(const PointCloudType::ConstPtr& point_cloud_msg);
};

ObjectsFiltering::ObjectsFiltering(ros::NodeHandle node)
{
    point_cloud_sub_ = node.subscribe("raw_point_cloud", 10, &ObjectsFiltering::CloudCallBack, this);
    filtered_cloud_pub_ = node.advertise<PointCloudType>("filtered_point_cloud", 10);
}

ObjectsFiltering::~ObjectsFiltering()
{
    
}

void ObjectsFiltering::CloudCallBack(const PointCloudType::ConstPtr& point_cloud_msg)
{

    pcl::PCLPointCloud2 pcl_point_cloud;
    pcl_conversions::toPCL(*point_cloud_msg, pcl_point_cloud);
    sensor_msgs::PointCloud2 filtered_point_cloud;
    pcl_conversions::moveFromPCL(pcl_point_cloud, filtered_point_cloud); // Test water
    std::cout << filtered_point_cloud.width << std::endl;
    filtered_cloud_pub_.publish(filtered_point_cloud);
}

#endif //_OBJECT_FILTER_H_