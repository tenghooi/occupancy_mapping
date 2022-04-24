#ifndef _OBJECT_FILTER_H_
#define _OBJECT_FILTER_H_

#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <memory>

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

void SetNodeParameters(const ros::NodeHandle& node,
                       DynamicObject& objA,
                       DynamicObject& objB,
                       DynamicObject& objC);

/**************************
 DynamicObject class to 
 set object BBox and filtering
 **************************/
class DynamicObject
{
private:
    Eigen::Vector4f max_vec_;
    Eigen::Vector4f min_vec_;

public:
    DynamicObject();
    ~DynamicObject();

    void SetObjBBox();
    void FilterObject(pcl::PCLPointCloud2& point_cloud);
};

DynamicObject::DynamicObject(){};
DynamicObject::~DynamicObject(){};

void DynamicObject::SetObjBBox()
{

}

void DynamicObject::FilterObject(pcl::PCLPointCloud2& point_cloud)
{

}

/**************************
 ObjectsFiltering class to 
 encapsulate and process
 objects and filtering.
 **************************/
class ObjectsFiltering
{
private:
    std::shared_ptr<DynamicObject> objA = std::make_shared<DynamicObject>();
    std::shared_ptr<DynamicObject> objB = std::make_shared<DynamicObject>();
    std::shared_ptr<DynamicObject> objC = std::make_shared<DynamicObject>();

    ros::Publisher filtered_cloud_pub_;

    ros::Subscriber point_cloud_sub_;
    ros::Subscriber objA_pose_sub_;
    ros::Subscriber objB_pose_sub_;
    ros::Subscriber objC_pose_sub_;

public:
    ObjectsFiltering(ros::NodeHandle node);
    ~ObjectsFiltering();

    void CloudCallBack(const PointCloudType::ConstPtr& point_cloud_msg);
    void ObjAPoseCallBack(const ObjectPoseType::ConstPtr& objA_pose_msg);
    void ObjBPoseCallBack(const ObjectPoseType::ConstPtr& objB_pose_msg);
    void ObjCPoseCallBack(const ObjectPoseType::ConstPtr& objC_pose_msg);
};

ObjectsFiltering::ObjectsFiltering(ros::NodeHandle node)
{
    SetNodeParameters(node, objA, objB, objC);

    point_cloud_sub_ = node.subscribe("raw_point_cloud", 10, &ObjectsFiltering::CloudCallBack, this);
    objA_pose_sub_ = node.subscribe("objA_pose", 10, &ObjectsFiltering::ObjAPoseCallBack, this);
    objB_pose_sub_ = node.subscribe("objB_pose", 10, &ObjectsFiltering::ObjBPoseCallBack, this);
    objC_pose_sub_ = node.subscribe("objC_pose", 10, &ObjectsFiltering::ObjCPoseCallBack, this);

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