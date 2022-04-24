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

    void SetObjBBox(double& min_x, double& min_y, double& min_z,
                    double& max_x, double& max_y, double& max_z);
    void FilterObject(pcl::PCLPointCloud2& point_cloud);
};

DynamicObject::DynamicObject(){};
DynamicObject::~DynamicObject(){};

void DynamicObject::SetObjBBox(double& min_x, double& min_y, double& min_z,
                               double& max_x, double& max_y, double& max_z)
{
    min_vec_ << min_x, min_y, min_z, 1.0;
    max_vec_ << max_x, max_y, max_z, 1.0;
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
    DynamicObject objA_;
    DynamicObject objB_;
    DynamicObject objC_;

    std::queue<ObjectPoseType>objA_queue_;
    std::queue<ObjectPoseType>objB_queue_;
    std::queue<ObjectPoseType>objC_queue_;

    ros::Publisher filtered_cloud_pub_;

    ros::Subscriber point_cloud_sub_;
    ros::Subscriber objA_pose_sub_;
    ros::Subscriber objB_pose_sub_;
    ros::Subscriber objC_pose_sub_;

public:
    ObjectsFiltering(ros::NodeHandle node);
    ~ObjectsFiltering();

    void FilterObjects(const PointCloudType& point_cloud);

    void CloudCallBack(const PointCloudType::ConstPtr& point_cloud_msg);
    void ObjAPoseCallBack(const ObjectPoseType::ConstPtr& objA_pose_msg);
    void ObjBPoseCallBack(const ObjectPoseType::ConstPtr& objB_pose_msg);
    void ObjCPoseCallBack(const ObjectPoseType::ConstPtr& objC_pose_msg);
};

ObjectsFiltering::ObjectsFiltering(ros::NodeHandle node)
{
    SetNodeParameters(node, objA_, objB_, objC_);

    point_cloud_sub_ = node.subscribe("raw_point_cloud", 10, &ObjectsFiltering::CloudCallBack, this);
    objA_pose_sub_ = node.subscribe("objA_pose", 10, &ObjectsFiltering::ObjAPoseCallBack, this);
    objB_pose_sub_ = node.subscribe("objB_pose", 10, &ObjectsFiltering::ObjBPoseCallBack, this);
    objC_pose_sub_ = node.subscribe("objC_pose", 10, &ObjectsFiltering::ObjCPoseCallBack, this);

    filtered_cloud_pub_ = node.advertise<PointCloudType>("filtered_point_cloud", 10);
}

ObjectsFiltering::~ObjectsFiltering(){}

void ObjectsFiltering::FilterObjects(const PointCloudType& point_cloud)
{
    ros::Duration time_tolerance(1e-3); // 1ms
    ros::Time point_cloud_time = point_cloud.header.stamp;
    ROS_INFO("Point cloud msg time: %lf", point_cloud_time.toSec());

    pcl::PCLPointCloud2 pcl_point_cloud;
    pcl_conversions::toPCL(point_cloud, pcl_point_cloud);

    Eigen::Vector3d sync_pos_;

    while(objA_queue_.size() > 1 &&
          objA_queue_.front().header.stamp <= point_cloud_time + time_tolerance)
    {
        sync_pos_ << objA_queue_.front().pose.pose.position.x,
                     objA_queue_.front().pose.pose.position.y,
                     objA_queue_.front().pose.pose.position.z;

        objA_queue_.pop();
    }
}

void ObjectsFiltering::CloudCallBack(const PointCloudType::ConstPtr& point_cloud_msg)
{

    //pcl::PCLPointCloud2 pcl_point_cloud;
    //pcl_conversions::toPCL(*point_cloud_msg, pcl_point_cloud);
    FilterObjects(*point_cloud_msg);

    //sensor_msgs::PointCloud2 filtered_point_cloud;
    //pcl_conversions::moveFromPCL(pcl_point_cloud, filtered_point_cloud);
    //std::cout << filtered_point_cloud.width << std::endl;
    //filtered_cloud_pub_.publish(filtered_point_cloud);
}

void ObjectsFiltering::ObjAPoseCallBack(const ObjectPoseType::ConstPtr& objA_pose_msg)
{
    ObjectPoseType objA_pose;
    objA_pose.header = objA_pose_msg->header;
    objA_pose.pose = objA_pose_msg->pose;

    objA_queue_.push(objA_pose);
}

void ObjectsFiltering::ObjBPoseCallBack(const ObjectPoseType::ConstPtr& objB_pose_msg)
{
    ObjectPoseType objB_pose;
    objB_pose.header = objB_pose_msg->header;
    objB_pose.pose = objB_pose_msg->pose;

    objA_queue_.push(objB_pose);
}

void ObjectsFiltering::ObjCPoseCallBack(const ObjectPoseType::ConstPtr& objC_pose_msg)
{
    ObjectPoseType objC_pose;
    objC_pose.header = objC_pose_msg->header;
    objC_pose.pose = objC_pose_msg->pose;

    objC_queue_.push(objC_pose);
}

#endif //_OBJECT_FILTER_H_